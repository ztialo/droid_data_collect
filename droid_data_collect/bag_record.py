#!/usr/bin/env python3

import os
import signal
import subprocess
import threading
import time
import tkinter as tk
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Int32, Bool


class ButtonBagToggle(Node):
    def __init__(self) -> None:
        super().__init__('button_bag_toggle')

        self.declare_parameter('a_button_topic', '/quest/right/a_button')
        self.declare_parameter('joystick_click_topic', '/quest/right/joystick_click')
        self.declare_parameter('output_base_dir', '/media/tactilemanipulationlab/ext_linux')
        self.declare_parameter('topic_mode', 'full')
        # self.declare_parameter('topic_mode', 'test')
        self.a_button_topic = self.get_parameter('a_button_topic').get_parameter_value().string_value
        self.joystick_click_topic = self.get_parameter('joystick_click_topic').get_parameter_value().string_value
        self.output_base_dir = self.get_parameter('output_base_dir').get_parameter_value().string_value
        self.topic_mode = self.get_parameter('topic_mode').get_parameter_value().string_value
        if self.topic_mode == 'full':
            self.topics = ['/droid/wrist_image_left/compressed', '/external_rgb/compressed', '/droid/joint_state', '/droid/gripper_position']
        elif self.topic_mode == 'test':
            self.topics = ['/external_rgb/compressed']
        if not self.topics:
            self.get_logger().error("Parameter 'topics' is empty; bag recording cannot start.")

        self._record_proc: Optional[subprocess.Popen] = None
        self._prev_button_count: Optional[int] = None
        self._state_lock = threading.Lock()
        self._ui_state = 'init'
        self._ui_message = ''
        self._record_start_monotonic: Optional[float] = None
        self._last_duration_sec = 0.0
        self._finished_until_monotonic: Optional[float] = None
        self._delete_arm_min_recording_sec = 0.5
        self._topic_check_period_sec = 0.5
        self._topic_msg_timeout_sec = 1.5
        self._topic_ready = {topic: False for topic in self.topics}
        self._topic_last_msg_monotonic = {topic: None for topic in self.topics}
        self._topic_subscriptions = {}
        self._topic_type_names = {}

        self.subscription = self.create_subscription(
            Int32,
            self.a_button_topic,
            self._a_button_callback,
            10,
        )
        self.joystick_subscription = self.create_subscription(
            Bool,
            self.joystick_click_topic,
            self._joystick_click_callback,
            10,
        )
        self._state_timer = self.create_timer(0.05, self._state_tick)
        self._topic_timer = self.create_timer(self._topic_check_period_sec, self._refresh_topic_ready)
        self._refresh_topic_ready()

        self.bag_path = None
        self.last_click_state = None
        self._delete_armed_on_click = False

    def _a_button_callback(self, msg: Int32) -> None:
        count = msg.data
        if self._prev_button_count is None:
            self._prev_button_count = count
            return

        if count > self._prev_button_count:
            # This topic is an incrementing press counter; each odd number of
            # new presses flips state once.
            if (count - self._prev_button_count) % 2 == 1:
                self._toggle_recording()
        self._prev_button_count = count

    def _joystick_click_callback(self, msg: Bool) -> None:
        prev_click_state = self.last_click_state
        self.last_click_state = msg.data

        if prev_click_state is None:
            return

        with self._state_lock:
            current_state = self._ui_state

        # No action unless we are actively recording.
        if current_state != 'recording':
            self._delete_armed_on_click = False
            return

        # Arm delete on press while recording.
        if msg.data and not prev_click_state:
            with self._state_lock:
                start_time = self._record_start_monotonic
            if start_time is None:
                self._delete_armed_on_click = False
                return
            if (time.monotonic() - start_time) < self._delete_arm_min_recording_sec:
                # Ignore press events that coincide with recording startup.
                self._delete_armed_on_click = False
                return
            self._delete_armed_on_click = True
            return

        # Delete only on click completion: release after press.
        if not ((not msg.data) and prev_click_state and self._delete_armed_on_click):
            return

        self._delete_armed_on_click = False

        proc = self._record_proc
        if proc is None:
            return

        self._record_proc = None
        self.get_logger().info('Joystick click detected; stopping and deleting current recording.')
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('rosbag did not exit after SIGINT; terminating process.')
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('rosbag did not terminate; killing process.')
                proc.kill()
                proc.wait(timeout=2)

        if self.bag_path and os.path.exists(self.bag_path):
            try:
                subprocess.run(['rm', '-rf', self.bag_path], check=True)
                self.get_logger().info(f'Deleted bag at {self.bag_path}')
            except Exception as exc:
                self.get_logger().error(f'Failed to delete bag at {self.bag_path}: {exc}')

        with self._state_lock:
            now = time.monotonic()
            if self._record_start_monotonic is not None:
                self._last_duration_sec = max(0.0, now - self._record_start_monotonic)
            self._record_start_monotonic = None
            self._ui_state = 'deleted'
            self._finished_until_monotonic = now + 2.0

    def _toggle_recording(self) -> None:
        with self._state_lock:
            state = self._ui_state
            proc_is_none = self._record_proc is None
            can_start = (state == 'ready' and proc_is_none)
            can_stop = (state == 'recording' and not proc_is_none)

        if can_start:
            self._start_recording()
        elif can_stop:
            self._stop_recording()
        else:
            with self._state_lock:
                missing_topics = [topic for topic in self.topics if not self._topic_ready.get(topic, False)]
            message = (
                f'Cannot start: state={state}, active_proc={not proc_is_none}, '
                f'missing={missing_topics if missing_topics else "none"}'
            )
            with self._state_lock:
                self._ui_message = message
            self.get_logger().warn(
                f'Cannot start recording (state={state}, record_proc_none={proc_is_none}). '
                f'Missing topics: {missing_topics if missing_topics else "none"}'
            )

    def _start_recording(self) -> None:
        if not self.topics:
            return

        os.makedirs(self.output_base_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.bag_path = os.path.join(self.output_base_dir, f'session_{stamp}')

        cmd = ['ros2', 'bag', 'record', '-o', self.bag_path, *self.topics]
        try:
            self._record_proc = subprocess.Popen(cmd)
        except Exception as exc:
            self._record_proc = None
            self.get_logger().error(f'Failed to start rosbag record: {exc}')
            return

        with self._state_lock:
            self._ui_state = 'recording'
            self._ui_message = ''
            self._record_start_monotonic = time.monotonic()
            self._last_duration_sec = 0.0
            self._finished_until_monotonic = None

        self.get_logger().info(f'Started recording to {self.bag_path}')

    def _stop_recording(self) -> None:
        if self._record_proc is None:
            return

        proc = self._record_proc
        self._record_proc = None

        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('rosbag did not exit after SIGINT; terminating process.')
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('rosbag did not terminate; killing process.')
                proc.kill()
                proc.wait(timeout=2)

        with self._state_lock:
            now = time.monotonic()
            if self._record_start_monotonic is not None:
                self._last_duration_sec = max(0.0, now - self._record_start_monotonic)
            self._record_start_monotonic = None
            self._ui_state = 'finished'
            self._finished_until_monotonic = now + 2.0

        self.get_logger().info('Stopped recording.')

    def _state_tick(self) -> None:
        with self._state_lock:
            if (self._ui_state == 'finished' or self._ui_state == 'deleted') and self._finished_until_monotonic is not None:
                if time.monotonic() >= self._finished_until_monotonic:
                    self._ui_state = 'init'
                    self._finished_until_monotonic = None
                    self._last_duration_sec = 0.0
        
        ready = self._ready_check()
        with self._state_lock:
            if self._ui_state == 'init' and ready:
                self._ui_state = 'ready'
            elif self._ui_state == 'ready' and not ready:
                self._ui_state = 'init'

    def _topic_monitor_callback(self, topic: str) -> None:
        with self._state_lock:
            if self._ui_state not in ('init', 'ready'):
                return
            self._topic_last_msg_monotonic[topic] = time.monotonic()

    def _ensure_topic_subscription(self, topic: str, topic_types: dict[str, list[str]]) -> None:
        if topic in self._topic_subscriptions:
            return

        if topic not in topic_types or not topic_types[topic]:
            return

        type_name = topic_types[topic][0]
        try:
            msg_type = get_message(type_name)
            self._topic_subscriptions[topic] = self.create_subscription(
                msg_type,
                topic,
                lambda _msg, t=topic: self._topic_monitor_callback(t),
                10,
            )
            self._topic_type_names[topic] = type_name
        except Exception as exc:
            self.get_logger().warn(f'Failed to monitor topic {topic} ({type_name}): {exc}')

    def _refresh_topic_ready(self) -> None:
        with self._state_lock:
            if self._ui_state not in ('init', 'ready'):
                return

        topic_types = {name: types for name, types in self.get_topic_names_and_types()}
        now = time.monotonic()

        for topic in self.topics:
            self._ensure_topic_subscription(topic, topic_types)

        with self._state_lock:
            for topic in self.topics:
                has_publisher = topic in topic_types
                last_msg = self._topic_last_msg_monotonic.get(topic)
                has_recent_msg = (
                    last_msg is not None
                    and (now - last_msg) <= self._topic_msg_timeout_sec
                )
                self._topic_ready[topic] = has_publisher and has_recent_msg

    def _ready_check(self) -> bool:
        with self._state_lock:
            return all(self._topic_ready.get(topic, False) for topic in self.topics)

    def get_ui_snapshot(self) -> tuple[str, float, str, list[tuple[str, bool]]]:
        with self._state_lock:
            state = self._ui_state
            message = self._ui_message
            topic_status = [(topic, self._topic_ready.get(topic, False)) for topic in self.topics]
            if state == 'recording' and self._record_start_monotonic is not None:
                duration = max(0.0, time.monotonic() - self._record_start_monotonic)
            elif state == 'finished':
                duration = self._last_duration_sec
            else:
                duration = 0.0
            return state, duration, message, topic_status

    def destroy_node(self) -> bool:
        if self._record_proc is not None:
            self.get_logger().info('Node shutting down; stopping active recording.')
            self._stop_recording()
        return super().destroy_node()


class RecorderStatusWindow:
    COLORS = {
        'init': '#7a7a7a',
        'ready': "#2d6cdf",
        'recording': "#95e268",
        'finished': "#1f7c39",
        'deleted': "#ca5860",
    }
    LABELS = {
        'init': 'INIT',
        'ready': 'READY',
        'recording': 'RECORDING',
        'finished': 'FINISHED',
        'deleted': 'DELETED',
    }

    def __init__(self, node: ButtonBagToggle) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title('Bag Recorder')
        self.root.geometry('900x620')
        self.root.minsize(700, 460)
        self.root.resizable(True, True)
        self.root.configure(bg='#111827')
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        self.container = tk.Frame(self.root, bg='#111827')
        self.container.grid(row=0, column=0, sticky='nsew')
        self.container.grid_rowconfigure(0, weight=3)
        self.container.grid_rowconfigure(1, weight=2)
        self.container.grid_rowconfigure(2, weight=1)
        self.container.grid_rowconfigure(3, weight=1)
        self.container.grid_rowconfigure(4, weight=1)
        self.container.grid_columnconfigure(0, weight=1)

        self.status = tk.Label(
            self.container,
            text='INIT',
            font=('Helvetica', 82, 'bold'),
            fg='white',
            bg=self.COLORS['init'],
            padx=30,
            pady=24,
        )
        self.status.grid(row=0, column=0, sticky='nsew', padx=34, pady=(34, 22))

        self.timer = tk.Label(
            self.container,
            text='00:00.0',
            font=('Helvetica', 88, 'bold'),
            fg='white',
            bg='#111827',
        )
        self.timer.grid(row=1, column=0, sticky='n', pady=(0, 10))

        self.hint = tk.Label(
            self.container,
            text='Press A button to start/stop recording\n' \
            'Press joystick button to delete recording',
            font=('Helvetica', 24),
            fg='#d1d5db',
            bg='#111827',
        )
        self.hint.grid(row=2, column=0, sticky='n', pady=(0, 30))

        self.message = tk.Label(
            self.container,
            text='',
            font=('Helvetica', 18),
            fg='#fca5a5',
            bg='#111827',
            wraplength=820,
            justify='center',
        )
        self.message.grid(row=3, column=0, sticky='n', pady=(0, 20))

        self.topic_frame = tk.Frame(self.container, bg='#111827')
        self.topic_frame.grid(row=4, column=0, sticky='ew', padx=24, pady=(0, 18))
        self.topic_blocks = {}
        for idx, topic in enumerate(self.node.topics):
            self.topic_frame.grid_columnconfigure(idx, weight=1)
            label = tk.Label(
                self.topic_frame,
                text=topic,
                font=('Helvetica', 12, 'bold'),
                fg='white',
                bg='#6b7280',
                padx=10,
                pady=8,
            )
            label.grid(row=0, column=idx, sticky='ew', padx=6)
            self.topic_blocks[topic] = label

        self.root.bind('<Configure>', self._on_resize)

    def _format_duration(self, sec: float) -> str:
        minutes = int(sec // 60)
        seconds = int(sec % 60)
        tenth = int((sec - int(sec)) * 10)
        return f'{minutes:02d}:{seconds:02d}.{tenth}'

    def _refresh(self) -> None:
        state, duration, message, topic_status = self.node.get_ui_snapshot()
        self.status.config(text=self.LABELS[state], bg=self.COLORS[state])
        self.timer.config(text=self._format_duration(duration))
        self.message.config(text=message)
        for topic, is_ready in topic_status:
            block = self.topic_blocks.get(topic)
            if block is not None:
                block.config(bg='#2da44e' if is_ready else '#6b7280')
        self.root.after(100, self._refresh)

    def _on_resize(self, _event) -> None:
        width = self.root.winfo_width()
        height = self.root.winfo_height()

        status_size = max(42, min(140, int(min(width * 0.10, height * 0.16))))
        timer_size = max(40, min(152, int(min(width * 0.11, height * 0.17))))
        hint_size = max(14, min(34, int(min(width * 0.028, height * 0.045))))
        message_size = max(10, min(24, int(min(width * 0.02, height * 0.032))))
        topic_size = max(9, min(16, int(min(width * 0.014, height * 0.022))))

        pad_x = max(18, int(width * 0.035))
        pad_top = max(18, int(height * 0.05))
        pad_mid = max(10, int(height * 0.03))
        pad_bottom = max(18, int(height * 0.04))

        self.status.config(font=('Helvetica', status_size, 'bold'))
        self.timer.config(font=('Helvetica', timer_size, 'bold'))
        self.hint.config(font=('Helvetica', hint_size))
        self.message.config(font=('Helvetica', message_size))
        for block in self.topic_blocks.values():
            block.config(font=('Helvetica', topic_size, 'bold'))
        self.status.grid_configure(padx=pad_x, pady=(pad_top, pad_mid))
        self.hint.grid_configure(pady=(0, pad_bottom))

    def run(self) -> None:
        self._refresh()
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ButtonBagToggle()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        window = RecorderStatusWindow(node)
        window.run()
    except KeyboardInterrupt:
        pass
    except tk.TclError as exc:
        node.get_logger().error(f'Failed to create UI window: {exc}')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
