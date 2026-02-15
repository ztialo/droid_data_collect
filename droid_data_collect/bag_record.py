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
from std_msgs.msg import Int32, Bool


class ButtonBagToggle(Node):
    def __init__(self) -> None:
        super().__init__('button_bag_toggle')

        self.declare_parameter('a_button_topic', '/quest/right/a_button')
        self.declare_parameter('joystick_click_topic', '/quest/right/joystick_click')
        self.declare_parameter('output_base_dir', '/media/tactilemanipulationlab/ext_linux')
        self.declare_parameter('topics', ['/droid/wrist_image_left', '/external_rgb/compressed', '/droid/joint_states', '/droid/gripper_position'])

        self.a_button_topic = self.get_parameter('a_button_topic').get_parameter_value().string_value
        self.joystick_click_topic = self.get_parameter('joystick_click_topic').get_parameter_value().string_value
        self.output_base_dir = self.get_parameter('output_base_dir').get_parameter_value().string_value
        self.topics = list(self.get_parameter('topics').get_parameter_value().string_array_value)

        if not self.topics:
            self.get_logger().error("Parameter 'topics' is empty; bag recording cannot start.")

        self._record_proc: Optional[subprocess.Popen] = None
        self._prev_button_count: Optional[int] = None
        self._state_lock = threading.Lock()
        self._ui_state = 'init'
        self._record_start_monotonic: Optional[float] = None
        self._last_duration_sec = 0.0
        self._finished_until_monotonic: Optional[float] = None

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

        self.get_logger().info(
            f"Listening on {self.a_button_topic}. Press A to toggle bag recording."
        )

        self.bag_path = None
        self.last_click_state = None

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

        # Delete only on click completion: release edge (True -> False).
        if not ((not msg.data) and prev_click_state):
            return

        if self._record_proc is None:
            return
        # if the joystick click state changed, delete recording
        if self._ui_state == 'recording':
            proc = self._record_proc
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
        if self._record_proc is None:
            self._start_recording()
        else:
            self._stop_recording()

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
            

    def get_ui_snapshot(self) -> tuple[str, float]:
        with self._state_lock:
            state = self._ui_state
            if state == 'recording' and self._record_start_monotonic is not None:
                duration = max(0.0, time.monotonic() - self._record_start_monotonic)
            elif state == 'finished':
                duration = self._last_duration_sec
            else:
                duration = 0.0
            return state, duration

    def destroy_node(self) -> bool:
        if self._record_proc is not None:
            self.get_logger().info('Node shutting down; stopping active recording.')
            self._stop_recording()
        return super().destroy_node()


class RecorderStatusWindow:
    COLORS = {
        'init': '#7a7a7a',
        'recording': '#2d6cdf',
        'finished': '#2da44e',
        'deleted': '#cf222e',
    }
    LABELS = {
        'init': 'INIT',
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
            text='Press A button to start/stop recording',
            font=('Helvetica', 24),
            fg='#d1d5db',
            bg='#111827',
        )
        self.hint.grid(row=2, column=0, sticky='n', pady=(0, 30))

        self.root.bind('<Configure>', self._on_resize)

    def _format_duration(self, sec: float) -> str:
        minutes = int(sec // 60)
        seconds = int(sec % 60)
        tenth = int((sec - int(sec)) * 10)
        return f'{minutes:02d}:{seconds:02d}.{tenth}'

    def _refresh(self) -> None:
        state, duration = self.node.get_ui_snapshot()
        self.status.config(text=self.LABELS[state], bg=self.COLORS[state])
        self.timer.config(text=self._format_duration(duration))
        self.root.after(100, self._refresh)

    def _on_resize(self, _event) -> None:
        width = self.root.winfo_width()
        height = self.root.winfo_height()

        status_size = max(42, min(140, int(min(width * 0.10, height * 0.16))))
        timer_size = max(40, min(152, int(min(width * 0.11, height * 0.17))))
        hint_size = max(14, min(34, int(min(width * 0.028, height * 0.045))))

        pad_x = max(18, int(width * 0.035))
        pad_top = max(18, int(height * 0.05))
        pad_mid = max(10, int(height * 0.03))
        pad_bottom = max(18, int(height * 0.04))

        self.status.config(font=('Helvetica', status_size, 'bold'))
        self.timer.config(font=('Helvetica', timer_size, 'bold'))
        self.hint.config(font=('Helvetica', hint_size))
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
