# droid_data_collect

ROS 2 Python package template for:
- subscribing to a topic (starter node)
- launching subscription with params
- button-triggered rosbag recording with a standalone node

## Package layout
- `droid_data_collect/topic_subscriber.py`: starter subscriber node (`std_msgs/msg/String`)
- `droid_data_collect/button_bag_toggle.py`: standalone A-button toggle recorder node
- `launch/subscriber.launch.py`: launches the subscriber with `config/topics.yaml`
- `config/topics.yaml`: subscriber parameters

## Build
From your ROS 2 workspace root:

```bash
colcon build --packages-select droid_data_collect
source install/setup.bash
```

## Run subscriber
```bash
ros2 launch droid_data_collect subscriber.launch.py
```

To test quickly:
```bash
ros2 topic pub /chatter std_msgs/msg/String "{data: 'hello'}" -r 1
```

## Run bag toggle recorder
```bash
ros2 run droid_data_collect button_bag_toggle
```

## Next customization
- Change message type in `droid_data_collect/topic_subscriber.py` from `std_msgs/msg/String` to your target type.
- Add additional subscriber nodes or timer callbacks for data collection logic.
- Tune `button_bag_toggle` parameters (`button_topic`, `output_base_dir`, `topics`) for your setup.
