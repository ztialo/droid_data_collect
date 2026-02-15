# droid_data_collect

ROS 2 Python package template for:
- subscribing to a topic (starter node)
- launching subscription with params
- recording a topic with rosbag2

## Package layout
- `droid_data_collect/topic_subscriber.py`: starter subscriber node (`std_msgs/msg/String`)
- `launch/subscriber.launch.py`: launches the subscriber with `config/topics.yaml`
- `launch/bag_record.launch.py`: records one topic to rosbag2
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

## Record bag
```bash
ros2 launch droid_data_collect bag_record.launch.py topic:=/chatter output_dir:=my_bag
```

## Next customization
- Change message type in `droid_data_collect/topic_subscriber.py` from `std_msgs/msg/String` to your target type.
- Add additional subscriber nodes or timer callbacks for data collection logic.
- Expand bag launch to include multiple topics when needed.
