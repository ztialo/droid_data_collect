# Source your ROS 2 workspaces once here so env is ready
source ~/ros2_ws/install/setup.bash

# Open ZED rgb camera node in a new terminal
gnome-terminal --tab -- bash -c "ros2 launch zed_wrapper zed_rgb_only.launch.py camera_model:=zed2i; exec bash"

sleep 0.5

# Subscribe to zed rgb topic and publish in compressed format for recording
gnome-terminal --tab -- bash -c "ros2 launch droid_data_collect subscriber.launch.py; exec bash"

# External RGB publisher
gnome-terminal --tab -- bash -c "ros2 run policy_training external_rgb_publisher; exec bash"

# Launch the bag record launch file 
gnome-terminal --tab -- bash -c "ros2 launch droid_data_collect bag_record.launch.py; exec bash"