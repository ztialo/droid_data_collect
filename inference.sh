source ~/ros2_ws/install/setup.bash
gnome-terminal --tab -- bash -c "ros2 launch zed_wrapper zed_rgb_only.launch.py camera_model:=zed2i; exec bash"
gnome-terminal --tab -- bash -c "ros2 run policy_training external_rgb_publisher; exec bash"