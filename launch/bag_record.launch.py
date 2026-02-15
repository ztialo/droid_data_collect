from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='bag_output',
        description='Directory name for rosbag output',
    )

    record_cmd = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'record',
            '-o',
            LaunchConfiguration('output_dir'),
            '/droid/wrist_image_left',
            '/external_rgb',
        ],
        output='screen',
    )

    return LaunchDescription([
        output_dir_arg,
        record_cmd,
    ])
