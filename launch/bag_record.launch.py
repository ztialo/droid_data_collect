import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('droid_data_collect')
    params_file = os.path.join(package_share, 'config', 'topics.yaml')

    bag_record_node = Node(
        package='droid_data_collect',
        executable='bag_record',
        name='bag_record',
        output='screen',
    )

    return LaunchDescription([
        bag_record_node,
    ])
