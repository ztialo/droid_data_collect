from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('droid_data_collect')
    params_file = os.path.join(package_share, 'config', 'topics.yaml')

    subscriber = Node(
        package='droid_data_collect',
        executable='zed_image_subscriber',
        name='zed_image_subscriber',
        output='screen',
        parameters=[ParameterFile(params_file, allow_substs=True)],
    )

    return LaunchDescription([subscriber])
