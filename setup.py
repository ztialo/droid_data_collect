from setuptools import find_packages, setup

package_name = 'droid_data_collect'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/subscriber.launch.py', 'launch/bag_record.launch.py']),
        ('share/' + package_name + '/config', ['config/topics.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zi Tao Li',
    maintainer_email='zdli@ucsc.edu',
    description='ROS 2 package for topic subscription and rosbag recording workflows.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'zed_image_subscriber = droid_data_collect.zed_bridge:main',
            'button_bag_toggle = droid_data_collect.button_bag_toggle:main',
        ],
    },
)
