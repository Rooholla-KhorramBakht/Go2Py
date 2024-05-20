import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    robot_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sportmode_nav2'), 'launch'),
            '/ros_ekf.launch.py'])
    )
    async_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sportmode_nav2'), 'launch'),
            '/mapping_async.launch.py'])
    )

    return LaunchDescription([
        robot_localization,
        async_mapping,
    ])
