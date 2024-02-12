from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # launch the pointcloud to laser scan converter
        Node(
            package='go2py_node',
            executable='bridge',
            name='go2py_bridge'
        ),
    ])