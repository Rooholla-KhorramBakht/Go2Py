from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    go2_xacro_file = os.path.join(
        get_package_share_directory("go2_description"), "xacro", "robot.xacro"
    )
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", go2_xacro_file, " DEBUG:=", 'false']
    )

    return LaunchDescription([        
        Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[
                    ("/joint_states", "/go2/joint_states"),
                    ("/robot_description", "/go2/robot_description"),
                ],
            ),
        Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0.15",
                    "0",
                    "0.15",
                    "0",
                    "0",
                    "0.707107",
                    "0.707107",
                    "/trunk",
                    "/go2/hesai_lidar",
                ],
                name="static_tf_pub_trunk_to_lidar",
            ),
    ])