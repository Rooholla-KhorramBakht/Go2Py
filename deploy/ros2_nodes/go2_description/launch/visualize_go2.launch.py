import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    user_debug_parameter_name = "user_debug"
    user_debug = LaunchConfiguration(user_debug_parameter_name)
    # prefix = LaunchConfiguration("prefix", default="go2/")

    go2_xacro_file = os.path.join(
        get_package_share_directory("go2_description"), "xacro", "robot.xacro"
    )
    robot_description = Command(
        [FindExecutable(name="xacro"), " ", go2_xacro_file, " DEBUG:=", user_debug]
    )

    rviz_file = os.path.join(
        get_package_share_directory("go2_description"), "launch", "visualize_go2.rviz"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                user_debug_parameter_name,
                default_value="false",
                description="debug or not",
            ),
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
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz2",
            #     arguments=["--display-config", rviz_file],
            # ),
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     arguments=[
            #         "0.15",
            #         "0",
            #         "0.15",
            #         "0",
            #         "0",
            #         "0.707107",
            #         "0.707107",
            #         "/trunk",
            #         "/go2/hesai_lidar",
            #     ],
            #     name="static_tf_pub_trunk_to_lidar",
            # ),
        ]
    )
