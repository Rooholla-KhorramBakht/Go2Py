import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    color_format_converter_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_color',
        parameters=[{
                'encoding_desired': 'rgb8',
        }],
        remappings=[
            ('image_raw', '/go2/d455/color/image_raw'),
            ('image', '/go2/d455/color/image_raw_mono')]
    )

    color_encoder_node = ComposableNode(
        package='isaac_ros_h264_encoder',
        plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
        name='color_encoder_node',
        parameters=[{
            'input_width': 640,
            'input_height': 480,
        }],
        remappings=[
            ('image_raw', 'go2/realsense/color/image_raw_mono'),
            ('image_compressed', 'go2/realsense/color/image_raw_compressed')]
    )

    container = ComposableNodeContainer(
        name='encoder_container',
        namespace='encoder',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
                                      color_format_converter_node,
                                    #   color_encoder_node
                                      ],
        output='screen'
    )

    return (launch.LaunchDescription([container]))
