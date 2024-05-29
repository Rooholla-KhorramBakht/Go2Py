import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # RealSense
    # realsense_config_file_path = os.path.join(
    #     get_package_share_directory('isaac_ros_h264_encoder'),
    #     'config', 'realsense.yaml'
    # )
    realsense_config_file_path = 'realsense.yaml'
    realsense_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[realsense_config_file_path],
        remappings=[
            ('infra1/image_rect_raw', 'go2/realsense/left/image_rect_raw_mono'),
            ('infra2/image_rect_raw', 'go2/realsense/right/image_rect_raw_mono'),
            ('color/image_raw',       'go2/realsense/color/image_raw_mono'),
            ('infra1/camera_info',    'go2/realsense/left/camera_info'),
            ('infra2/camera_info',    'go2/realsense/right/camera_info'),
            ('color/camera_info',     'go2/realsense/color/camera_info'),
            ('depth/camera_info',     'go2/realsense/depth/camera_info'),
            ('depth/color/points',     'go2/realsense/depth/color/points'),
            ('depth/image_rect_raw',     'go2/realsense/depth/image_rect_raw'),
            ('extrinsics/depth_to_accel',     'go2/realsense/extrinsics/depth_to_accel'),
            ('extrinsics/depth_to_color',     'go2/realsense/extrinsics/depth_to_color'),
            ('extrinsics/depth_to_gyro',     'go2/realsense/extrinsics/depth_to_gyro'),
            ('imu',     'go2/realsense/imu'),

        ]
    )
    color_format_converter_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_color',
        parameters=[{
                'encoding_desired': 'rgb8',
        }],
        remappings=[
            ('image_raw', 'go2/realsense/color/image_raw_mono'),
            ('image', 'go2/realsense/color/image_raw')]
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
        composable_node_descriptions=[realsense_node,
                                      color_format_converter_node,
                                      color_encoder_node],
        output='screen'
    )

    return (launch.LaunchDescription([container]))
