from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='d455',
            namespace='go2',
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[{
                    'enable_infra1': False,
                    'enable_infra2': False,
                    'enable_color': True,
                    'enable_depth': True,
                    'depth_module.emitter_enabled': 1,
                    'rgb_camera.color_profile':'640x480x30',
                    'depth_module.depth_profile': '640x480x30',
                    'enable_gyro': False,
                    'enable_accel': False,
                    'gyro_fps': 400,
                    'accel_fps': 200,
                    'unite_imu_method': 2,
                    'pointcloud.enable': True
                    # 'tf_publish_rate': 0.0
            }]
        )
    ])
