from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='go2_cam',
            namespace='go2/cam',
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[{
                    'enable_infra1': True,
                    'enable_infra2': True,
                    'enable_color': True,
                    'enable_depth': False,
                    'depth_module.emitter_enabled': 0,
                    'rgb_camera.profile':'640x480x30',
                    'depth_module.profile': '640x480x30',
                    'enable_gyro': True,
                    'enable_accel': True,
                    'gyro_fps': 400,
                    'accel_fps': 200,
                    'unite_imu_method': 2,
                    # 'tf_publish_rate': 0.0
            }]
        )
    ])
