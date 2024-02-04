from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # launch the pointcloud to laser scan converter
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/go2/lidar_points'),
                        ('scan', '/go2/lidar_scans')],
            parameters=[{
                'target_frame': 'go2/go2/hesai_lidar',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ),

        Node(
            package='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            name='b1_hesai_ros_driver_node'
        ),

        Node(
            name='go2_d455_cam',
            namespace='go2/d435i_cam',
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[{
                    'enable_infra1': True,
                    'enable_infra2': True,
                    'enable_color': False,
                    'enable_depth': False,
                    'depth_module.emitter_enabled': 0,
                    'depth_module.profile': '640x480x60',
                    'enable_gyro': True,
                    'enable_accel': True,
                    'gyro_fps': 400,
                    'accel_fps': 200,
                    'unite_imu_method': 2,
                    # 'tf_publish_rate': 0.0
            }]
        ),

        # Launch the front looking D455 camera
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/d455.launch.py'])
        # ),
        # Run the B1py node
        # Node(
        #     package='b1py_node',
        #     executable='highlevel',
        #     name='b1_highlevel_node'
        # ),

        # Run the B1py calibration TF broadcaster
        # Node(
        #     package='b1py_calib',
        #     executable='calib_broadcaster',
        #     name='b1_calib_broadcaster_node'
        # ),
        # Launch the LiDAR sensor
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rslidar.launch.py'])
        # ),
        # Launch the LiDAR sensor
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(['/B1Py/deploy/docker/launch/state_estimation/ekf.launch.py'])
        # ),
    ])