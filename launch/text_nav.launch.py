#!/usr/bin/env python3
"""
TextNav Launch File (Phase 2: Navigation)
Runs RTAB-Map localization + Nav2 navigation + text_nav_bridge
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    landmark_file = LaunchConfiguration('landmark_file')

    # Package directories
    text_nav_bridge_dir = get_package_share_directory('text_nav_bridge')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(text_nav_bridge_dir, 'config', 'nav2_params.yaml')

    # RTAB-Map parameters (localization mode)
    rtabmap_parameters = [{
        'frame_id': 'camera_link',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': True,
        'wait_imu_to_init': False,
    }]

    rtabmap_remappings = [
        ('imu', '/imu/data'),
        ('rgb/image', '/camera/infra1/image_rect_raw'),
        ('rgb/camera_info', '/camera/infra1/camera_info'),
        ('depth/image', '/camera/depth/image_rect_raw')
    ]

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time (true for rosbag, false for real camera)'
        ),
        DeclareLaunchArgument(
            'landmark_file',
            description='Path to landmarks.yaml file (required)'
        ),

        # Global parameter
        SetParameter(name='use_sim_time', value=use_sim_time),

        # === IMU Filter ===
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            output='log',
            arguments=['--ros-args', '--log-level', 'error'],
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
            }],
            remappings=[('imu/data_raw', '/camera/imu')]
        ),

        # === RGBD Odometry (odom → camera_link TF) ===
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='log',
            arguments=['--ros-args', '--log-level', 'error'],
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings
        ),

        # === RTAB-Map Localization Mode (map → odom TF + /map) ===
        # No '-d' argument = localization mode (uses existing database)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='log',
            arguments=['--ros-args', '--log-level', 'error'],
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings
        ),

        # === Nav2 Navigation Stack ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
                'autostart': 'true',
                'log_level': 'warn',
            }.items()
        ),

        # === Text Navigation Bridge ===
        Node(
            package='text_nav_bridge',
            executable='text_nav_bridge_node',
            name='text_nav_bridge',
            output='screen',
            parameters=[{
                'landmark_file': landmark_file,
                'match_threshold': 0.5,
                'robot_frame': 'camera_link',
                'world_frame': 'map',
            }]
        ),

        # === RViz ===
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(text_nav_bridge_dir, 'rviz', 'text_nav.rviz'),
                       '--ros-args', '--log-level', 'warn'],
            output='screen',
        ),
    ])
