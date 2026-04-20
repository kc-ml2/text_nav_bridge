#!/usr/bin/env python3
"""TextNav launch file (Phase 2) — rtabmap localization + Nav2 + text_nav_bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _default_data_dir():
    # text_nav_bridge is installed under <workspace>/install/text_nav_bridge/share/...
    # but landmark YAML files and rtabmap DBs live in the source tree at
    # <workspace>/src/text_nav_bridge. This resolves that path from the share dir
    # for the standard colcon layout; override via the 'data_dir' launch argument.
    share_dir = get_package_share_directory('text_nav_bridge')
    marker = os.sep + 'install' + os.sep
    workspace_root = share_dir.split(marker)[0] if marker in share_dir else share_dir
    return os.path.join(workspace_root, 'src', 'text_nav_bridge')


def launch_setup(context):
    bag_name = LaunchConfiguration('bag_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    data_dir = LaunchConfiguration('data_dir').perform(context)
    match_threshold = LaunchConfiguration('match_threshold').perform(context)
    robot_frame = LaunchConfiguration('robot_frame').perform(context)
    world_frame = LaunchConfiguration('world_frame').perform(context)

    text_nav_bridge_dir = get_package_share_directory('text_nav_bridge')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    landmark_file = os.path.join(data_dir, 'landmarks', f'{bag_name}.yaml')
    database_path = os.path.join(data_dir, 'rtabmap_db', f'{bag_name}.db')
    nav2_params_file = os.path.join(text_nav_bridge_dir, 'config', 'nav2_params.yaml')

    rtabmap_parameters = [{
        'frame_id': 'camera_link',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': True,
        'wait_imu_to_init': False,
        'Mem/IncrementalMemory': 'false',
        'Mem/InitWMWithAllNodes': 'true',
        'database_path': database_path,
    }]

    rtabmap_remappings = [
        ('imu', '/imu/data'),
        ('rgb/image', '/camera/infra1/image_rect_raw'),
        ('rgb/camera_info', '/camera/infra1/camera_info'),
        ('depth/image', '/camera/depth/image_rect_raw')
    ]

    return [

        SetParameter(name='use_sim_time', value=(use_sim_time.lower() == 'true')),

        # Static TF for RealSense D455
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera_infra1',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'camera_infra1_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_aligned_depth',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'camera_link', 'camera_aligned_depth_to_infra1_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_infra1_optical',
            arguments=[
                '0', '0', '0', '-0.5', '0.5', '-0.5', '0.5',
                'camera_aligned_depth_to_infra1_frame', 'camera_infra1_optical_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_depth',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'camera_depth_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_depth_optical',
            arguments=[
                '0', '0', '0', '-0.5', '0.5', '-0.5', '0.5',
                'camera_depth_frame', 'camera_depth_optical_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_gyro',
            arguments=[
                '-0.01602', '-0.03022', '0.0074', '0', '0', '0',
                'camera_link', 'camera_gyro_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_frame', 'camera_imu_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu_optical',
            arguments=[
                '0', '0', '0', '-0.5', '0.5', '-0.5', '0.5',
                'camera_imu_frame', 'camera_imu_optical_frame']
        ),

        # IMU Filter
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

        # RGBD Odometry (odom -> camera_link TF)
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='log',
            arguments=['--ros-args', '--log-level', 'error'],
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings
        ),

        # RTAB-Map Localization Mode (map -> odom TF + /map).
        # No '-d' argument = localization mode (uses existing database)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='log',
            arguments=['--ros-args', '--log-level', 'error'],
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings
        ),

        # Nav2 Navigation Stack
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

        # Text Navigation Bridge
        Node(
            package='text_nav_bridge',
            executable='text_nav_bridge_node',
            name='text_nav_bridge',
            output='screen',
            parameters=[{
                'landmark_file': landmark_file,
                'match_threshold': float(match_threshold),
                'robot_frame': robot_frame,
                'world_frame': world_frame,
            }]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(text_nav_bridge_dir, 'rviz', 'text_nav.rviz'),
                       '--ros-args', '--log-level', 'warn'],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_name',
            description='Rosbag name (e.g. rosbag2_2026_01_08-15_03_00)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time (true for rosbag, false for real camera)'
        ),
        DeclareLaunchArgument(
            'data_dir',
            default_value=_default_data_dir(),
            description='Directory containing landmarks/ and rtabmap_db/ subdirectories'
        ),
        DeclareLaunchArgument(
            'match_threshold',
            default_value='0.5',
            description='Text similarity threshold for landmark matching (0~1)'
        ),
        DeclareLaunchArgument(
            'robot_frame',
            default_value='camera_link',
            description='Robot base frame id'
        ),
        DeclareLaunchArgument(
            'world_frame',
            default_value='map',
            description='World/map frame id'
        ),
        OpaqueFunction(function=launch_setup),
    ])
