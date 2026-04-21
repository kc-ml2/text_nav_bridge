#!/usr/bin/env python3
"""TextNav launch file (Phase 2) — rtabmap localization + Nav2 + text_nav_bridge."""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _load_static_tf_nodes(yaml_path, use_sim_time):
    """Spawn static_transform_publisher nodes from a YAML descriptor.

    Empty / missing path yields no nodes so users can disable injection
    with ``static_tfs_file:=''`` or point the argument at their own file.
    rotation length 3 -> rpy (radians); length 4 -> quaternion (x, y, z, w).
    """
    if not yaml_path or not os.path.isfile(yaml_path):
        return []
    with open(yaml_path, 'r') as f:
        doc = yaml.safe_load(f) or {}
    nodes = []
    for entry in doc.get('static_transforms', []):
        translation = [str(v) for v in entry['translation']]
        rotation = [str(v) for v in entry['rotation']]
        args = translation + rotation + [entry['parent'], entry['child']]
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f"static_tf_{entry['child']}",
            arguments=args,
            parameters=[{'use_sim_time': use_sim_time}],
        ))
    return nodes


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
    static_tfs_file = LaunchConfiguration('static_tfs_file').perform(context)

    text_nav_bridge_dir = get_package_share_directory('text_nav_bridge')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    landmark_file = os.path.join(data_dir, 'landmarks', f'{bag_name}.yaml')
    database_path = os.path.join(data_dir, 'rtabmap_db', f'{bag_name}.db')
    nav2_params_file = os.path.join(text_nav_bridge_dir, 'config', 'nav2_params.yaml')

    static_tf_nodes = _load_static_tf_nodes(
        static_tfs_file, use_sim_time.lower() == 'true')

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

        # Static TFs loaded from YAML if 'static_tfs_file' was provided.
        *static_tf_nodes,

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
        DeclareLaunchArgument(
            'static_tfs_file',
            default_value=os.path.join(
                get_package_share_directory('text_nav_bridge'),
                'config', 'realsense_d455_tfs.yaml'),
            description='YAML file with static_transforms (empty string disables injection)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
