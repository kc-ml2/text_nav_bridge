"""Simulation navigation pipeline: map_server + AMCL + Nav2 + text_nav_bridge + RViz.

Requires a Gazebo simulation (text_nav_sim/simulation.launch.py) to be running.
Parallel to text_nav.launch.py for real hardware (which uses rtabmap localization).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    text_nav_bridge_dir = get_package_share_directory('text_nav_bridge')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    landmark_file = LaunchConfiguration('landmark_file')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    match_threshold = ParameterValue(
        LaunchConfiguration('match_threshold'), value_type=float)

    nav2_config = os.path.join(text_nav_bridge_dir, 'config', 'nav2_sim_params.yaml')
    rviz_config = os.path.join(text_nav_bridge_dir, 'rviz', 'text_nav_sim.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'landmark_file',
            description='Path to landmarks.yaml produced by the mapping phase'),
        DeclareLaunchArgument(
            'map_yaml_file',
            description='Path to map.yaml produced by nav2_map_server map_saver_cli'),
        DeclareLaunchArgument('match_threshold', default_value='0.5'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml_file,
            }],
            output='screen',
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[nav2_config, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py'),
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_config,
            }.items(),
        ),

        Node(
            package='text_nav_bridge',
            executable='text_nav_bridge_node',
            name='text_nav_bridge_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'landmark_file': landmark_file,
                'robot_frame': 'base_footprint',
                'world_frame': 'map',
                'match_threshold': match_threshold,
            }],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
