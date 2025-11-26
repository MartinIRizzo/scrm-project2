#!/usr/bin/env python3
"""
Exercise 3 - Step 3: Launch Nav2 Navigation Stack

Starts the full Nav2 stack with map server, AMCL localization, planners, and controllers.
Run this AFTER robot is spawned (ex3_2_spawn_robot.launch.py).

Usage:
    ros2 launch ex3_3_nav2.launch.py

Optional arguments:
    map:=/path/to/map.yaml    (default: landmark_map.yaml)
    params:=/path/to/params   (default: nav2_params_ex3.yaml)

Expected: Nav2 servers start, AMCL initializes, ready for navigation goals
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Package directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Default paths
    default_params = '/shared/configs/nav2_params.yaml'

    # Load world config to get map mappings
    world_config_path = '/shared/configs/worlds.yaml'
    if os.path.exists(world_config_path):
        with open(world_config_path, 'r') as f:
            world_config = yaml.safe_load(f)
        default_world = world_config.get('default_world', 'landmark')
    else:
        default_world = 'landmark'

    # Map name to path mapping (from worlds.yaml)
    map_files = {}
    if os.path.exists(world_config_path):
        worlds = world_config.get('worlds', {})
        for world_name, world_data in worlds.items():
            if 'map' in world_data:
                map_files[world_name] = world_data['map']

    # Default map path
    default_map_path = map_files.get(default_world, '/shared/maps/landmark_map.yaml')

    # Load simulation params to get spawn position for initial pose
    sim_params_path = '/shared/configs/simulation_params.yaml'
    if os.path.exists(sim_params_path):
        with open(sim_params_path, 'r') as f:
            sim_params = yaml.safe_load(f)
        default_x = str(sim_params.get('spawn', {}).get('x', -1.0))
        default_y = str(sim_params.get('spawn', {}).get('y', 1.0))
    else:
        default_x = '-1.0'
        default_y = '1.0'

    # Launch arguments
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_world,
        description=f'Map name to load. Available: {", ".join(map_files.keys())}'
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to Nav2 parameter file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    # Get configurations
    map_name = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Resolve map name to path using OpaqueFunction
    def launch_nav2_with_map(context):
        """Resolve map name to path at runtime and launch Nav2."""
        map_name_str = context.launch_configurations['map']

        # Resolve map name to path
        if map_name_str in map_files:
            map_path = map_files[map_name_str]
        else:
            # Assume it's already a full path
            map_path = map_name_str

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_path,
                    'use_sim_time': context.launch_configurations['use_sim_time'],
                    'params_file': context.launch_configurations['params_file'],
                    'autostart': context.launch_configurations['autostart']
                }.items()
            )
        ]

    nav2_bringup = OpaqueFunction(function=launch_nav2_with_map)

    # Publish initial pose after Nav2 starts (gives AMCL the robot's starting position)
    publish_initial_pose = TimerAction(
        period=5.0,  # Wait 5 seconds for Nav2 to fully initialize
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{header: {frame_id: "map"}, pose: {pose: {position: {x: ' +
                    default_x + ', y: ' + default_y + ', z: 0.0}, orientation: {w: 1.0}}, ' +
                    'covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, ' +
                    '0.0, 0.25, 0.0, 0.0, 0.0, 0.0, ' +
                    '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ' +
                    '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ' +
                    '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ' +
                    '0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]}}',
                    '--times', '3'
                ],
                output='screen',
                shell=False
            )
        ]
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_use_sim_time,
        declare_autostart,
        nav2_bringup,
        publish_initial_pose,
    ])
