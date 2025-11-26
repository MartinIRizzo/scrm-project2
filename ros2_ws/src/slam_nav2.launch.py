#!/usr/bin/env python3
"""
SLAM-Based Navigation Launch File

Launches Nav2 stack with SLAM Toolbox for simultaneous mapping and localization.
NO pre-built map required - builds map online while navigating.

Usage:
    ros2 launch slam_nav2.launch.py

Key Differences from nav2.launch.py:
- Uses SLAM Toolbox (online_async) instead of map_server + AMCL
- No initial pose required - robot localizes immediately
- Builds map in real-time during navigation
- Map can be saved with: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: {data: '/shared/maps/my_map'}"

Author: MUSI-nav2 Project
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Parameter files
    default_nav2_params = '/shared/configs/nav2_params_slam.yaml'
    default_slam_params = '/shared/configs/slam_toolbox_params.yaml'

    # Launch arguments
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=default_nav2_params,
        description='Full path to Nav2 parameter file (without AMCL params)'
    )

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=default_slam_params,
        description='Full path to SLAM Toolbox parameter file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Get configurations
    nav2_params = LaunchConfiguration('nav2_params_file')
    slam_params = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. SLAM Toolbox (replaces map_server + AMCL)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': use_sim_time
        }.items()
    )

    # 2. Nav2 Navigation Stack (without localization/map_server)
    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items()
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value='/opt/ros/humble/share/turtlebot3_cartographer/rviz/tb3_cartographer.rviz',
    )

    use_rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        # Launch arguments
        declare_nav2_params,
        declare_slam_params,
        declare_use_sim_time,
        declare_rviz,
        declare_rviz_config,

        # SLAM Toolbox (provides localization via map->odom transform)
        slam_toolbox,

        # Nav2 navigation stack (planning, control, behaviors)
        nav2_stack,
        
        rviz_node

    ])
