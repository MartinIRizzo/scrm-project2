#!/usr/bin/env python3
"""
SLAM with Cartographer Launch File

Simple wrapper to launch Cartographer SLAM with configurable parameter files.

Usage:
    ros2 launch slam_cartographer.launch.py
    ros2 launch slam_cartographer.launch.py configuration_basename:=cartographer_high_detail.lua

Examples:
    # Balanced config (default)
    ros2 launch slam_cartographer.launch.py

    # High detail config
    ros2 launch slam_cartographer.launch.py configuration_basename:=cartographer_high_detail.lua

    # Realtime config
    ros2 launch slam_cartographer.launch.py configuration_basename:=cartographer_realtime.lua

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
    # Package directory
    cartographer_launch_dir = get_package_share_directory('turtlebot3_cartographer')

    # Launch arguments
    declare_config_basename = DeclareLaunchArgument(
        'configuration_basename',
        default_value='cartographer_balanced.lua',
        description='Cartographer .lua config filename (must be in /shared/configs/)'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value='/opt/ros/humble/share/turtlebot3_cartographer/rviz/tb3_cartographer.rviz',
        description='RViz config file path'
    )

    # Get configurations
    configuration_basename = LaunchConfiguration('configuration_basename')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # Include TurtleBot3 Cartographer launch
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cartographer_launch_dir, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'cartographer_config_dir': '/shared/configs',
            'configuration_basename': configuration_basename,
            'use_rviz': 'false',
        }.items()
    )

    # RViz node
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
        declare_config_basename,
        declare_use_sim_time,
        declare_rviz,
        declare_rviz_config,
        cartographer_launch,
        rviz_node,
    ])
