#!/usr/bin/env python3
"""
Waypoint Navigation Launch File

Launches the waypoint follower node that commands the robot to follow
a predefined sequence of waypoints using Nav2's FollowWaypoints action.

This launch file assumes:
- Simulation is already running (simulation.launch.py)
- Nav2 stack is active (nav2.launch.py)
- Robot pose has been initialized

Usage:
    ros2 launch /ros2_ws/src/waypoint_navigation.launch.py

Optional arguments:
    waypoint_script:=/path/to/script.py    (default: waypoint_follower.py in same directory)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the directory containing this launch file
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    default_script = os.path.join(launch_dir, 'waypoint_follower.py')

    # Launch arguments
    declare_script = DeclareLaunchArgument(
        'waypoint_script',
        default_value=default_script,
        description='Full path to waypoint follower script'
    )

    # Get configuration
    waypoint_script = LaunchConfiguration('waypoint_script')

    # Waypoint follower node
    waypoint_node = Node(
        package='nav2_bringup',  # Using nav2_bringup as a dummy package
        executable='python3',
        name='waypoint_follower',
        output='screen',
        arguments=[waypoint_script],
        parameters=[{'use_sim_time': True}],
        emulate_tty=True,
    )

    # Startup message
    startup_msg = LogInfo(
        msg=[
            '\n',
            '=' * 60, '\n',
            'Waypoint Navigation Launcher\n',
            '=' * 60, '\n',
            'Starting waypoint follower node...\n',
            'The robot will navigate through predefined waypoints.\n',
            '\n',
            'Prerequisites:\n',
            '  1. Simulation running (simulation.launch.py)\n',
            '  2. Nav2 active (nav2.launch.py)\n',
            '  3. Robot pose initialized (/initialpose published)\n',
            '\n',
            'Press Ctrl+C to cancel waypoint following\n',
            '=' * 60, '\n'
        ]
    )

    return LaunchDescription([
        declare_script,
        startup_msg,
        waypoint_node,
    ])
