#!/usr/bin/env python3
"""
MUSI-nav2 Simulation Launch File

Starts complete simulation environment with visualization in one launch file:
- Gazebo server and client (landmark world)
- Robot state publisher
- TurtleBot3 spawner
- RViz visualization (optional)

Usage:
    ros2 launch simulation.launch.py

Optional arguments:
    headless:=true        (run Gazebo without GUI, saves resources)
    x:=2.0 y:=-2.0        (robot spawn position)
    rviz:=true            (launch RViz, set to false to skip)

Expected: Gazebo opens with robot, RViz shows visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    bringup_dir = get_package_share_directory('nav2_bringup')
    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Files
    world = '/shared/worlds/landmark_world.world'
    urdf_file = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    rviz_config = '/shared/configs/default.rviz'

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch arguments
    declare_x = DeclareLaunchArgument(
        'x', default_value='-2.0',
        description='X coordinate for robot spawn'
    )

    declare_y = DeclareLaunchArgument(
        'y', default_value='2.0',
        description='Y coordinate for robot spawn'
    )

    declare_z = DeclareLaunchArgument(
        'z', default_value='0.01',
        description='Z coordinate for robot spawn'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )

    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo in headless mode (no GUI)'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz visualization'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config,
        description='Full path to RViz config file'
    )

    # Get configurations
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless_mode = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('rviz')
    rviz_config_file = LaunchConfiguration('rviz_config')

    # 5. RViz (conditional, delayed to let robot spawn)
    rviz_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'rviz_launch.py')
                ),
                launch_arguments={
                    'rviz_config': rviz_config_file
                }.items(),
                condition=IfCondition(use_rviz)
            )
        ]
    )

    return LaunchDescription([
        # Declare arguments
        declare_x,
        declare_y,
        declare_z,
        declare_use_sim_time,
        declare_headless,
        declare_rviz,
        declare_rviz_config,

        # Launch components
        rviz_launch,
    ])
