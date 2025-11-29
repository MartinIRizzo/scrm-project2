#!/usr/bin/env python3
"""
MUSI-nav2 Simulation Launch File

Starts complete simulation environment with visualization in one launch file:
- Gazebo server and client
- Robot state publisher
- TurtleBot3 spawner
- RViz visualization (optional)

Usage:
    ros2 launch simulation.launch.py

Optional arguments:
    world:=<world_name>   (select Gazebo world: landmark, turtlebot3_world,
                           turtlebot3_house, empty_world. Default: landmark)
    headless:=true        (run Gazebo without GUI, saves resources)
    x:=2.0 y:=-2.0        (robot spawn position)
    rviz:=true            (launch RViz, set to false to skip)

Examples:
    ros2 launch simulation.launch.py world:=turtlebot3_world
    ros2 launch simulation.launch.py world:=empty_world headless:=true

Expected: Gazebo opens with robot, RViz shows visualization
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_world_config():
    """
    Load world configuration from YAML file.

    Returns:
        Dictionary containing world configurations

    Raises:
        FileNotFoundError: If config file doesn't exist
        yaml.YAMLError: If config file is malformed
    """
    config_path = '/shared/configs/worlds.yaml'

    if not os.path.exists(config_path):
        raise FileNotFoundError(
            f"World configuration file not found: {config_path}. "
            f"Please ensure the file exists with world definitions."
        )

    with open(config_path, 'r') as f:
        try:
            config = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise yaml.YAMLError(f"Failed to parse world config: {e}")

    return config


def get_world_path(world_name, config=None):
    """
    Resolve world name to full file path using config.

    Args:
        world_name: Name of the world (from worlds.yaml)
        config: Pre-loaded config dict (optional, will load if not provided)

    Returns:
        Full path to the world file

    Raises:
        ValueError: If world_name is not supported
        FileNotFoundError: If world file doesn't exist
    """
    if config is None:
        config = load_world_config()

    worlds = config.get('worlds', {})

    if world_name not in worlds:
        supported = ', '.join(worlds.keys())
        raise ValueError(
            f"Unknown world: '{world_name}'. "
            f"Supported worlds: {supported}"
        )

    world_config = worlds[world_name]
    world_path = world_config['path']

    # Verify file exists
    if not os.path.exists(world_path):
        raise FileNotFoundError(
            f"World file not found: {world_path}. "
            f"Ensure world file exists in /shared/worlds/"
        )

    return world_path


def load_simulation_params():
    """
    Load simulation parameters from YAML file.

    Returns:
        Dictionary containing simulation parameters

    Raises:
        FileNotFoundError: If params file doesn't exist
    """
    params_path = '/shared/configs/simulation_params.yaml'

    if not os.path.exists(params_path):
        # Return defaults if file doesn't exist
        return {
            'spawn': {'x': -1.0, 'y': 1.0, 'z': 0.01},
            'world': {'default_world': 'landmark'},
            'simulation': {'use_sim_time': True, 'headless': False},
            'visualization': {'rviz': True, 'rviz_config': '/shared/configs/default.rviz'}
        }

    with open(params_path, 'r') as f:
        params = yaml.safe_load(f)

    return params


def generate_launch_description():
    # Package directories
    bringup_dir = get_package_share_directory('nav2_bringup')
    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Files
    urdf_file = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Load configurations
    world_config = load_world_config()
    sim_params = load_simulation_params()

    # Get defaults from params
    available_worlds = ', '.join(world_config.get('worlds', {}).keys())
    default_world = sim_params.get('world', {}).get('default_world', 'landmark')
    default_x = str(sim_params.get('spawn', {}).get('x', -1.0))
    default_y = str(sim_params.get('spawn', {}).get('y', 1.0))
    default_z = str(sim_params.get('spawn', {}).get('z', 0.01))
    default_rviz_config = sim_params.get('visualization', {}).get('rviz_config', '/shared/configs/default.rviz')
    default_use_sim_time = str(sim_params.get('simulation', {}).get('use_sim_time', True)).lower()
    default_headless = str(sim_params.get('simulation', {}).get('headless', False)).lower()
    default_rviz = str(sim_params.get('visualization', {}).get('rviz', True)).lower()

    # Launch arguments
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description=f'Gazebo world to load. Available: {available_worlds}'
    )

    declare_x = DeclareLaunchArgument(
        'x', default_value=default_x,
        description='X coordinate for robot spawn'
    )

    declare_y = DeclareLaunchArgument(
        'y', default_value=default_y,
        description='Y coordinate for robot spawn'
    )

    declare_z = DeclareLaunchArgument(
        'z', default_value=default_z,
        description='Z coordinate for robot spawn'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value=default_use_sim_time,
        description='Use simulation clock'
    )

    declare_headless = DeclareLaunchArgument(
        'headless', default_value=default_headless,
        description='Run Gazebo in headless mode (no GUI)'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value=default_rviz,
        description='Launch RViz visualization'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
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

    # 1. Gazebo server (physics simulation) - use OpaqueFunction for dynamic world resolution
    def launch_gazebo_server(context):
        """Resolve world path at runtime and launch Gazebo server."""
        world_name = context.launch_configurations['world']
        world_path = get_world_path(world_name, world_config)

        return [
            ExecuteProcess(
                cmd=['gzserver',
                     '-s', 'libgazebo_ros_init.so',
                     '-s', 'libgazebo_ros_factory.so',
                     world_path],
                output='screen'
            )
        ]

    start_gazebo_server = OpaqueFunction(function=launch_gazebo_server)

    # 2. Gazebo client (GUI) - delayed to let server initialize, skip if headless
    start_gazebo_client = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['gzclient'],
                output='screen'
            )
        ],
        condition=UnlessCondition(headless_mode)
    )

    # 3. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # 4. Spawn TurtleBot3 in Gazebo - delayed to let Gazebo initialize
    spawn_robot = TimerAction(
        period=40.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_turtlebot3',
                output='screen',
                arguments=[
                    '-entity', 'turtlebot3_waffle',
                    '-file', os.path.join(gazebo_dir, 'models', 'turtlebot3_waffle', 'model.sdf'),
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', z_pose,
                ]
            )
        ]
    )

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
        declare_world,
        declare_x,
        declare_y,
        declare_z,
        declare_use_sim_time,
        declare_headless,
        declare_rviz,
        declare_rviz_config,

        # Launch components
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher,
        spawn_robot,
        rviz_launch,
    ])
