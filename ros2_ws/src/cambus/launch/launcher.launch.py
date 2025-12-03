from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    robot_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'urdf',
        'turtlebot3_waffle.urdf'
    )
    world_path = "/shared/worlds/uib.world"

    x = -28
    y = 6.7
    z = 0.01

    with open(robot_path, 'r') as f:
        robot_description = f.read()

    start_gazebo = ExecuteProcess(
        cmd=['gazebo', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    spawn_robot = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_turtlebot3',
                output='screen',
                arguments=[
                    '-entity', 'turtlerobot3_waffle',
                    '-file', os.path.join(gazebo_dir, 'models', 'turtlebot3_waffle', 'model.sdf'),
                    '-x', str(x),
                    '-y', str(y),
                    '-z', str(z),
                ]
            )
        ]
    )

    start_bus = Node(
        package='cambus',
        executable='controller',
        output='screen'
    )

    return LaunchDescription([
        start_gazebo,
        robot_state_publisher,
        spawn_robot,
        start_bus
    ])
