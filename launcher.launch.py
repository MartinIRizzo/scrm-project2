from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', os.path.join(os.getenv('HOME'), 'ros2_ws/src/my_robot/worlds/simple.world'), '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Launch movement node
        Node(
            package='my_robot',
            executable='move_robot.py',
            output='screen'
        )
    ])
