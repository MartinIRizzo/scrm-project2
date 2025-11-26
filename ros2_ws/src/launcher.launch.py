from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', os.path.join(os.getenv('HOME'), 'scrm-project2/worlds/simple.world'), '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Launch movement node
        Node(
            package='my_robot',
            executable='controller.py',
            output='screen'
        )
    ])
