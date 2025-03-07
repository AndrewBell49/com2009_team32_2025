from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='com2009_team32_2025',
            executable='task2.py',
            name='task2'
        )
    ])