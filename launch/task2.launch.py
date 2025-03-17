from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='com2009_team32_2025',
            executable='explore_server.py',
            name='explore_server'
        ),
        Node(
            package='com2009_team32_2025',
            executable='explore_client.py',
            name='explore_client'
        )
    ])
