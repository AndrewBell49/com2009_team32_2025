from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration 

def generate_launch_description(): 
    return LaunchDescription([ 
        DeclareLaunchArgument(
            name='target_colour', 
            description="The colour of the beacon to search for (yellow|red|green|blue)."
        ),
        Node( 
            package='com2009_team32_2025', 
            executable='beacon_search.py', 
            name='beacon_search',
            parameters=[{'target_colour': LaunchConfiguration('target_colour')}] 
        )
    ])