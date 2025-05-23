from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name='target_colour',
            description="The colour of the beacon to search for (yellow|red|green|blue)."
        ),

        # Suppress map saver output
        ExecuteProcess(
            cmd=['ros2', 'launch', 'nav2_map_server', 'map_saver_server.launch.py'],
            output='log'
        ),

        # Suppress SLAM output
        ExecuteProcess(
            cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', 'use_sim_time:=true'],
            output='log'
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='com2009_team32_2025',
                    executable='map_saver_client.py',
                    name='map_saver_client',
                    output='screen'
                )
            ]
        ),

        Node(
            package='com2009_team32_2025',
            executable='beacon_search.py',
            name='beacon_search',
            parameters=[{'target_colour': LaunchConfiguration('target_colour')}],
            output='screen'
        ),

        Node(
            package='com2009_team32_2025', 
            executable='exploration2.py', 
            name='exploration',
            output='screen'
        )   
    ])
