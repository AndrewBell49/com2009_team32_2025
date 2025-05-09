from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description(): 
    # Get the cartographer launch file
    cartographer_launch_dir = get_package_share_directory('tuos_simulations')
    cartographer_launch_path = os.path.join(cartographer_launch_dir, 'launch', 'cartographer.launch.py')

    # Get the map saver launch file
    map_saver_launch_dir = get_package_share_directory('nav2_map_server')
    map_saver_launch_path = os.path.join(map_saver_launch_dir, 'launch', 'map_saver_server.launch.py')

    online_async_launch_dir = get_package_share_directory('slam_toolbox')
    online_async_launch_path = os.path.join(online_async_launch_dir, 'launch', 'online_async_launch.py')

    return LaunchDescription([ 
        DeclareLaunchArgument(
            name='target_colour', 
            description="The colour of the beacon to search for (yellow|red|green|blue)."
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer_launch_path),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_saver_launch_path)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(online_async_launch_path),
            launch_arguments={'use_sim_time': 'false'}.items()
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
            parameters=[{'target_colour': LaunchConfiguration('target_colour')}] 
        ),

        Node( 
            package='com2009_team32_2025', 
            executable='exploration.py', 
            name='exploration'
        )       

    ])