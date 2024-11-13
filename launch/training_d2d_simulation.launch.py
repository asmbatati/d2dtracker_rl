#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Define paths to relevant packages
    pkg_d2dtracker_rl = get_package_share_directory('d2dtracker_rl')
    pkg_d2dtracker_sim = get_package_share_directory('d2dtracker_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Path to the world file, if needed
    world_file_name = 'interceptor_world.world'
    world = os.path.join(pkg_d2dtracker_sim, 'worlds', world_file_name)

    # Include the Gazebo server and client for simulation
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
    )

    # Training node using interceptor settings
    training_node = Node(
        package='d2dtracker_rl',
        executable='train_d2d_node',
        name='train_d2d_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        namespace='interceptor'
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        training_node
    ])
