#!/usr/bin/env python3
"""Launch evaluation of a trained interception policy against the running sim."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('stage', default_value='high'),
        DeclareLaunchArgument('model', default_value='checkpoints/high/final_model.zip'),
        DeclareLaunchArgument('episodes', default_value='10'),
        Node(
            package='d2dtracker_rl', executable='evaluate', name='d2d_evaluate',
            output='screen',
            arguments=['--stage', LaunchConfiguration('stage'),
                       '--model', LaunchConfiguration('model'),
                       '--episodes', LaunchConfiguration('episodes')],
        ),
    ])
