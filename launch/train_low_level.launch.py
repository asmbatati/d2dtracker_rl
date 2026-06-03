#!/usr/bin/env python3
"""Launch stage-2 (low-level SAC) interception training.

Requires the simulation to be running. Optionally bootstraps from a stage-1
checkpoint via the ``init_from`` argument.
"""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('d2dtracker_rl'),
                       'config', 'train_sac_low_level.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=cfg),
        DeclareLaunchArgument('timesteps', default_value='300000'),
        DeclareLaunchArgument('init_from', default_value=''),
        Node(
            package='d2dtracker_rl', executable='train', name='d2d_train',
            output='screen',
            arguments=['--stage', 'low',
                       '--config', LaunchConfiguration('config'),
                       '--timesteps', LaunchConfiguration('timesteps'),
                       '--init-from', LaunchConfiguration('init_from')],
        ),
    ])
