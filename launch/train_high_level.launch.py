#!/usr/bin/env python3
"""Launch stage-1 (high-level PPO) interception training.

The simulation (drone_interception_sim interception.launch.py) must already be
running. This only starts the training process against it.
"""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('d2dtracker_rl'),
                       'config', 'train_ppo_high_level.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=cfg),
        DeclareLaunchArgument('timesteps', default_value='200000'),
        Node(
            package='d2dtracker_rl', executable='train', name='d2d_train',
            output='screen',
            arguments=['--stage', 'high',
                       '--config', LaunchConfiguration('config'),
                       '--timesteps', LaunchConfiguration('timesteps')],
        ),
    ])
