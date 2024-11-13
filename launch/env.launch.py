#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from math import radians
def generate_launch_description():
    ld = LaunchDescription()

    ns='interceptor'

    # Node for Interceptor Drone
    world = {'gz_world': 'default'}
    model_name = {'gz_model_name': 'x500_d435'}
    autostart_id = {'px4_autostart_id': '4020'}
    instance_id = {'instance_id': '1'}
    headless= {'headless' : '0'}

    xpos = {'xpos': '0.0'}
    ypos = {'ypos': '0.0'}
    zpos = {'zpos': '0.1'}


    # PX4 SITL + Spawn x500_d435
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_sim'),
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_ns': ns,
            'headless': headless['headless'],
            'gz_world': world['gz_world'],
            'gz_model_name': model_name['gz_model_name'],
            'px4_autostart_id': autostart_id['px4_autostart_id'],
            'instance_id': instance_id['instance_id'],
            'xpos': xpos['xpos'],
            'ypos': ypos['ypos'],
            'zpos': zpos['zpos']
        }.items()
    )

    # MAVROS
    file_name = 'interceptor_px4_pluginlists.yaml'
    package_share_directory = get_package_share_directory('d2dtracker_sim')
    plugins_file_path = os.path.join(package_share_directory, file_name)
    file_name = 'interceptor_px4_config.yaml'
    config_file_path = os.path.join(package_share_directory, file_name)
    
    
    mavros_launch = IncludeLaunchDescription(
                            
                    PythonLaunchDescriptionSource([
            
                PathJoinSubstitution([
                FindPackageShare('d2dtracker_sim'),
                'mavros.launch.py'
                                    ])
                                                ]),
                launch_arguments={
                    'mavros_namespace' :ns+'/mavros',
                    'tgt_system': '2',
                    'fcu_url': 'udp://:14541@127.0.0.1:14558',
                    'pluginlists_yaml': plugins_file_path,
                    'config_yaml': config_file_path,
                    'base_link_frame': 'interceptor/base_link',
                    'odom_frame': 'interceptor/odom',
                    'map_frame': 'map'
                }.items()
                                        )

    odom_frame = 'odom'
    base_link_frame=  'base_link'

    # Static TF map/world -> local_pose_ENU
    map_frame='map'
    map2pose_tf_node = Node(
        package='tf2_ros',
        name='map2px4_'+ns+'_tf_node',
        executable='static_transform_publisher',
        arguments=[str(xpos['xpos']), str(ypos['ypos']), str(zpos['zpos']), '0.0', '0', '0', map_frame, ns+'/'+odom_frame],
    )

    interceptor_offboard_control_node = Node(
        package='d2dtracker_sim',
        executable='interceptor_offboard_control',
        output='screen',
        name='interceptor_offboard_node',
        namespace=ns,
        remappings=[
            ('mavros/state', 'mavros/state'),
            ('mavros/local_position/odom', 'mavros/local_position/odom'),
            ('mavros/setpoint_raw/local', 'mavros/setpoint_raw/local'),
            ('mpc_tracker/command/trajectory', '/mpc_tracker/command/trajectory')
        ]
    )

    # geometric controller node
    file_name = 'geometric_controller.yaml'
    package_share_directory = get_package_share_directory('d2dtracker_sim')
    geometric_controller_file_path = os.path.join(package_share_directory, file_name)
    geometric_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mav_controllers_ros'),
                'launch/geometric_controller.launch.py'
            ])
        ]),
        launch_arguments={
            'yaml_path': geometric_controller_file_path,
            'controller_ns': ns
            }.items()
    )

    # Geometric controller mavros interface node
    file_name = 'geometric_mavros.yaml'
    package_share_directory = get_package_share_directory('d2dtracker_sim')
    geometric_mavros_file_path = os.path.join(package_share_directory, file_name)
    geometric_to_mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mav_controllers_ros'),
                'launch/geometric_to_mavros.launch.py'
            ])
        ]),
        launch_arguments={
            'param_file': geometric_mavros_file_path,
            'mavros_ns': ns
            }.items()
    )

    ld.add_action(gz_launch)
    ld.add_action(map2pose_tf_node)
    ld.add_action(mavros_launch)
    ld.add_action(geometric_controller_launch)
    ld.add_action(geometric_to_mavros_launch)
    ld.add_action(interceptor_offboard_control_node)

    return ld
