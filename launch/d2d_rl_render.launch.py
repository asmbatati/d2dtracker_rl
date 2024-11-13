from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    interceptor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('d2dtracker_sim'),
                'interceptor.launch.py'
            ])
        ])
    )

    rl_render_node = Node(
        package='d2dtracker_rl',
        executable='d2d_rl_render_node',
        name='d2d_rl_render_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        interceptor_launch,
        rl_render_node
    ])
