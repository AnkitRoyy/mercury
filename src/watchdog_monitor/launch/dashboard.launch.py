"""
dashboard.launch.py
===================
Standalone dashboard launch (optional).

Usage:
  ros2 launch watchdog_monitor dashboard.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('watchdog_monitor'), 'config', 'watchdog_params.yaml'
        ]),
        description='Path to parameters YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    dashboard_node = Node(
        package='watchdog_monitor',
        executable='dashboard',
        name='dashboard',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        params_arg,
        use_sim_time_arg,
        dashboard_node,
    ])