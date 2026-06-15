"""
watchdog.launch.py
==================
Single launch file for all monitoring nodes.

Launches:
  health     - System health monitoring + watchdog alerts (merged)
  waypoints  - Waypoint detection and events
  dashboard  - Terminal UI (optional, enabled with dashboard:=true)

Usage:
  # Full monitoring stack
  ros2 launch watchdog_monitor watchdog.launch.py

  # With dashboard
  ros2 launch watchdog_monitor watchdog.launch.py dashboard:=true

  # With custom params
  ros2 launch watchdog_monitor watchdog.launch.py params_file:=/path/to/params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description() -> LaunchDescription:

    # ── Arguments ──────────────────────────────────────────────────────────
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('watchdog_monitor'), 'config', 'watchdog_params.yaml'
        ]),
        description='Path to parameters YAML file'
    )

    dashboard_arg = DeclareLaunchArgument(
        'dashboard',
        default_value='false',
        description='Launch terminal dashboard'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # ── Nodes ──────────────────────────────────────────────────────────────
    # Health monitoring (merged system_monitor + watchdog)
    health_node = Node(
        package='watchdog_monitor',
        executable='health',
        name='health',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )

    # Waypoint detection
    waypoints_node = Node(
        package='watchdog_monitor',
        executable='waypoints',
        name='waypoints',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )

    # Dashboard (optional - uses IfCondition)
    dashboard_node = Node(
        package='watchdog_monitor',
        executable='dashboard',
        name='dashboard',
        output='screen',
        condition=IfCondition(LaunchConfiguration('dashboard')),
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        params_arg,
        dashboard_arg,
        use_sim_time_arg,
        health_node,
        waypoints_node,
        dashboard_node,
    ])