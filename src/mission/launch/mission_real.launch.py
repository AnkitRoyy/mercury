"""
mission_real.launch.py
=======================
Real-hardware mission launch. Fully automatic GPS → Cartesian conversion.

Launch sequence (each step waits for the previous to EXIT cleanly):

  1. gps_datum_injector
       ├─ Subscribes to /gps
       ├─ Averages N fixes → datum (= map-frame 0,0)
       ├─ Converts GPS waypoints (from gps_waypoints.yaml) → (x, y)
       └─ Writes patched mission_params.yaml  →  exits

  2. mission_setup
       ├─ Waits for map→odom TF
       ├─ Reads robot's current map-frame pose as HOME
       ├─ Appends HOME to mission_params.yaml  →  exits

  3. watchdog + turret + waypoint_sender  (all start together)
       └─ waypoint_sender sends Nav2 the converted (x,y) goals

Usage:
  ros2 launch mission mission_real.launch.py
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

# ── Shared paths ──────────────────────────────────────────────────

MISSION_SHARE   = get_package_share_directory('mission')
GENERATED_YAML  = os.path.join(MISSION_SHARE, 'config', 'mission_params.yaml')
GPS_WP_YAML     = os.path.join(MISSION_SHARE, 'config', 'gps_waypoints.yaml')


def generate_launch_description():

    # ── 1. GPS datum injector — converts lat/lon → (x,y) ─────────
    gps_datum_injector = Node(
        package='mission',
        executable='gps_datum_injector',
        name='gps_datum_injector',
        output='screen',
        parameters=[{
            'generated_yaml_path': GENERATED_YAML,
            'gps_waypoints_yaml':  GPS_WP_YAML,
        }],
    )

    # ── 2. mission_setup — records HOME in map frame ──────────────
    mission_setup = Node(
        package='mission',
        executable='mission_setup',
        name='mission_setup',
        output='screen',
        parameters=[{
            'generated_yaml_path': GENERATED_YAML,
        }],
    )

    # ── 3. Watchdog monitor ───────────────────────────────────────
    watchdog = IncludeLaunchDescription_helper(
        package='watchdog_monitor',
        launch_file='watchdog.launch.py',
        args={'params_file': GENERATED_YAML},
    )

    # ── 4. Turret vision ──────────────────────────────────────────
    turret = IncludeLaunchDescription_helper(
        package='turret_vision',
        launch_file='turret_vision.launch.py',
    )

    # ── 5. Waypoint sender ────────────────────────────────────────
    waypoint_sender = Node(
        package='mission',
        executable='waypoint_sender',
        name='waypoint_sender',
        output='screen',
        parameters=[{
            'generated_yaml_path': GENERATED_YAML,
        }],
    )

    # ── Chain: injector → setup → (watchdog + turret + sender) ───
    start_mission_setup = RegisterEventHandler(
        OnProcessExit(
            target_action=gps_datum_injector,
            on_exit=[mission_setup],
        )
    )

    start_mission_stack = RegisterEventHandler(
        OnProcessExit(
            target_action=mission_setup,
            on_exit=[
                watchdog,
                turret,
                waypoint_sender,
            ],
        )
    )

    return LaunchDescription([
        gps_datum_injector,
        start_mission_setup,
        start_mission_stack,
    ])


# ── Helper to keep the launch file concise ────────────────────────

def IncludeLaunchDescription_helper(package, launch_file, args=None):
    from launch.actions import IncludeLaunchDescription
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package),
                'launch',
                launch_file,
            ])
        ),
        launch_arguments=(args or {}).items(),
    )
    return include
