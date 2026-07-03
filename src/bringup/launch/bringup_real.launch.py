from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_desc = get_package_share_directory('description')
    xacro_file = os.path.join(pkg_desc, 'urdf', 'robot_real.urdf.xacro')

    # ── 1. Hardware drivers (GPS, IMU, motors …) ──────────────────
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('hardware'),
                'launch',
                'hardware.launch.py',
            ])
        )
    )

    # ── 2. Base bringup: URDF + localization + Nav2 + perception ──
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bringup'),
                'launch',
                'bringup_base.launch.py',
            ])
        ),
        launch_arguments={
            'xacro_file':          xacro_file,
            'use_sim_time':        'false',
            'localization_launch': 'real_localization.launch.py',
        }.items()
    )

    # ── 3. Mission stack (GPS inject → HOME record → navigate) ────
    #       gps_datum_injector reads /gps, converts waypoints,
    #       then mission_setup adds HOME, then waypoint_sender drives.
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mission'),
                'launch',
                'mission_real.launch.py',
            ])
        )
    )

    return LaunchDescription([
        hardware,
        base,
        mission,
    ])
