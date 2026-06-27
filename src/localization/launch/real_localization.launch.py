"""
real_localization.launch.py
============================
Real-hardware EKF localization pipeline:

  /wheel_encoders  →  wheel_odom_node  →  /odom  ─┐
  /imu             ──────────────────────────────── ┼→  ekf_filter_node  →  /odometry/filtered
  /gps  →  navsat_transform_node  →  /odometry/gps ─┘         ↓
                                                           /tf  odom→base_link
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_loc = get_package_share_directory('localization')
    ekf_yaml    = os.path.join(pkg_loc, 'config', 'ekf_real.yaml')
    navsat_yaml = os.path.join(pkg_loc, 'config', 'navsat_real.yaml')

    # ── 1. Wheel odometry ─────────────────────────────────────────────────────
    wheel_odom = Node(
        package='hardware',
        executable='wheel_odom_node.py',
        name='wheel_odom_node',
        output='screen',
        parameters=[{
            'wheel_radius':     0.075,
            'wheel_separation': 0.44,
            'publish_tf':       False,   # EKF publishes odom→base_link TF
        }]
    )

    # ── 2. EKF (odom + IMU + GPS-odom) ────────────────────────────────────────
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
    )

    # ── 3. GPS → local odom  (needs /gps  +  /odometry/filtered  +  /imu) ────
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_yaml],
        remappings=[
            ('gps/fix',            '/gps'),                 # from nmea_navsat
            ('imu',                '/imu'),                 # from Teensy
            ('odometry/filtered',  '/odometry/filtered'),   # from EKF
            ('odometry/gps',       '/odometry/gps'),        # back to EKF odom1
        ]
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('localization'),
                'launch',
                'slam.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 10.0,
            'node_names': ['slam_toolbox']
        }]
    )

    return LaunchDescription([
        wheel_odom,
        ekf,
        navsat,
        slam,
        lifecycle_manager,
    ])
