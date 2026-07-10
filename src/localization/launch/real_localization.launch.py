"""
real_localization.launch.py
============================
Real-hardware localization pipeline (GPS-anchored):

  /wheel_encoders  →  wheel_odom_node  →  /odom  ─┐
  /imu             ─────────────────────────────── ┼→  ekf_filter_node  →  /odometry/filtered
  /gps  →  navsat_transform_node  →  /odometry/gps ┘         ↓
                                                         /tf  map→odom
  /scan  →  slam_toolbox (map building only, no TF)  →  /map

TF tree:   map → odom → base_link
           ^^^^^^^^^^^
           EKF owns this (world_frame=map, fusing GPS via odom1)
           slam_toolbox has publish_map_odom_tf: false to avoid conflict.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_loc     = get_package_share_directory('localization')
    ekf_yaml    = os.path.join(pkg_loc, 'config', 'ekf_real.yaml')
    navsat_yaml = os.path.join(pkg_loc, 'config', 'navsat_real.yaml')

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 with bringup.rviz for visual debug'
    )

    # ── 1. Wheel odometry ─────────────────────────────────────────────────────
    wheel_odom = Node(
        package='hardware',
        executable='wheel_odom_node.py',
        name='wheel_odom_node',
        output='screen',
        parameters=[{
            'wheel_radius':     0.075,
            'wheel_separation': 0.44,
            'publish_tf':       True,   # EKF publishes map→odom TF
        }]
    )

    # ── 2. EKF (odom + IMU + GPS) ─────────────────────────────────────────────
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
    )

    # ── 3. GPS → local odom  (needs /gps + /odometry/filtered + /imu) ─────────
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_yaml],
        remappings=[
            ('gps/fix',            '/gps'),                 # from nmea_navsat_driver
            ('imu',                '/imu'),                 # from Teensy
            ('odometry/filtered',  '/odometry/filtered'),   # from EKF
            ('odometry/gps',       '/odometry/gps'),        # back to EKF odom1
        ]
    )

    # ── 4. SLAM (map building only — TF ownership is with EKF) ────────────────
    #   slam_toolbox still runs for visual map building and local costmap
    #   obstacle awareness, but does NOT publish map→odom TF (EKF owns that).
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('localization'),
                'launch',
                'slam.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'publish_map_odom_tf': 'false',
        }.items()
    )

    # ── 5. Lifecycle manager for slam_toolbox ─────────────────────────────────
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

    # ── 6. Optional RViz2 for debug ───────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('bringup'),
            'config',
            'bringup.rviz'
        ])],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        wheel_odom,
        ekf,
        navsat,
        slam,
        lifecycle_manager,
        rviz_node,
    ])
