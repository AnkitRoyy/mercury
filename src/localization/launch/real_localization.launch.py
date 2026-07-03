"""
real_localization.launch.py
============================
Real-hardware EKF localization pipeline:
  /cmd_vel          →  cmd_vel_odom_node  →  /odom  ─┐
  /imu (Madgwick)   ─────────────────────────────────┼→  ekf_filter_node  →  /odometry/filtered
  /gps  →  navsat_transform_node  →  /odometry/gps  ─┘         ↓
                                                            /tf  odom→base_link

NOTE: cmd_vel_odom_node is OPEN-LOOP (no encoders). Its /odom carries a
trusted twist (mirrors /cmd_vel) and an untrusted pose (covariance 1e6).
ekf_real.yaml's odom0_config must only fuse vx/vy/vyaw from it, never x/y.
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

    # ── 1. Odometry source (open-loop, cmd_vel-based — no encoders) ───────
    cmd_vel_odom = Node(
        package='hardware',
        executable='cmd_vel_odom_node.py',
        name='cmd_vel_odom_node',
        output='screen',
    )

    # ── 2. EKF (odom twist + IMU + GPS-odom) ───────────────────────────────
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
    )

    # ── 3. GPS → local odom  (needs /gps  +  /odometry/filtered  +  /imu) ──
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_yaml],
        remappings=[
            ('gps/fix',            '/gps'),
            ('imu',                '/imu'),                 # Madgwick-filtered RealSense IMU
            ('odometry/filtered',  '/odometry/filtered'),
            ('odometry/gps',       '/odometry/gps'),
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
        cmd_vel_odom,
        ekf,
        navsat,
        slam,
        lifecycle_manager,
    ])