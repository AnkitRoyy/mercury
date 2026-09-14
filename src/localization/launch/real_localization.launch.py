# localization/launch/real_localization.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_loc = get_package_share_directory('localization')

    # ── EKF (real-hardware config) ──────────────────────────────────
    ekf_real_yaml = os.path.join(pkg_loc, 'config', 'ekf_real.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_real_yaml]
    )

    # ── SLAM (real-time clock) ──────────────────────────────────────
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

    # ── GPS localization pipeline (real hardware) ───────────────────
    navsat_real_yaml = os.path.join(pkg_loc, 'config', 'navsat_real.yaml')

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_real_yaml],
        remappings=[
            ('gps/fix', '/gps'),          # real GPS publishes directly here
            ('imu', '/imu'),
            ('odometry/filtered', '/odometry/filtered'),
        ]
    )

    return LaunchDescription([
        ekf_node,
        slam,
        lifecycle_manager,
        navsat_transform,
    ])