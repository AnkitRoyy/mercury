# localization/launch/real_localization.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_loc = get_package_share_directory('localization')

    # 'false' (default): slam_toolbox owns map->odom (rock-solid indoors from LiDAR).
    # 'true': ekf_global owns map->odom (outdoor GPS waypoint missions in open fields).
    use_gps_global_tf_arg = DeclareLaunchArgument(
        'use_gps_global_tf',
        default_value='false',
        description='If true, EKF Global owns map->odom from GPS. If false, SLAM owns map->odom from LiDAR.'
    )
    use_gps_global_tf = LaunchConfiguration('use_gps_global_tf')

    # ── 1. Local EKF (High-rate, butter-smooth: odom -> base_link) ─────────────
    # Fuses wheel velocity + IMU gyro + anchored yaw. NEVER fuses GPS.
    ekf_local_yaml = os.path.join(pkg_loc, 'config', 'ekf_local.yaml')
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_local_yaml],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ]
    )

    # ── 2. Global EKF (Active when use_gps_global_tf is true) ──────────────────
    ekf_global_yaml = os.path.join(pkg_loc, 'config', 'ekf_global.yaml')
    ekf_global_node = Node(
        condition=IfCondition(use_gps_global_tf),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_global_yaml],
        remappings=[
            ('odometry/filtered', '/odometry/global'),
        ]
    )

    # ── 3. GPS Transform Pipeline (Lat/Lon -> Cartesian /odometry/gps) ────────
    navsat_real_yaml = os.path.join(pkg_loc, 'config', 'navsat_real.yaml')
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_real_yaml],
        remappings=[
            ('gps/fix', '/gps'),
            ('imu', '/imu'),
            ('odometry/filtered', '/odometry/filtered'),
        ]
    )

    # ── 4. SLAM (Real-time LiDAR mapping & localization) ───────────────────────
    # If use_gps_global_tf is false, SLAM publishes map->odom from laser scan matching.
    slam_indoor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('localization'),
                'launch',
                'slam.launch.py'
            ])
        ),
        condition=UnlessCondition(use_gps_global_tf),
        launch_arguments={
            'use_sim_time': 'false',
            'publish_map_odom_tf': 'true'
        }.items()
    )

    # If use_gps_global_tf is true, SLAM still maps obstacles but leaves map->odom to EKF.
    slam_gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('localization'),
                'launch',
                'slam.launch.py'
            ])
        ),
        condition=IfCondition(use_gps_global_tf),
        launch_arguments={
            'use_sim_time': 'false',
            'publish_map_odom_tf': 'false'
        }.items()
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
        use_gps_global_tf_arg,
        ekf_local_node,
        ekf_global_node,
        navsat_transform,
        slam_indoor,
        slam_gps,
        lifecycle_manager,
    ])
