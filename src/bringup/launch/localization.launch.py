from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ================= ROBOT DESCRIPTION =================
    pkg_description = get_package_share_directory('description')
    description_launch = os.path.join(
        pkg_description,
        'launch',
        'description.launch.py'
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch)
    )

    # ================= LIDAR =================
    pkg_rplidar = get_package_share_directory('rplidar_ros')
    rplidar_launch = os.path.join(
        pkg_rplidar,
        'launch',
        'rplidar_a3_launch.py'
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch),
        launch_arguments={
            'serial_port': '/dev/ttyUSB1',
            'frame_id': 'laser_link'
        }.items()
    )

    # ================= REALSENSE IMU =================
    pkg_realsense = get_package_share_directory('realsense2_camera')
    realsense_launch = os.path.join(
        pkg_realsense,
        'launch',
        'rs_launch.py'
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            'enable_color': 'false',
            'enable_depth': 'false',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2'
        }.items()
    )

    # ================= EKF =================
    pkg_localization = get_package_share_directory('localization')
    ekf_launch = os.path.join(
        pkg_localization,
        'launch',
        'ekf.launch.py'
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch)
    )

    # ================= SLAM =================
    pkg_bringup = get_package_share_directory('bringup')
    slam_launch = os.path.join(
        pkg_bringup,
        'launch',
        'slam.launch.py'
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch)
    )

    return LaunchDescription([
        description,
        lidar,
        realsense,
        ekf,
        slam
    ])