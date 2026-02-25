from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -------- Include Robot Description --------
    pkg_description = get_package_share_directory('description')
    description_launch = os.path.join(
        pkg_description,
        'launch',
        'description.launch.py'
    )

    description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch)
    )

    # -------- RPLidar A3 --------
    pkg_rplidar = get_package_share_directory('rplidar_ros')
    rplidar_launch = os.path.join(
        pkg_rplidar,
        'launch',
        'rplidar_a3_launch.py'
    )

    rplidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch),
        launch_arguments={
             'serial_port': '/dev/ttyUSB1',
            'frame_id': 'laser_link'
        }.items()
    )

    return LaunchDescription([
        description_node,
        rplidar_node
    ])