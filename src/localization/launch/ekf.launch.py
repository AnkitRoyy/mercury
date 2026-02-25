from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_loc = get_package_share_directory('localization')
    ekf_yaml = os.path.join(pkg_loc, 'config', 'ekf.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml]
    )

    return LaunchDescription([
        ekf_node
    ])