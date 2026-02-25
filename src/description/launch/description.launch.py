from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_desc = get_package_share_directory('description')
    urdf_file = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file]),
                    value_type=str
                )
            }],
            output='screen'
        )
    ])