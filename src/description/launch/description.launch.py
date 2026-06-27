from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    declare_xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        description='Path to the xacro file'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        declare_xacro_file_arg,
        declare_use_sim_time_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', LaunchConfiguration('xacro_file')]),
                    value_type=str
                ),
                'use_sim_time': use_sim_time
            }],
            output='screen'
        )
    ])
