"""
Usage:
  ros2 launch turret_vision turret_vision.launch.py target_image:=/path/to/face.jpg
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    target_image_arg = DeclareLaunchArgument(
        'target_image',
        default_value='',
        description='Absolute path to the target face image'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('turret_vision'), 'config', 'vision_params.yaml'
        ]),
        description='Path to parameters YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    turret_node = Node(
        package='turret_vision',
        executable='turret',
        name='turret',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )

    recognition_node = Node(
        package='turret_vision',
        executable='recognition',
        name='recognition',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'target_image_path': LaunchConfiguration('target_image'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('/camera/image_raw', '/turret_camera/image_raw'),
        ],
    )

    scanner_node = Node(
        package='turret_vision',
        executable='scanner',
        name='scanner',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )

    trigger_node = Node(
        package='turret_vision',
        executable='trigger',
        name='trigger',
        output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )

    return LaunchDescription([
        target_image_arg,
        params_arg,
        use_sim_time_arg,
        turret_node,
        recognition_node,
        scanner_node,
        trigger_node,
    ])