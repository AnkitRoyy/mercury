from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # When EKF owns map→odom (GPS-anchored mode), set this to 'false'
    # to avoid TF conflict. Default 'true' for sim where SLAM owns map→odom.
    publish_map_odom_tf_arg = DeclareLaunchArgument(
        'publish_map_odom_tf',
        default_value='true',
        description='Let slam_toolbox publish map→odom TF (false when EKF owns it)'
    )
    publish_map_odom_tf = LaunchConfiguration('publish_map_odom_tf')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'resolution': 0.05,
            'max_laser_range': 10.0,
            'minimum_travel_distance': 0.02,
            'minimum_travel_heading': 0.02,
            'scan_queue_size': 20,
            'throttle_scans': 1,
            'map_update_interval': 0.3,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'publish_map_odom_tf': publish_map_odom_tf,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        publish_map_odom_tf_arg,
        slam_node
    ])
