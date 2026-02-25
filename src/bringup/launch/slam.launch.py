from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    slam_params = {
        'use_sim_time': False,
        'odom_frame': 'odom',
        'base_frame': 'base_link',
        'map_frame': 'map',
        'scan_topic': '/scan',
        'mode': 'mapping'
    }

    slam_node = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen',
    parameters=[{
        'use_sim_time': False,
        'odom_frame': 'odom',
        'base_frame': 'base_link',
        'map_frame': 'map',
        'scan_topic': '/scan',
        'mode': 'mapping',
        'resolution': 0.05,
        'max_laser_range': 12.0,
        'minimum_travel_distance': 0.1,
        'minimum_travel_heading': 0.1,
        'transform_timeout': 0.2,
    'tf_buffer_duration': 30.0,
    }]
)

    return LaunchDescription([
        slam_node
    ])