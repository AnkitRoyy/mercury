from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ── Lane detection node ───────────────────────────────────────────
    # Replaces lane_costmap.  Publishes:
    #   /lane_center_error  (std_msgs/Float32)  — pixel offset from lane centre
    #   /lane_visible       (std_msgs/Bool)     — whether a lane is detected
    #   /lane_debug/image   (sensor_msgs/Image) — BEV overlay for RViz / rqt
    lane_detection_node = Node(
        package='perception',
        executable='lane_detection',
        name='lane_detection_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image_raw',
            'bev_width': 640,
            'bev_height': 480,
            # Set False in headless / CI environments
            'show_debug': True,
        }]
    )

    return LaunchDescription([
        lane_detection_node,
    ])