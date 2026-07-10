#!/usr/bin/env python3
"""
send_gps_goal.py — Lat/Lon → /final_goal bridge
=================================================
Converts a GPS coordinate (latitude, longitude) into a local map-frame
PoseStamped and publishes it on /final_goal, using navsat_transform_node's
/fromLL service.

Usage:
    ros2 run mission send_gps_goal --lat 28.75321 --lon 77.11765
    ros2 run mission send_gps_goal --lat 28.75321 --lon 77.11765 --heading 90

The --heading flag is optional (degrees, 0=North CW). If omitted, the goal
orientation defaults to identity (w=1.0) and the planner determines the
approach angle automatically.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL


class SendGpsGoalNode(Node):

    def __init__(self):
        super().__init__('send_gps_goal')
        self._client = self.create_client(FromLL, '/fromLL')
        self._pub = self.create_publisher(PoseStamped, '/final_goal', 10)

    def convert_and_publish(self, lat: float, lon: float,
                            heading: float | None = None) -> bool:
        """Convert lat/lon → map frame via /fromLL, then publish to /final_goal."""

        # ── Wait for service ──────────────────────────────────────────────────
        self.get_logger().info('Waiting for /fromLL service (navsat_transform_node)...')
        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'navsat_transform_node not running / datum not locked yet — '
                'is real_localization.launch.py up?'
            )
            return False

        # ── Call /fromLL ──────────────────────────────────────────────────────
        req = FromLL.Request()
        req.ll_point.latitude = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude = 0.0

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('/fromLL service call failed or timed out.')
            return False

        map_point = future.result().map_point

        # ── Build PoseStamped ─────────────────────────────────────────────────
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position = map_point

        if heading is not None:
            # Convert compass heading (0=North, CW) to ENU yaw (0=East, CCW)
            yaw_rad = math.radians(90.0 - heading)
            goal.pose.orientation.z = math.sin(yaw_rad / 2.0)
            goal.pose.orientation.w = math.cos(yaw_rad / 2.0)
        else:
            goal.pose.orientation.w = 1.0

        # ── Safety countdown ──────────────────────────────────────────────────
        self.get_logger().info(
            f'Converted (lat={lat}, lon={lon}) → map frame '
            f'(x={map_point.x:.2f}, y={map_point.y:.2f})'
        )
        self.get_logger().warn(
            'Publishing to /final_goal in 2 seconds... Ctrl+C to abort.'
        )

        try:
            time.sleep(1.0)
            self.get_logger().warn('1...')
            time.sleep(1.0)
        except KeyboardInterrupt:
            self.get_logger().info('Aborted by operator.')
            return False

        self._pub.publish(goal)
        self.get_logger().info(
            f'Published /final_goal  x={map_point.x:.2f}  y={map_point.y:.2f}'
        )
        return True


def main():
    parser = argparse.ArgumentParser(
        description='Convert GPS lat/lon to /final_goal in map frame'
    )
    parser.add_argument('--lat', type=float, required=True,
                        help='Target latitude (decimal degrees)')
    parser.add_argument('--lon', type=float, required=True,
                        help='Target longitude (decimal degrees)')
    parser.add_argument('--heading', type=float, default=None,
                        help='Optional goal heading (degrees, 0=North CW). '
                             'If omitted, planner chooses approach angle.')

    # rclpy.init() must be called before argparse to handle ROS args
    rclpy.init()
    args = parser.parse_args()

    node = SendGpsGoalNode()
    try:
        ok = node.convert_and_publish(args.lat, args.lon, args.heading)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
