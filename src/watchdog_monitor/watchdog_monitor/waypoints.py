#!/usr/bin/env python3
"""
waypoints.py
============
Waypoint detection node.

Subscribes:
    /diff_drive_controller/odom  (nav_msgs/Odometry)

Publishes:
    /waypoint_reached  (std_msgs/String)  - JSON event on each detection
    /waypoint_status   (std_msgs/String)  - Periodic JSON overview

Parameters:
    spawn_x, spawn_y     - Robot spawn position (world coordinates)
    waypoints            - Flat list [x1,y1, x2,y2, ...] in world coords
    waypoint_names       - Names for each waypoint
    arrival_radius       - Distance to count as "reached"
    status_interval      - Seconds between status publishes
    odom_topic           - Odometry topic to subscribe to
"""

import json
import math
import time
from typing import Any
from rclpy.parameter import Parameter
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from nav_msgs.msg import Odometry
    _HAVE_NAV = True
except ImportError:
    _HAVE_NAV = False
    Odometry = None


class Waypoint:
    """Internal state for a single waypoint."""

    def __init__(self, idx: int, name: str, x: float, y: float, radius: float):
        self.idx = idx
        self.name = name
        self.x = x
        self.y = y
        self.radius = radius
        self.reached = False
        self.reached_at: float | None = None
        self.reach_count = 0

    def distance_to(self, rx: float, ry: float) -> float:
        return math.hypot(rx - self.x, ry - self.y)

    def to_dict(self) -> dict:
        return {
            'index': self.idx,
            'name': self.name,
            'x': self.x,
            'y': self.y,
            'radius': self.radius,
            'reached': self.reached,
            'reach_count': self.reach_count,
            'reached_at': self.reached_at,
        }


class WaypointsNode(Node):

    def __init__(self):
        super().__init__('waypoints')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter(
            'waypoints',
            Parameter.Type.DOUBLE_ARRAY
        )

        self.declare_parameter(
            'waypoint_names',
            Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter('arrival_radius', 0.5)
        self.declare_parameter('status_interval', 1.0)
        self.declare_parameter('odom_topic', '/diff_drive_controller/odom')
        
        spawn_x = self.get_parameter('spawn_x').value
        spawn_y = self.get_parameter('spawn_y').value
        waypoints_flat = self.get_parameter('waypoints').value
        names = self.get_parameter('waypoint_names').value
        radius = self.get_parameter('arrival_radius').value
        self._status_interval = self.get_parameter('status_interval').value
        odom_topic = self.get_parameter('odom_topic').value

        # ── Validate waypoints ───────────────────────────────────────────────
        if len(waypoints_flat) % 2 != 0:
            self.get_logger().error('Waypoints must have an even number of values (x,y pairs)')
            waypoints_flat = waypoints_flat[:-1]

        # ── Build waypoints ───────────────────────────────────────────────────
        self._waypoints: list[Waypoint] = []
        for i in range(0, len(waypoints_flat), 2):
            x = waypoints_flat[i]
            y = waypoints_flat[i + 1]
            name = names[i // 2] if (i // 2) < len(names) else f'WP-{(i//2)+1}'
            self._waypoints.append(Waypoint((i//2)+1, name, x, y, radius))

        self.get_logger().info(f'Loaded {len(self._waypoints)} waypoints, radius={radius}m')
        for wp in self._waypoints:
            self.get_logger().info(f'  {wp.name}: ({wp.x:.2f}, {wp.y:.2f})')

        # ── Robot pose ────────────────────────────────────────────────────────
        self._robot_x = spawn_x
        self._robot_y = spawn_y
        self._pose_received = False

        # ── Publishers ────────────────────────────────────────────────────────
        self._event_pub = self.create_publisher(String, '/waypoint_reached', 10)
        self._status_pub = self.create_publisher(String, '/waypoint_status', 10)

        # ── Subscriber ────────────────────────────────────────────────────────
        if _HAVE_NAV:
            self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        else:
            self.get_logger().warn('nav_msgs not available - pose will not update')

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(0.1, self._detection_loop)      # 10 Hz detection
        self.create_timer(self._status_interval, self._publish_status)

        self.get_logger().info('Waypoints node ready')

    def _odom_cb(self, msg: Odometry):
        # Odometry is relative to spawn (0,0 at boot). Add spawn offset for world coords.
        self._robot_x = self.get_parameter('spawn_x').value + msg.pose.pose.position.x
        self._robot_y = self.get_parameter('spawn_y').value + msg.pose.pose.position.y
        self._pose_received = True

    def _detection_loop(self):
        if not self._pose_received:
            return

        rx, ry = self._robot_x, self._robot_y

        for wp in self._waypoints:
            dist = wp.distance_to(rx, ry)

            # Arrival detection
            if not wp.reached and dist <= wp.radius:
                wp.reached = True
                wp.reached_at = time.time()
                wp.reach_count += 1

                self.get_logger().info(
                    f'*** WAYPOINT REACHED: {wp.name} (#{wp.reach_count}) '
                    f'at ({rx:.3f}, {ry:.3f}), dist={dist:.3f}m ***'
                )

                event = {
                    'event': 'waypoint_reached',
                    'waypoint': wp.to_dict(),
                    'robot_x': rx,
                    'robot_y': ry,
                    'distance': round(dist, 4),
                    'timestamp': wp.reached_at,
                }
                self._event_pub.publish(String(data=json.dumps(event)))

            # Re-arm: allow re-detection if robot leaves the zone
            elif wp.reached and dist > wp.radius * 2.0:
                wp.reached = False
                self.get_logger().debug(f'{wp.name} re-armed (moved {dist:.2f}m away)')

    def _publish_status(self):
        reached_count = sum(1 for wp in self._waypoints if wp.reach_count > 0)
        all_reached = reached_count == len(self._waypoints)

        status = {
            'timestamp': time.time(),
            'robot_x': round(self._robot_x, 4),
            'robot_y': round(self._robot_y, 4),
            'pose_received': self._pose_received,
            'waypoints': [wp.to_dict() for wp in self._waypoints],
            'total': len(self._waypoints),
            'reached_at_least_once': reached_count,
            'all_completed': all_reached,
        }
        self._status_pub.publish(String(data=json.dumps(status)))

        if all_reached:
            self.get_logger().info('✓ All waypoints have been reached!')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()