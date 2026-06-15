#!/usr/bin/env python3
"""
health.py
=========
Merged system health + watchdog monitoring node.

Tracks:
  - Running vs expected nodes
  - Topic liveness (odom, scan, imu)
  - Critical node crashes
  - TF frame availability

Publishes:
  /system_status  (std_msgs/String)  - Health summary JSON
  /system_alerts  (std_msgs/String)  - Alert list JSON
"""

import json
import subprocess
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.parameter import Parameter
# Optional imports for topic monitoring
try:
    from nav_msgs.msg import Odometry
    _HAVE_NAV = True
except ImportError:
    _HAVE_NAV = False

try:
    from sensor_msgs.msg import LaserScan, Imu
    _HAVE_SENSOR = True
except ImportError:
    _HAVE_SENSOR = False

# Suggested fixes for common failures
FIX_HINTS: dict[str, str] = {
    '/slam_toolbox': 'Check slam_toolbox installation: sudo apt install ros-$ROS_DISTRO-slam-toolbox',
    '/ekf_filter_node': 'Check robot_localization pkg and ekf.yaml topic names',
    '/controller_server': 'Check nav2_params.yaml and DWB critic config',
    '/planner_server': 'Verify global_costmap.yaml and map availability',
    '/bt_navigator': 'Check BT XML file path in nav2_params.yaml',
    '/robot_state_publisher': 'Verify xacro file path is correct',
    '/diff_drive_controller': 'Check controller_manager and hardware interface',
    'odom_timeout': 'Odometry silent - check diff_drive_controller or simulation',
    'scan_timeout': 'LaserScan silent - check LiDAR driver and /scan topic',
    'imu_timeout': 'IMU silent - check realsense2_camera node',
    'tf_missing': 'TF frame missing - robot_state_publisher or EKF may have crashed',
}


class HealthNode(Node):
    """Merged system monitor and watchdog functionality."""

    def __init__(self):
        super().__init__('health')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('check_interval', 2.0)
        self.declare_parameter('topic_timeout', 5.0)
        self.declare_parameter(
            'expected_nodes',
            Parameter.Type.STRING_ARRAY
        )

        self.declare_parameter(
            'critical_nodes',
            Parameter.Type.STRING_ARRAY
        )
        self.declare_parameter('monitored_topics', [
            '/diff_drive_controller/odom',
            '/scan',
            '/imu'
        ])
        self.declare_parameter(
            'tf_pairs',
            Parameter.Type.STRING_ARRAY
        )

        self._check_interval = self.get_parameter('check_interval').value
        self._topic_timeout = self.get_parameter('topic_timeout').value
        self._expected_nodes = self.get_parameter('expected_nodes').value
        self._critical_nodes = self.get_parameter('critical_nodes').value
        self._monitored_topics = self.get_parameter('monitored_topics').value
        self._tf_pairs = self.get_parameter('tf_pairs').value

        # ── Topic liveness tracking ──────────────────────────────────────────
        self._last_seen: dict[str, float] = {}
        self._start_time = time.time()
        self._seen_nodes: set[str] = set()
        self._launch_order: list[dict] = []

        # ── Publishers ────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/system_status', 10)
        self._alert_pub = self.create_publisher(String, '/system_alerts', 10)

        # ── Subscribers for liveness ─────────────────────────────────────────
        for topic in self._monitored_topics:
            self._last_seen[topic] = time.time()
            self._subscribe_to_topic(topic)

        # ── Timer ────────────────────────────────────────────────────────────
        self.create_timer(self._check_interval, self._check)

        self.get_logger().info(f'Health node started - checking every {self._check_interval}s')
        self.get_logger().info(f'Expecting {len(self._expected_nodes)} nodes')

    def _subscribe_to_topic(self, topic: str):
        """Subscribe to a topic just to track its liveness."""
        if 'odom' in topic and _HAVE_NAV:
            self.create_subscription(Odometry, topic, lambda _: self._touch(topic), 10)
        elif 'scan' in topic and _HAVE_SENSOR:
            self.create_subscription(LaserScan, topic, lambda _: self._touch(topic), 10)
        elif 'imu' in topic and _HAVE_SENSOR:
            self.create_subscription(Imu, topic, lambda _: self._touch(topic), 10)

    def _touch(self, topic: str):
        self._last_seen[topic] = time.time()

    def _get_running_nodes(self) -> set[str]:
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True, text=True, timeout=5.0
            )
            return {line.strip() for line in result.stdout.splitlines() if line.strip()}
        except Exception as e:
            self.get_logger().warn(f'Failed to list nodes: {e}')
            return set()

    def _track_launch_order(self, running: set[str]):
        for node in running:
            if node not in self._seen_nodes:
                self._seen_nodes.add(node)
                elapsed = time.time() - self._start_time
                self._launch_order.append({
                    'node': node,
                    'detected_at_s': round(elapsed, 2)
                })
                self.get_logger().info(f'New node detected: {node} (+{elapsed:.1f}s)')

    def _make_alert(self, level: str, category: str, subject: str, message: str, fix: str = '') -> dict:
        if not fix:
            for pattern, hint in FIX_HINTS.items():
                if pattern in subject:
                    fix = hint
                    break
        return {
            'level': level,
            'category': category,
            'subject': subject,
            'message': message,
            'suggested_fix': fix,
            'timestamp': time.time(),
        }

    def _check(self):
        now = time.time()
        running = self._get_running_nodes()
        self._track_launch_order(running)

        alerts = []
        missing = []
        healthy = []

        # ── Check expected nodes ─────────────────────────────────────────────
        for node in self._expected_nodes:
            if node in running:
                healthy.append(node)
            else:
                missing.append(node)
                if node in self._critical_nodes:
                    alerts.append(self._make_alert(
                        'ERROR', 'node_crash', node,
                        f'Critical node not running: {node}'
                    ))

        # ── Check topic liveness ─────────────────────────────────────────────
        for topic, last_seen in self._last_seen.items():
            silence = now - last_seen
            if silence > self._topic_timeout:
                hint_key = topic.split('/')[-1] + '_timeout'
                alerts.append(self._make_alert(
                    'WARN', 'topic_inactive', topic,
                    f'Topic silent for {silence:.1f}s (timeout={self._topic_timeout}s)'
                ))

        # ── Check TF availability (quick check via topic) ────────────────────
        system_silence = now - self._last_seen.get('/system_status', now)
        if system_silence > self._topic_timeout * 2:
            for pair in self._tf_pairs:
                alerts.append(self._make_alert(
                    'WARN', 'tf_failure', pair,
                    f'TF frame {pair} may be unavailable (system_status silent)',
                    fix=FIX_HINTS.get('tf_missing', '')
                ))

        # ── Publish system status ────────────────────────────────────────────
        status = {
            'timestamp': now,
            'healthy': healthy,
            'missing': missing,
            'total_running': len(running),
            'total_expected': len(self._expected_nodes),
            'all_ok': len(missing) == 0 and len(alerts) == 0,
            'launch_order': self._launch_order[-10:],  # last 10
        }
        self._status_pub.publish(String(data=json.dumps(status)))

        # ── Publish alerts ───────────────────────────────────────────────────
        alert_msg = {
            'timestamp': now,
            'alert_count': len(alerts),
            'alerts': alerts,
            'all_ok': len(alerts) == 0,
        }
        self._alert_pub.publish(String(data=json.dumps(alert_msg)))

        # ── Log summary ──────────────────────────────────────────────────────
        if missing:
            self.get_logger().warn(f'Missing nodes: {missing}')
        if alerts:
            self.get_logger().warn(f'{len(alerts)} alert(s) active')
        else:
            self.get_logger().debug('All systems nominal')


def main(args=None):
    rclpy.init(args=args)
    node = HealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()