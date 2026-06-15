#!/usr/bin/env python3
"""
trigger.py
==========
Bridge node: connects waypoint detector → turret vision pipeline.

Subscribes:
    /waypoint_reached    (std_msgs/String)  JSON from watchdog_monitor
    /complete            (std_msgs/Bool)    result from scanner

Publishes:
    /start               (std_msgs/Bool)    start the face task
    /done                (std_msgs/Bool)    navigation may proceed to WP-3
    /state               (std_msgs/String)  human-readable state
    /cmd_vel             (geometry_msgs/Twist) zero-twist to hold vehicle

Parameters:
    trigger_waypoint_name   str    default: "WP-2"
    trigger_waypoint_index  int    default: 2
    hard_timeout_sec        float  default: 55.0
    stop_vehicle_on_trigger bool   default: true
    cmd_vel_topic           str    default: /cmd_vel
"""

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

STATE_IDLE = 'IDLE'
STATE_TRIGGERED = 'TRIGGERED'
STATE_COMPLETE = 'COMPLETE'


class TriggerNode(Node):

    def __init__(self):
        super().__init__('trigger')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('trigger_waypoint_name', 'WP-2')
        self.declare_parameter('trigger_waypoint_index', 2)
        self.declare_parameter('hard_timeout_sec', 55.0)
        self.declare_parameter('stop_vehicle_on_trigger', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self._trigger_name = self.get_parameter('trigger_waypoint_name').value
        self._trigger_idx = self.get_parameter('trigger_waypoint_index').value
        self._hard_timeout = self.get_parameter('hard_timeout_sec').value
        self._stop_vehicle = self.get_parameter('stop_vehicle_on_trigger').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # ── State ─────────────────────────────────────────────────────────────
        self._state = STATE_IDLE
        self._task_start_t = None
        self._task_result = None

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_start = self.create_publisher(Bool, '/start', 10)
        self._pub_done = self.create_publisher(Bool, '/done', 10)
        self._pub_state = self.create_publisher(String, '/state', 10)
        self._pub_cmdvel = self.create_publisher(Twist, cmd_vel_topic, 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(String, '/waypoint_reached', self._waypoint_cb, 10)
        self.create_subscription(Bool, '/complete', self._complete_cb, 10)

        # ── Watchdog timer ────────────────────────────────────────────────────
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(f'Trigger ready — waypoint="{self._trigger_name}", timeout={self._hard_timeout}s')
        self._publish_state(STATE_IDLE)

    def _waypoint_cb(self, msg: String):
        if self._state != STATE_IDLE:
            return

        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        wp = event.get('waypoint', {})
        wp_name = wp.get('name', '')
        wp_index = wp.get('index', -1)

        if wp_name != self._trigger_name and wp_index != self._trigger_idx:
            return

        self.get_logger().info(f'WP-2 reached ("{wp_name}"). Starting face task.')
        self._state = STATE_TRIGGERED
        self._task_start_t = time.time()
        self._task_result = None

        # Stop the robot
        if self._stop_vehicle:
            self._pub_cmdvel.publish(Twist())

        self._pub_start.publish(Bool(data=True))
        self._publish_state(STATE_TRIGGERED)

    def _complete_cb(self, msg: Bool):
        if self._state != STATE_TRIGGERED:
            return

        self._task_result = msg.data
        elapsed = time.time() - self._task_start_t
        result_str = 'SUCCESS' if msg.data else 'NOT FOUND'
        self.get_logger().info(f'Face task {result_str} in {elapsed:.1f}s')
        self._finish()

    def _watchdog(self):
        if self._state != STATE_TRIGGERED:
            return

        if self._stop_vehicle:
            self._pub_cmdvel.publish(Twist())

        elapsed = time.time() - self._task_start_t

        if not hasattr(self, '_last_log'):
            self._last_log = 0.0
        if elapsed - self._last_log >= 5.0:
            self._last_log = elapsed
            self.get_logger().info(f'Scanning... {elapsed:.0f}s elapsed')

        if elapsed >= self._hard_timeout:
            self.get_logger().warn(f'Hard timeout after {elapsed:.1f}s — forcing exit.')
            self._task_result = False
            self._finish()

    def _finish(self):
        self._state = STATE_COMPLETE
        self._pub_done.publish(Bool(data=True))
        self._publish_state(STATE_COMPLETE)
        self.get_logger().info('/done=True published. Navigation may proceed.')

    def _publish_state(self, state: str):
        payload = {
            'node': 'trigger',
            'state': state,
            'trigger_waypoint': self._trigger_name,
            'task_result': self._task_result,
            'elapsed': round(time.time() - self._task_start_t, 2) if self._task_start_t else None,
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()