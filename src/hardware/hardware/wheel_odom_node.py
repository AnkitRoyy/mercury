#!/usr/bin/env python3
"""
wheel_odom_node.py
==================
Converts wheel encoder RPMs → nav_msgs/Odometry on /odom.
If physical encoders are not connected/publishing, automatically falls back
to Open-Loop Integration of /cmd_vel for smooth navigation without hardware encoders.

Subscribes:
    /wheel_encoders  (std_msgs/Float32MultiArray) - physical encoders (optional)
    /cmd_vel         (geometry_msgs/Twist)       - velocity commands (fallback)

Publishes:
    /odom            (nav_msgs/Odometry)
    /tf              odom → base_link
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class WheelOdomNode(Node):

    def __init__(self):
        super().__init__('wheel_odom_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('wheel_radius',     0.16)   # m (32cm diameter)
        self.declare_parameter('wheel_separation', 0.54)   # m (left↔right)
        self.declare_parameter('publish_tf',       True)

        self.r   = self.get_parameter('wheel_radius').value
        self.sep = self.get_parameter('wheel_separation').value
        self.pub_tf = self.get_parameter('publish_tf').value

        # ── State ─────────────────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.v   = 0.0
        self.w   = 0.0
        self._last_stamp = None
        self._last_encoder_time = None
        self._last_cmd_vel_time = None
        self.cmd_v = 0.0
        self.cmd_w = 0.0

        # ── I/O ───────────────────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_br    = TransformBroadcaster(self)

        self.create_subscription(Float32MultiArray, '/wheel_encoders',
                                 self._enc_cb, 10)
        self.create_subscription(Twist, '/cmd_vel',
                                 self._cmd_vel_cb, 10)

        # 20Hz loop for smooth integration & TF publishing
        self._timer = self.create_timer(0.05, self._update_loop)

        self.get_logger().info(
            f'WheelOdom ready (Encoder + Open-Loop Fallback)  r={self.r} m  sep={self.sep} m')

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _rpm_to_mps(rpm: float, r: float) -> float:
        return rpm * 2.0 * math.pi * r / 60.0

    def _cmd_vel_cb(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self._last_cmd_vel_time = self.get_clock().now()

    def _enc_cb(self, msg: Float32MultiArray):
        now = self.get_clock().now()
        self._last_encoder_time = now

        data_len = len(msg.data)
        if data_len >= 4:
            rpm_fr, rpm_fl, rpm_rr, rpm_rl = msg.data[:4]
            v_right = self._rpm_to_mps((rpm_fr + rpm_rr) / 2.0, self.r)
            v_left  = self._rpm_to_mps((rpm_fl + rpm_rl) / 2.0, self.r)
        elif data_len >= 2:
            rpm_left, rpm_right = msg.data[:2]
            v_left  = self._rpm_to_mps(rpm_left, self.r)
            v_right = self._rpm_to_mps(rpm_right, self.r)
        else:
            return

        self.v = (v_right + v_left) / 2.0
        self.w = (v_right - v_left) / self.sep

    def _update_loop(self):
        now = self.get_clock().now()

        if self._last_stamp is None:
            self._last_stamp = now
            return

        dt = (now - self._last_stamp).nanoseconds * 1e-9
        self._last_stamp = now

        if dt <= 0.0 or dt > 0.5:
            return

        has_encoders = (self._last_encoder_time is not None and 
                        (now - self._last_encoder_time).nanoseconds * 1e-9 < 0.3)

        if not has_encoders:
            # Fallback to Open-Loop /cmd_vel integration
            has_cmd_vel = (self._last_cmd_vel_time is not None and 
                           (now - self._last_cmd_vel_time).nanoseconds * 1e-9 < 0.5)
            if has_cmd_vel:
                self.v = self.cmd_v
                self.w = self.cmd_w
            else:
                self.v = 0.0
                self.w = 0.0

        # Integrate pose
        self.x   += self.v * math.cos(self.yaw) * dt
        self.y   += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        # Publish odom & TF
        cy, sy = math.cos(self.yaw / 2.0), math.sin(self.yaw / 2.0)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        odom.twist.twist.linear.x  = self.v
        odom.twist.twist.angular.z = self.w

        pc = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        odom.pose.covariance = pc
        odom.twist.covariance = pc

        self._odom_pub.publish(odom)

        if self.pub_tf:
            tf = TransformStamped()
            tf.header.stamp    = stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id  = 'base_link'
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z    = sy
            tf.transform.rotation.w    = cy
            self._tf_br.sendTransform(tf)


def main():
    rclpy.init()
    node = WheelOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
