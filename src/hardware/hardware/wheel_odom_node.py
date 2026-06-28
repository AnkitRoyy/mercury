#!/usr/bin/env python3
"""
wheel_odom_node.py
==================
Converts 4-wheel encoder RPMs → nav_msgs/Odometry on /odom.

Subscribes:
    /wheel_encoders  (Float32MultiArray)  [rpmFR, rpmFL, rpmRR, rpmRL]

Publishes:
    /odom            (nav_msgs/Odometry)
    /tf              odom → base_link

Differential drive: average left (FL+RL)/2 and right (FR+RR)/2 sides.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class WheelOdomNode(Node):

    def __init__(self):
        super().__init__('wheel_odom_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('wheel_radius',    0.075)   # m
        self.declare_parameter('wheel_separation', 0.44)   # m  (left↔right)
        self.declare_parameter('publish_tf',       True)

        self.r   = self.get_parameter('wheel_radius').value
        self.sep = self.get_parameter('wheel_separation').value
        self.pub_tf = self.get_parameter('publish_tf').value

        # ── State ─────────────────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self._last_stamp = None

        # ── I/O ───────────────────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_br    = TransformBroadcaster(self)

        self.create_subscription(Float32MultiArray, '/wheel_encoders',
                                 self._enc_cb, 10)

        self.get_logger().info(
            f'WheelOdom ready  r={self.r} m  sep={self.sep} m')

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _rpm_to_mps(rpm: float, r: float) -> float:
        return rpm * 2.0 * math.pi * r / 60.0

    # ── callback ──────────────────────────────────────────────────────────────

    def _enc_cb(self, msg: Float32MultiArray):
        now = self.get_clock().now()

        if self._last_stamp is None:
            self._last_stamp = now
            return

        dt = (now - self._last_stamp).nanoseconds * 1e-9
        self._last_stamp = now

        if dt <= 0.0 or dt > 0.5:   # ignore stale / first packet
            return

        rpm_fr, rpm_fl, rpm_rr, rpm_rl = msg.data[:4]

        # Average each side
        v_right = self._rpm_to_mps((rpm_fr + rpm_rr) / 2.0, self.r)
        v_left  = self._rpm_to_mps((rpm_fl + rpm_rl) / 2.0, self.r)

        v   = (v_right + v_left) / 2.0          # linear  (m/s)
        w   = (v_right - v_left) / self.sep      # angular (rad/s)

        # Integrate pose
        self.x   += v * math.cos(self.yaw) * dt
        self.y   += v * math.sin(self.yaw) * dt
        self.yaw += w * dt

        # Quaternion from yaw
        cy, sy = math.cos(self.yaw / 2.0), math.sin(self.yaw / 2.0)

        stamp = now.to_msg()

        # ── Odometry message ──────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w

        # Covariances — tuned conservatively for wheel slip
        # [x, y, z, roll, pitch, yaw] diagonal
        pc = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        odom.pose.covariance = pc

        tc = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.2]
        odom.twist.covariance = tc

        self._odom_pub.publish(odom)

        # ── TF broadcast ─────────────────────────────────────────────────────
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
