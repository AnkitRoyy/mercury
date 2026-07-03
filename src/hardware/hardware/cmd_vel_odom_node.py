#!/usr/bin/env python3
"""
Open-loop odom source. No encoders available (single command-only ESP32).
Publishes /odom with TRUSTED TWIST (last commanded cmd_vel) and
UNTRUSTED POSE (huge covariance) — EKF should only fuse the twist half,
never the position half. Configure ekf_real.yaml odom0_config accordingly:
    odom0_config: [false, false, false,  false, false, false,
                   true,  true,  false,  false, false, true,
                   false, false, false]
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class CmdVelOdomNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_odom_node')
        self._twist = Twist()
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        self._pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_timer(0.02, self._tick)  # 50 Hz

    def _cb(self, msg: Twist):
        self._twist = msg

    def _tick(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.twist.twist = self._twist
        # pose left at zero / high covariance — EKF must ignore it
        odom.pose.covariance[0] = 1e6
        odom.pose.covariance[7] = 1e6
        odom.pose.covariance[35] = 1e6
        odom.twist.covariance[0] = 0.05
        odom.twist.covariance[35] = 0.05
        self._pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()