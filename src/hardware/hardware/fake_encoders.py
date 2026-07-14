#!/usr/bin/env python3
"""
fake_encoders.py
================
Publishes fake /wheel_encoders (Float32MultiArray: [rpmFR, rpmFL, rpmRR, rpmRL])
so wheel_odom_node.py can run without the Teensy connected.

Default: all wheels same RPM -> robot drives straight.
Set FAKE_RPM = 0.0 to test pure standing-still odom (still needed for TF to exist).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

FAKE_RPM = 20.0   # same value on all 4 wheels = straight line, no turning
RATE_HZ  = 20.0   # match your real encoder publish rate if you know it


class FakeEncoders(Node):
    def __init__(self):
        super().__init__('fake_encoders')
        self.pub = self.create_publisher(Float32MultiArray, '/wheel_encoders', 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info(f'Publishing fake /wheel_encoders at {RATE_HZ} Hz, rpm={FAKE_RPM}')

    def _tick(self):
        msg = Float32MultiArray()
        msg.data = [FAKE_RPM, FAKE_RPM, FAKE_RPM, FAKE_RPM]  # FR, FL, RR, RL
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeEncoders()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()