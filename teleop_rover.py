#!/usr/bin/env python3
"""
teleop_rover.py — Mercury Rover Teleop Receiver
==================================================
Runs on:  Rover  (192.168.88.3)
Recv from: Base  (:5003)

Listens for cmd_vel UDP packets from the base station and republishes
them onto the ROS2 /cmd_vel topic.

Usage:
  source ~/mercury_venv/bin/activate
  source ~/mercury/install/setup.bash
  python3 teleop_rover.py
"""

import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

PORT_CMD_IN = 5003
MAX_UDP     = 65507

VX_LIMIT = 1.0   # m/s safety clamp
WZ_LIMIT = 2.0   # rad/s safety clamp


class TeleopReceiverNode(Node):

    def __init__(self):
        super().__init__("teleop_rover")

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", reliable_qos)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", PORT_CMD_IN))
        self._sock.settimeout(1.0)

        self._running = True
        threading.Thread(target=self._recv_loop, daemon=True).start()

        self.get_logger().info(f"Teleop receiver listening on :{PORT_CMD_IN} → /cmd_vel")

    def _recv_loop(self):
        while self._running:
            try:
                data, addr = self._sock.recvfrom(MAX_UDP)
                msg = json.loads(data.decode())

                if msg.get("type") != "cmd_vel":
                    continue

                vx = float(msg.get("linear_ms",   0.0))
                wz = float(msg.get("angular_rads", 0.0))

                # Safety clamp
                vx = max(-VX_LIMIT, min(VX_LIMIT, vx))
                wz = max(-WZ_LIMIT, min(WZ_LIMIT, wz))

                twist = Twist()
                twist.linear.x  = vx
                twist.angular.z = wz
                self._cmd_pub.publish(twist)

                self.get_logger().debug(
                    f"CMD  vx={vx:+.3f}m/s  wz={wz:+.3f}rad/s  ← {addr[0]}")

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Recv loop: {e}")
                time.sleep(0.05)

    def destroy_node(self):
        self._running = False
        self._sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TeleopReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
