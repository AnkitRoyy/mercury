#!/usr/bin/env python3
"""
turret.py
=========
Unified turret interface that auto-detects simulation vs real hardware.

In simulation:
    - Subscribes to /pan_deg, /tilt_deg
    - Publishes to /turret_controller/commands (Float64MultiArray, radians)

On real hardware:
    - Subscribes to /pan_deg, /tilt_deg, /laser_fire
    - Sends serial commands to Arduino/ESP32

Auto-detection:
    - Checks if Gazebo bridge topic exists (by attempting to create a subscriber)
    - Falls back to serial if Gazebo not detected

Subscribes:
    /pan_deg      (std_msgs/Float32)  degrees, 0=forward, +=left
    /tilt_deg     (std_msgs/Float32)  degrees, 0=level, +=up
    /laser_fire   (std_msgs/Bool)     laser on/off

Publishes (sim only):
    /turret_controller/commands (Float64MultiArray)  [pan_rad, tilt_rad]
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Float64MultiArray


class TurretNode(Node):

    def __init__(self):
        super().__init__('turret')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('dry_run', False)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        dry_run = self.get_parameter('dry_run').value

        # ── Auto-detect mode ─────────────────────────────────────────────────
        self._mode = self._detect_mode()
        self.get_logger().info(f'Turret mode: {self._mode.upper()}')

        # ── State ─────────────────────────────────────────────────────────────
        self._cur_pan = 0.0
        self._cur_tilt = 0.0
        self._laser_on = False

        # ── Publishers (sim only) ────────────────────────────────────────────
        self._sim_pub = None
        if self._mode == 'sim':
            self._sim_pub = self.create_publisher(
                Float64MultiArray, '/turret_controller/commands', 10)

        # ── Serial (real hardware) ───────────────────────────────────────────
        self._serial = None
        if self._mode == 'real' and not dry_run:
            try:
                import serial
                self._serial = serial.Serial(port, baud, timeout=0.1)
                import time
                time.sleep(2.0)
                self.get_logger().info(f'Serial connected: {port} @ {baud}')
            except Exception as e:
                self.get_logger().error(f'Serial failed: {e}')
                self.get_logger().warn('Falling back to dry_run mode')
                dry_run = True

        if self._mode == 'real' and dry_run:
            self.get_logger().warn('dry_run=True — commands will be logged only')

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Float32, '/pan_deg', self._pan_cb, 10)
        self.create_subscription(Float32, '/tilt_deg', self._tilt_cb, 10)
        if self._mode == 'real':
            self.create_subscription(Bool, '/laser_fire', self._laser_cb, 10)

        self.get_logger().info('Turret interface ready.')

    def _detect_mode(self) -> str:
        """Detect if running in simulation by checking for Gazebo."""
        try:
            # Try to find any Gazebo-related node
            import subprocess
            result = subprocess.run(
                ['ros2', 'node', 'list', '|', 'grep', '-E', '(gz|gazebo)'],
                shell=True, capture_output=True, text=True, timeout=2.0
            )
            if result.stdout.strip():
                return 'sim'
        except Exception:
            pass
        return 'real'

    def _pan_cb(self, msg: Float32):
        self._cur_pan = max(-170.0, min(170.0, msg.data))
        self._publish()

    def _tilt_cb(self, msg: Float32):
        self._cur_tilt = max(-80.0, min(80.0, msg.data))
        self._publish()

    def _laser_cb(self, msg: Bool):
        if msg.data != self._laser_on:
            cmd = 'L1' if msg.data else 'L0'
            self._send_serial(cmd)
            self._laser_on = msg.data

    def _publish(self):
        if self._mode == 'sim' and self._sim_pub:
            pan_rad = math.radians(self._cur_pan)
            tilt_rad = math.radians(self._cur_tilt)
            msg = Float64MultiArray()
            msg.data = [pan_rad, tilt_rad]
            self._sim_pub.publish(msg)
            self.get_logger().debug(f'Sim: pan={self._cur_pan:+.1f} tilt={self._cur_tilt:+.1f}')
        elif self._mode == 'real':
            self._send_serial(f'P{self._cur_pan:.1f}')
            self._send_serial(f'T{self._cur_tilt:.1f}')

    def _send_serial(self, cmd: str):
        line = cmd + '\n'
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(line.encode('ascii'))
            except Exception as e:
                self.get_logger().error(f'Serial error: {e}')
        else:
            self.get_logger().info(f'[DRY] {cmd}')

    def destroy_node(self):
        if self._serial and self._serial.is_open:
            self._send_serial('L0')
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TurretNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()