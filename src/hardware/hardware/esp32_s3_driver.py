#!/usr/bin/env python3
"""
esp32_s3_driver.py
===================
Converts cmd_vel (m/s, rad/s) to wheel RPM and sends to ESP32-S3 via serial.
Also reads telemetry/status strings from ESP32-S3 and publishes to /esp32_s3_status.
Supports auto-reconnection if USB serial is disconnected or re-plugged.

Subscribes:
    /cmd_vel                (geometry_msgs/Twist) linear & angular velocity from Nav2/controllers

Publishes:
    /wheel_speeds           (Float32MultiArray) [left_rpm, right_rpm] for logging/monitoring
    /esp32_s3_status        (String) serial communication and firmware status

Parameters:
    wheel_radius            float   default: 0.16   (meters)
    wheel_separation        float   default: 0.54   (meters between left/right wheels)
    max_wheel_rpm           float   default: 240.0  (RPM limit for safety)
    serial_port             string  default: "/dev/ttyACM0"  (serial port)
    serial_baud             int     default: 115200  (baud rate)
    enable_serial           bool    default: True    (enable serial communication)
    enable_debug            bool    default: True    (debug output)

Conversion:
    1. Calculate wheel velocities (m/s):
       v_left  = linear_vel - (angular_vel × wheel_separation/2)
       v_right = linear_vel + (angular_vel × wheel_separation/2)
    2. Convert to RPM:
       circumference = 2π × wheel_radius
       rpm = (velocity_m_s / circumference) × 60
    3. Send to ESP32-S3:
       Format: "L{rpm_left},R{rpm_right}\n"  (e.g., "L100.5,R-50.2\n")
"""

import math
import os
import threading

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class ESP32S3RpmNode(Node):
    def __init__(self):
        super().__init__('esp32_s3')

        # Parameters
        self.declare_parameter('wheel_radius', 0.16)
        self.declare_parameter('wheel_separation', 0.54)
        self.declare_parameter('max_wheel_rpm', 240.0)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('enable_serial', True)
        self.declare_parameter('enable_debug', True)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_wheel_rpm = self.get_parameter('max_wheel_rpm').value
        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = self.get_parameter('serial_baud').value
        self.enable_serial = self.get_parameter('enable_serial').value
        self.enable_debug = self.get_parameter('enable_debug').value

        # Pre-calculate circumference
        self.circumference = 2.0 * math.pi * self.wheel_radius

        # Serial connection state
        self.serial_conn = None
        self.serial_lock = threading.Lock()
        self._last_warn_time = None

        self.init_serial()

        # Publishers
        self.pub_wheel_speeds = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)
        self.pub_status = self.create_publisher(String, '/esp32_s3_status', 10)

        # Subscribers
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # Periodic timers: read serial at 20Hz, check connection every 1s
        self.create_timer(0.01, self._read_serial)
        self.create_timer(1.0, self._check_connection)

        self.get_logger().info(
            f'ESP32-S3 Driver ready\n'
            f'  wheel_radius={self.wheel_radius}m\n'
            f'  wheel_separation={self.wheel_separation}m\n'
            f'  circumference={self.circumference:.4f}m\n'
            f'  max_wheel_rpm={self.max_wheel_rpm}\n'
            f'  serial_port={self.serial_port}\n'
            f'  serial_baud={self.serial_baud}\n'
            f'  serial_enabled={self.enable_serial}\n'
            f'  Subscribed to /cmd_vel\n'
            f'  Publishing /wheel_speeds, /esp32_s3_status'
        )

    def init_serial(self):
        """Initialize serial connection to ESP32-S3 with auto-fallback to /dev/ttyUSB0."""
        if not self.enable_serial:
            self.get_logger().warn('Serial communication disabled')
            return

        with self.serial_lock:
            if self.serial_conn is not None and self.serial_conn.is_open:
                return

            candidate_ports = [self.serial_port]
            for p in ['/dev/ttyUSB1', '/dev/ttyUSB0', '/dev/ttyACM0']:
                if p not in candidate_ports:
                    candidate_ports.append(p)

            for port in candidate_ports:
                if not os.path.exists(port):
                    continue
                try:
                    self.serial_conn = serial.Serial(
                        port=port,
                        baudrate=self.serial_baud,
                        timeout=0.05
                    )
                    self.serial_conn.reset_input_buffer()
                    self.serial_port = port
                    self.get_logger().info(f'Serial connected: {self.serial_port} @ {self.serial_baud} baud')
                    return
                except serial.SerialException as e:
                    self.get_logger().warn(f'Failed to open serial port {port}: {e}')
                    self.serial_conn = None

            self.get_logger().warn(
                f'Serial port {self.serial_port} not available yet (will retry in background)',
                throttle_duration_sec=5.0
            )

    def _check_connection(self):
        """Periodically reconnect if serial was lost or plugged in after launch."""
        if self.enable_serial:
            with self.serial_lock:
                is_closed = (self.serial_conn is None or not self.serial_conn.is_open)
            if is_closed:
                self.init_serial()

    def _read_serial(self):
        """Read responses and status logs from ESP32."""
        if not self.enable_serial or self.serial_conn is None:
            return

        try:
            with self.serial_lock:
                if not self.serial_conn.is_open:
                    return
                while self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        if self.enable_debug:
                            status_msg = String()
                            status_msg.data = line
                            self.pub_status.publish(status_msg)
                        if '[BOOT]' in line or '[ERROR]' in line:
                            self.get_logger().info(f'[ESP32] {line}')
                        elif '[STATUS]' in line:
                            self.get_logger().info(f'[ESP32] {line}', throttle_duration_sec=0.1)
        except Exception as e:
            self.get_logger().error(f'Serial read error ({type(e).__name__}): {e}')
            self._close_serial()

    def _close_serial(self):
        with self.serial_lock:
            if self.serial_conn is not None:
                try:
                    self.serial_conn.close()
                except Exception:
                    pass
                self.serial_conn = None

    def send_to_esp32_s3(self, rpm_left: float, rpm_right: float):
        """Send RPM values to ESP32-S3 via serial."""
        if not self.enable_serial:
            return

        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().warn('Serial connection not open; skipping command', throttle_duration_sec=3.0)
            return

        try:
            with self.serial_lock:
                cmd = f"L{rpm_left:.1f},R{rpm_right:.1f}\n"
                bytes_written = self.serial_conn.write(cmd.encode())
                if self.enable_debug:
                    self.get_logger().info(
                        f'Wrote {bytes_written} bytes to serial: {cmd.strip()}',
                        throttle_duration_sec=0.1
                    )
                    status = String()
                    status.data = f'Sent to ESP32-S3: {cmd.strip()}'
                    self.pub_status.publish(status)
        except Exception as e:
            self.get_logger().error(f'Serial write error ({type(e).__name__}): {e}')
            self._close_serial()

    def cmd_vel_cb(self, msg: Twist):
        """Convert cmd_vel to wheel RPM and send to ESP32-S3."""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        half_separation = self.wheel_separation / 2.0
        v_left = linear_vel - (angular_vel * half_separation)
        v_right = linear_vel + (angular_vel * half_separation)

        rpm_left = (v_left / self.circumference) * 60.0
        rpm_right = (v_right / self.circumference) * 60.0

        rpm_left = max(-self.max_wheel_rpm, min(self.max_wheel_rpm, rpm_left))
        rpm_right = max(-self.max_wheel_rpm, min(self.max_wheel_rpm, rpm_right))

        self.get_logger().info(
            f'Received cmd_vel: v={linear_vel:.2f} m/s, w={angular_vel:.2f} rad/s -> L={rpm_left:.1f}, R={rpm_right:.1f} RPM',
            throttle_duration_sec=0.1
        )

        wheel_speeds = Float32MultiArray()
        wheel_speeds.data = [rpm_left, rpm_right]
        self.pub_wheel_speeds.publish(wheel_speeds)

        self.send_to_esp32_s3(rpm_left, rpm_right)

    def destroy_node(self):
        """Cleanup serial connection on shutdown."""
        self._close_serial()
        super().destroy_node()


def main():
    rclpy.init()
    node = ESP32S3RpmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
