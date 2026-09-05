#!/usr/bin/env python3
"""
teensy_driver.py
================
Converts cmd_vel (m/s, rad/s) to wheel RPM and sends to Teensy via serial (micro USB).
Also reads wheel encoder data streamed back from the Teensy over the same serial link.

Subscribes:
    /cmd_vel                (geometry_msgs/Twist) linear & angular velocity from Nav2/controllers

Publishes:
    /wheel_speeds           (Float32MultiArray) [left_rpm, right_rpm] for logging/monitoring
    /teensy_status          (String) serial communication status (optional)
    /wheel_encoders         (Float32MultiArray) [rpmFR, rpmFL, rpmRR, rpmRL]

Parameters:
    wheel_radius            float   default: 0.075  (meters)
    wheel_separation        float   default: 0.44   (meters between left/right wheels)
    max_wheel_rpm           float   default: 240.0  (RPM limit for safety)
    serial_port             string  default: "/dev/ttyACM0"  (micro USB port)
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
    3. Send to Teensy:
       Format: "L{rpm_left},R{rpm_right}\n"  (e.g., "L100.5,R-50.2\n")

Encoder JSON format expected from Teensy (one line per message):
    {"t":"enc","rpmFR":..,"rpmFL":..,"rpmRR":..,"rpmRL":..}
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
import math
import serial
import threading

class TeensyRpmNode(Node):
    def __init__(self):
        super().__init__('teensy')

        # Parameters
        self.declare_parameter('wheel_radius', 0.24)
        self.declare_parameter('wheel_separation', 0.84)
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
        self.circumference = 2 * math.pi * self.wheel_radius

        # Serial connection
        self.serial_conn = None
        self.serial_lock = threading.Lock()

        self._enc_pub = self.create_publisher(Float32MultiArray, '/wheel_encoders', 10)

        self.init_serial()
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

        # Publishers
        self.pub_wheel_speeds = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)
        if self.enable_debug:
            self.pub_status = self.create_publisher(String, '/teensy_status', 10)

        # Subscribers
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.get_logger().info(
            f'Teensy Driver ready\n'
            f'  wheel_radius={self.wheel_radius}m\n'
            f'  wheel_separation={self.wheel_separation}m\n'
            f'  circumference={self.circumference:.4f}m\n'
            f'  max_wheel_rpm={self.max_wheel_rpm}\n'
            f'  serial_port={self.serial_port}\n'
            f'  serial_baud={self.serial_baud}\n'
            f'  serial_enabled={self.enable_serial}\n'
            f'  Subscribed to /cmd_vel\n'
            f'  Publishing /wheel_encoders'
        )

    def init_serial(self):
        """Initialize serial connection to Teensy via micro USB."""
        if not self.enable_serial:
            self.get_logger().warn('Serial communication disabled')
            return

        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baud,
                timeout=1.0
            )
            self.get_logger().info(f'Serial connected: {self.serial_port} @ {self.serial_baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {e}')
            self.serial_conn = None

    def _read_loop(self):
        import json, time
        while rclpy.ok():
            try:
                if self.serial_conn is None or not self.serial_conn.is_open:
                    time.sleep(1.0)
                    self.init_serial()
                    continue
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if not line or not line.startswith('{'):
                    continue
                data = json.loads(line)
                if data.get('t') == 'enc':
                    msg = Float32MultiArray()
                    msg.data = [float(data['rpmFR']), float(data['rpmFL']),
                                float(data['rpmRR']), float(data['rpmRL'])]
                    self._enc_pub.publish(msg)
            except (json.JSONDecodeError, KeyError):
                pass
            except Exception as e:
                self.get_logger().warn(f'Read error: {e}')
                self.serial_conn = None
                time.sleep(1.0)

    def send_to_teensy(self, rpm_left: float, rpm_right: float):
        """Send RPM values to Teensy via serial."""
        if not self.enable_serial:
            return

        if self.serial_conn is None:
            self.get_logger().debug('Serial connection is None, skipping write')
            return

        try:
            with self.serial_lock:
                cmd = f"L{rpm_left:.1f},R{rpm_right:.1f}\n"
                bytes_written = self.serial_conn.write(cmd.encode())
                self.get_logger().info(f'Wrote {bytes_written} bytes to serial: {cmd.strip()}')

                if self.enable_debug:
                    status = String()
                    status.data = f'Sent to Teensy: {cmd.strip()}'
                    self.pub_status.publish(status)
        except Exception as e:
            self.get_logger().error(f'Serial write error ({type(e).__name__}): {e}')
            self.serial_conn = None

    def cmd_vel_cb(self, msg: Twist):
        """Convert cmd_vel to wheel RPM and send to Teensy."""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        self.get_logger().info(f'Received cmd_vel: linear={linear_vel:.3f} m/s, angular={angular_vel:.3f} rad/s')

        half_separation = self.wheel_separation / 2.0
        v_left = linear_vel - (angular_vel * half_separation)
        v_right = linear_vel + (angular_vel * half_separation)

        rpm_left = (v_left / self.circumference) * 60.0
        rpm_right = (v_right / self.circumference) * 60.0

        rpm_left = max(-self.max_wheel_rpm, min(self.max_wheel_rpm, rpm_left))
        rpm_right = max(-self.max_wheel_rpm, min(self.max_wheel_rpm, rpm_right))

        wheel_speeds = Float32MultiArray()
        wheel_speeds.data = [rpm_left, rpm_right]
        self.pub_wheel_speeds.publish(wheel_speeds)

        self.send_to_teensy(rpm_left, rpm_right)

    def destroy_node(self):
        """Cleanup serial connection on shutdown."""
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
                self.get_logger().info('Serial connection closed')
            except:
                pass
        super().destroy_node()

def main():
    rclpy.init()
    node = TeensyRpmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()