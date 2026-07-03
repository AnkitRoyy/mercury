#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String
import math
import serial
import threading


class Esp32RpmNode(Node):

    def __init__(self):
        super().__init__('esp32_motor')

        self.declare_parameter('wheel_radius', 0.075)
        self.declare_parameter('wheel_separation', 0.44)
        self.declare_parameter('max_wheel_rpm', 240.0)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
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

        self.circumference = 2 * math.pi * self.wheel_radius
        self.serial_conn = None
        self.serial_lock = threading.Lock()

        self.init_serial()

        self.pub_wheel_speeds = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)
        if self.enable_debug:
            self.pub_status = self.create_publisher(String, '/teensy_status', 10)

        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.get_logger().info(
            f'ESP32 RPM Converter ready (command-only, no feedback)\n'
            f'  serial_port={self.serial_port} @ {self.serial_baud}'
        )

    def init_serial(self):
        if not self.enable_serial:
            self.get_logger().warn('Serial communication disabled')
            return
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port, baudrate=self.serial_baud, timeout=1.0)
            self.get_logger().info(f'Serial connected: {self.serial_port} @ {self.serial_baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {e}')
            self.serial_conn = None

    def send_to_esp32(self, rpm_left: float, rpm_right: float):
        if not self.enable_serial or self.serial_conn is None:
            return
        try:
            with self.serial_lock:
                cmd = f"L{rpm_left:.1f},R{rpm_right:.1f}\n"
                self.serial_conn.write(cmd.encode())
                if self.enable_debug:
                    self.pub_status.publish(String(data=f'Sent: {cmd.strip()}'))
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.serial_conn = None

    def cmd_vel_cb(self, msg: Twist):
        half_sep = self.wheel_separation / 2.0
        v_left = msg.linear.x - (msg.angular.z * half_sep)
        v_right = msg.linear.x + (msg.angular.z * half_sep)

        rpm_left = max(-self.max_wheel_rpm, min(self.max_wheel_rpm,
                        (v_left / self.circumference) * 60.0))
        rpm_right = max(-self.max_wheel_rpm, min(self.max_wheel_rpm,
                        (v_right / self.circumference) * 60.0))

        self.pub_wheel_speeds.publish(Float32MultiArray(data=[rpm_left, rpm_right]))
        self.send_to_esp32(rpm_left, rpm_right)

    def destroy_node(self):
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Esp32RpmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()