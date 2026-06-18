#!/usr/bin/env python3

import time
import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Bool


class TurretNode(Node):

    def __init__(self):
        super().__init__('turret')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('dry_run', False)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.dry_run = self.get_parameter('dry_run').value

        self._laser_on = False

        # Serial connection
        self._serial = None

        if not self.dry_run:
            try:
                self._serial = serial.Serial(
                    port=port,
                    baudrate=baud,
                    timeout=0.1
                )

                time.sleep(2.0)

                self.get_logger().info(
                    f'Serial connected: {port} @ {baud}'
                )

            except Exception as e:
                self.get_logger().error(
                    f'Failed to open serial port: {e}'
                )

                self.dry_run = True

        if self.dry_run:
            self.get_logger().warn(
                'Running in dry_run mode.'
            )

        # Face tracking topics
        self.create_subscription(
            Float32,
            '/horizontal_error',
            self._pan_cb,
            10
        )

        self.create_subscription(
            Float32,
            '/vertical_error',
            self._tilt_cb,
            10
        )

        # Optional laser topic
        self.create_subscription(
            Bool,
            '/laser_fire',
            self._laser_cb,
            10
        )

        self.get_logger().info('Turret interface ready.')

    def _pan_cb(self, msg: Float32):

        error = msg.data

        # Tune this gain if needed
        pan_cmd = int(90 + error * 0.5)

        pan_cmd = max(0, min(180, pan_cmd))

        self.get_logger().info(
            f'H error={error:.1f} -> P{pan_cmd}'
        )

        self._send_serial(f'P{pan_cmd}')

    def _tilt_cb(self, msg: Float32):

        error = msg.data

        # Invert sign if tilt moves opposite direction
        tilt_cmd = int(90 - error * 0.2)

        tilt_cmd = max(0, min(180, tilt_cmd))

        self.get_logger().info(
            f'V error={error:.1f} -> T{tilt_cmd}'
        )

        self._send_serial(f'T{tilt_cmd}')

    def _laser_cb(self, msg: Bool):

        if msg.data == self._laser_on:
            return

        self._laser_on = msg.data

        cmd = 'L1' if msg.data else 'L0'

        self._send_serial(cmd)

    def _send_serial(self, cmd: str):

        self.get_logger().info(
            f'Sending serial: {cmd}'
        )

        if self.dry_run:
            return

        try:
            self._serial.write((cmd + '\n').encode('ascii'))
            self._serial.flush()

        except Exception as e:
            self.get_logger().error(
                f'Serial write failed: {e}'
            )

    def destroy_node(self):

        try:
            if self._serial and self._serial.is_open:
                self._serial.write(b'L0\n')
                self._serial.close()

        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = TurretNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()