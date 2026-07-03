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
        self._fine_tune_active = False

        # Track current commanded position in DEGREES (matches firmware's
        # calibrated PAN:/TILT: format: pan +116..-116, tilt 0..40ish)
        self._cur_pan_deg = 0.0
        self._cur_tilt_deg = 0.0

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

        # ── Absolute-degree topics from scanner.py (grid scan + fine-tune moves) ──
        self.create_subscription(
            Float32,
            '/pan_deg',
            self._pan_deg_cb,
            10
        )

        self.create_subscription(
            Float32,
            '/tilt_deg',
            self._tilt_deg_cb,
            10
        )

        # NOTE: /horizontal_error and /vertical_error are intentionally NOT
        # subscribed to here anymore. turret.py previously ran its own
        # independent proportional controller off these raw topics AT THE
        # SAME TIME scanner.py's fine-tune state machine was computing and
        # publishing /pan_deg + /tilt_deg. Both were gated by the same
        # /fine_tune_active flag, so once fine-tune started, two separate
        # uncoordinated control loops were both moving the turret and
        # overwriting each other's target position — that's what caused the
        # divergence/oscillation seen on hardware. scanner.py is now the
        # SOLE owner of fine-tune correction logic; turret.py only ever
        # translates /pan_deg + /tilt_deg (absolute degrees) to serial.

        # Optional laser topic
        self.create_subscription(
            Bool,
            '/laser_fire',
            self._laser_cb,
            10
        )

        # Tracks scanner's fine-tune state for logging/visibility only.
        # No longer gates anything in this node — turret.py doesn't compute
        # corrections anymore, it just relays /pan_deg + /tilt_deg.
        self.create_subscription(
            Bool,
            '/fine_tune_active',
            self._fine_tune_active_cb,
            10
        )

        # Send home position through the calibrated path too
        self._move_deg(0.0, 0.0)

        self.get_logger().info('Turret interface ready.')

    # ── Absolute degree commands (from scanner grid scan) ──────────────────
    def _pan_deg_cb(self, msg: Float32):
        self._cur_pan_deg = msg.data
        self._send_pan_tilt()

    def _tilt_deg_cb(self, msg: Float32):
        self._cur_tilt_deg = msg.data
        self._send_pan_tilt()

    # ── Fine-tune gate ───────────────────────────────────────────────────────
    def _fine_tune_active_cb(self, msg: Bool):
        self._fine_tune_active = msg.data
        self.get_logger().info(
            f'Fine-tune active: {self._fine_tune_active}'
        )

    # ── Pixel-error fine-tune: REMOVED ────────────────────────────────────
    # _herr_cb / _verr_cb used to independently drive the turret off raw
    # /horizontal_error and /vertical_error. Deleted — scanner.py's
    # FINE_TUNE state machine is the only thing allowed to compute
    # corrections; it publishes the result as /pan_deg and /tilt_deg,
    # which _pan_deg_cb / _tilt_deg_cb above already handle.

    def _laser_cb(self, msg: Bool):

        if msg.data == self._laser_on:
            return

        self._laser_on = msg.data

        cmd = 'L1' if msg.data else 'L0'

        self._send_serial(cmd)

    # ── Serial helpers ───────────────────────────────────────────────────────
    def _move_deg(self, pan_deg: float, tilt_deg: float):
        self._cur_pan_deg = pan_deg
        self._cur_tilt_deg = tilt_deg
        self._send_pan_tilt()

    def _send_pan_tilt(self):
        # Uses the firmware's calibrated "PAN:<deg>,TILT:<deg>" format,
        # which applies moveTurret()'s 87 - 0.75*panDeg / 80 + tiltDeg
        # mapping on the Arduino side. Do NOT also scale here — that
        # would double-convert and scramble positions again.
        cmd = f'PAN:{self._cur_pan_deg:.1f},TILT:{self._cur_tilt_deg:.1f}'
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