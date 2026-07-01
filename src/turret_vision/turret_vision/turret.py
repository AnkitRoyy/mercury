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
        # Degrees-per-pixel-error gain for fine-tune (error is in px)
        self.declare_parameter('fine_tune_gain_h', 0.05)
        self.declare_parameter('fine_tune_gain_v', 0.05)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.dry_run = self.get_parameter('dry_run').value
        self._gain_h = self.get_parameter('fine_tune_gain_h').value
        self._gain_v = self.get_parameter('fine_tune_gain_v').value

        self._laser_on = False
        self._fine_tune_active = False

        # Track current commanded position in DEGREES (matches firmware's
        # calibrated PAN:/TILT: format: pan +120..-120, tilt 0..40ish)
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

        # ── Legacy pixel-error topics (kept for compatibility, routed through
        #    the same calibrated degree path instead of raw P/T) ──
        self.create_subscription(
            Float32,
            '/horizontal_error',
            self._herr_cb,
            10
        )

        self.create_subscription(
            Float32,
            '/vertical_error',
            self._verr_cb,
            10
        )

        # Optional laser topic
        self.create_subscription(
            Bool,
            '/laser_fire',
            self._laser_cb,
            10
        )

        # Gate for pixel-error fine-tune: only act on /horizontal_error and
        # /vertical_error when scanner says we're actually in FINE_TUNE.
        # Otherwise a stray face in-frame during the 21-position grid scan
        # would nudge the servo off its commanded grid position.
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

    # ── Pixel-error fine-tune (converted to degree deltas, same serial path) ─
    # Only applied when scanner has confirmed a match and entered FINE_TUNE.
    # During the 21-position grid scan this flag is False, so any face
    # detected mid-sweep is ignored and does not move the turret off-grid.
    def _herr_cb(self, msg: Float32):
        if not self._fine_tune_active:
            return

        error = msg.data
        delta = -self._gain_h * error
        self._cur_pan_deg = max(-120.0, min(120.0, self._cur_pan_deg + delta))
        self.get_logger().info(
            f'H error={error:.1f} -> pan_deg={self._cur_pan_deg:.1f}'
        )
        self._send_pan_tilt()

    def _verr_cb(self, msg: Float32):
        if not self._fine_tune_active:
            return

        error = msg.data
        delta = -self._gain_v * error
        self._cur_tilt_deg = max(0.0, min(100.0, self._cur_tilt_deg + delta))
        self.get_logger().info(
            f'V error={error:.1f} -> tilt_deg={self._cur_tilt_deg:.1f}'
        )
        self._send_pan_tilt()

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
        # which applies moveTurret()'s 87 - 0.725*panDeg / 80 + tiltDeg
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