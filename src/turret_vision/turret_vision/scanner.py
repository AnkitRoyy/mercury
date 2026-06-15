#!/usr/bin/env python3
"""
scanner.py
==========
State machine for the face recognition task.

States:
    IDLE       -- waiting for /start
    SCANNING   -- iterating through 21 turret positions
    FINE_TUNE  -- proportional controller to centre face
    FIRE       -- activate laser, wait, publish done
    DONE       -- publish /complete

Subscribes:
    /start                 (std_msgs/Bool)
    /match_found           (std_msgs/Bool)
    /horizontal_error      (std_msgs/Float32)
    /vertical_error        (std_msgs/Float32)

Publishes:
    /capture_request       (std_msgs/Bool)
    /pan_deg               (std_msgs/Float32)
    /tilt_deg              (std_msgs/Float32)
    /laser_fire            (std_msgs/Bool)
    /complete              (std_msgs/Bool)
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

IDLE = 'IDLE'
SCANNING = 'SCANNING'
FINE_TUNE = 'FINE_TUNE'
FIRE = 'FIRE'
DONE = 'DONE'


class ScannerNode(Node):

    def __init__(self):
        super().__init__('scanner')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('h_positions_deg', [-120.0, -80.0, -40.0, 0.0, 40.0, 80.0, 120.0])
        self.declare_parameter('v_positions_deg', [20.0, 0.0, -20.0])
        self.declare_parameter('settle_time_sec', 0.6)
        self.declare_parameter('fine_tune_px_tol', 20.0)
        self.declare_parameter('fine_tune_gain_h', 0.05)
        self.declare_parameter('fine_tune_gain_v', 0.05)
        self.declare_parameter('fine_tune_timeout', 5.0)
        self.declare_parameter('laser_on_time_sec', 3.0)

        self._h_pos = list(self.get_parameter('h_positions_deg').value)
        self._v_pos = list(self.get_parameter('v_positions_deg').value)
        self._settle = self.get_parameter('settle_time_sec').value
        self._tol = self.get_parameter('fine_tune_px_tol').value
        self._gain_h = self.get_parameter('fine_tune_gain_h').value
        self._gain_v = self.get_parameter('fine_tune_gain_v').value
        self._ft_timeout = self.get_parameter('fine_tune_timeout').value
        self._laser_time = self.get_parameter('laser_on_time_sec').value

        # Build 21-position scan grid (boustrophedon)
        self._grid = []
        for row_idx, tilt in enumerate(self._v_pos):
            pan_row = self._h_pos if row_idx % 2 == 0 else list(reversed(self._h_pos))
            for pan in pan_row:
                self._grid.append((pan, tilt))

        self.get_logger().info(f'Scan grid: {len(self._grid)} positions')

        # ── State ─────────────────────────────────────────────────────────────
        self._state = IDLE
        self._grid_idx = 0
        self._waiting_result = False
        self._match_found = False
        self._h_err = 0.0
        self._v_err = 0.0
        self._cur_pan = 0.0
        self._cur_tilt = 0.0

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Bool, '/start', self._start_cb, 10)
        self.create_subscription(Bool, '/match_found', self._match_cb, 10)
        self.create_subscription(Float32, '/horizontal_error', self._herr_cb, 10)
        self.create_subscription(Float32, '/vertical_error', self._verr_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_capture = self.create_publisher(Bool, '/capture_request', 10)
        self._pub_pan = self.create_publisher(Float32, '/pan_deg', 10)
        self._pub_tilt = self.create_publisher(Float32, '/tilt_deg', 10)
        self._pub_laser = self.create_publisher(Bool, '/laser_fire', 10)
        self._pub_complete = self.create_publisher(Bool, '/complete', 10)

        # Park turret
        self._move_turret(0.0, 0.0)

        self._timer = self.create_timer(0.1, self._loop)
        self.get_logger().info('Scanner ready. Waiting for /start...')

    def _start_cb(self, msg: Bool):
        if msg.data and self._state == IDLE:
            self.get_logger().info('Task started. Beginning scan.')
            self._reset()
            self._state = SCANNING

    def _match_cb(self, msg: Bool):
        if self._waiting_result:
            self._match_found = msg.data
            self._waiting_result = False

    def _herr_cb(self, msg: Float32):
        self._h_err = msg.data

    def _verr_cb(self, msg: Float32):
        self._v_err = msg.data

    def _loop(self):
        if self._state == IDLE:
            return
        elif self._state == SCANNING:
            self._run_scanning()
        elif self._state == FINE_TUNE:
            self._run_fine_tune()
        elif self._state == FIRE:
            self._run_fire()

    def _run_scanning(self):
        if self._waiting_result:
            if hasattr(self, '_capture_sent_t') and time.time() - self._capture_sent_t > self._settle + 1.0:
                self._waiting_result = False
                self._match_found = False
            return

        if self._grid_idx >= len(self._grid):
            self.get_logger().warn('Scan complete — target NOT found.')
            self._pub_complete.publish(Bool(data=False))
            self._state = DONE
            return

        if not hasattr(self, '_scan_step'):
            self._scan_step = 0
            self._scan_step_t = None

        pan, tilt = self._grid[self._grid_idx]

        if self._scan_step == 0:
            self._move_turret(pan, tilt)
            self._scan_step = 1
            self._scan_step_t = time.time()
            self.get_logger().info(f'[{self._grid_idx+1}/{len(self._grid)}] pan={pan:+.0f} tilt={tilt:+.0f}')

        elif self._scan_step == 1:
            if time.time() - self._scan_step_t >= self._settle:
                self._scan_step = 2

        elif self._scan_step == 2:
            self._waiting_result = True
            self._match_found = False
            self._capture_sent_t = time.time()
            self._pub_capture.publish(Bool(data=True))
            self._scan_step = 3

        elif self._scan_step == 3:
            if self._match_found:
                self.get_logger().info(f'TARGET FOUND at position {self._grid_idx+1}. Entering fine-tune.')
                self._state = FINE_TUNE
                self._ft_start = time.time()
                self._scan_step = 0
            else:
                self._grid_idx += 1
                self._scan_step = 0

    def _run_fine_tune(self):
        elapsed = time.time() - self._ft_start

        if not hasattr(self, '_ft_last_capture'):
            self._ft_last_capture = 0.0
        if time.time() - self._ft_last_capture >= 0.2:
            self._pub_capture.publish(Bool(data=True))
            self._ft_last_capture = time.time()

        h_ok = abs(self._h_err) <= self._tol
        v_ok = abs(self._v_err) <= self._tol

        if (h_ok and v_ok) or elapsed >= self._ft_timeout:
            self.get_logger().info(f'Fine-tune complete. h_err={self._h_err:.1f} v_err={self._v_err:.1f}')
            self._state = FIRE
            self._fire_start = time.time()
            return

        new_pan = self._cur_pan - self._gain_h * self._h_err
        new_tilt = self._cur_tilt - self._gain_v * self._v_err
        new_pan = max(-170.0, min(170.0, new_pan))
        new_tilt = max(-80.0, min(80.0, new_tilt))
        self._move_turret(new_pan, new_tilt)

    def _run_fire(self):
        if time.time() - self._fire_start < self._laser_time:
            self._pub_laser.publish(Bool(data=True))
        else:
            self._pub_laser.publish(Bool(data=False))
            self.get_logger().info('Laser OFF. Task COMPLETE.')
            self._pub_complete.publish(Bool(data=True))
            self._state = DONE

    def _move_turret(self, pan_deg: float, tilt_deg: float):
        self._cur_pan = pan_deg
        self._cur_tilt = tilt_deg
        self._pub_pan.publish(Float32(data=float(pan_deg)))
        self._pub_tilt.publish(Float32(data=float(tilt_deg)))

    def _reset(self):
        self._grid_idx = 0
        self._waiting_result = False
        self._match_found = False
        self._h_err = 0.0
        self._v_err = 0.0
        for attr in ('_scan_step', '_ft_last_capture', '_capture_sent_t'):
            if hasattr(self, attr):
                delattr(self, attr)


def main(args=None):
    rclpy.init(args=args)
    node = ScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()