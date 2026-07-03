#!/usr/bin/env python3
"""
scanner.py
==========
State machine for the face recognition task.

States:
    IDLE       -- waiting for /start
    INIT       -- center -> starting tilt -> starting pan (no diagonal jump)
    SCANNING   -- iterating through 21 turret positions (boustrophedon:
                  complete each horizontal row before moving to next tilt)
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
    /fine_tune_active      (std_msgs/Bool)   -- True only while in FINE_TUNE,
                                                 so turret.py knows when it's
                                                 safe to react to pixel error
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

IDLE = 'IDLE'
INIT = 'INIT'
SCANNING = 'SCANNING'
FINE_TUNE = 'FINE_TUNE'
FIRE = 'FIRE'
DONE = 'DONE'


class ScannerNode(Node):

    def __init__(self):
        super().__init__('scanner')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('h_positions_deg', [-120.0, -80.0, -40.0, 0.0, 40.0, 80.0, 120.0])
        self.declare_parameter('v_positions_deg', [0.0, 20.0, 40.0])
        self.declare_parameter('settle_time_sec', 1.5)   # time for servo to move
        self.declare_parameter('hold_time_sec', 1.5)     # time to hold & capture at position
        self.declare_parameter('fine_tune_px_tol', 20.0)
        self.declare_parameter('fine_tune_gain_h', 0.05)
        self.declare_parameter('fine_tune_gain_v', 0.05)
        self.declare_parameter('fine_tune_timeout', 5.0)
        self.declare_parameter('fine_tune_settle_step_sec', 0.3)
        self.declare_parameter('laser_on_time_sec', 3.0)

        self._h_pos = list(self.get_parameter('h_positions_deg').value)
        self._v_pos = list(self.get_parameter('v_positions_deg').value)
        self._settle = self.get_parameter('settle_time_sec').value
        self._hold_time = self.get_parameter('hold_time_sec').value
        self._tol = self.get_parameter('fine_tune_px_tol').value
        self._gain_h = self.get_parameter('fine_tune_gain_h').value
        self._gain_v = self.get_parameter('fine_tune_gain_v').value
        self._ft_timeout = self.get_parameter('fine_tune_timeout').value
        self._ft_settle_step = self.get_parameter('fine_tune_settle_step_sec').value
        self._laser_time = self.get_parameter('laser_on_time_sec').value

        # Build 21-position scan grid (boustrophedon).
        # All h_positions at a given tilt are visited consecutively;
        # tilt only changes between rows. Rows alternate direction so
        # the servo never has to snap back to the start of a row.
        self._grid = []
        for row_idx, tilt in enumerate(self._v_pos):
            pan_row = self._h_pos if row_idx % 2 == 0 else list(reversed(self._h_pos))
            for pan in pan_row:
                self._grid.append((pan, tilt))

        self.get_logger().info(f'Scan grid: {len(self._grid)} positions (boustrophedon)')
        for i, (p, t) in enumerate(self._grid):
            row = i // len(self._h_pos)
            col = i % len(self._h_pos)
            self.get_logger().info(f'  [{i+1:2d}/{len(self._grid)}] row={row} col={col} pan={p:+.0f} tilt={t:+.0f}')

        # ── State ─────────────────────────────────────────────────────────────
        self._state = IDLE
        self._grid_idx = 0
        self._waiting_result = False
        self._match_found = False
        self._h_err = 0.0
        self._v_err = 0.0
        self._cur_pan = 0.0
        self._cur_tilt = 0.0
        self._init_step = 0
        self._init_step_t = None

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
        self._pub_fine_tune_active = self.create_publisher(Bool, '/fine_tune_active', 10)

        # Park turret
        self._move_turret(0.0, 0.0)
        self._pub_fine_tune_active.publish(Bool(data=False))

        self._timer = self.create_timer(0.1, self._loop)
        self.get_logger().info('Scanner ready. Waiting for /start...')

    def _start_cb(self, msg: Bool):
        if msg.data and self._state == IDLE:
            self.get_logger().info('Task started. Beginning initialization.')
            self._reset()
            self._pub_fine_tune_active.publish(Bool(data=False))
            self._state = INIT
            self._init_step = 0
            self._init_step_t = None

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
        elif self._state == INIT:
            self._run_init()
        elif self._state == SCANNING:
            self._run_scanning()
        elif self._state == FINE_TUNE:
            self._run_fine_tune()
        elif self._state == FIRE:
            self._run_fire()

    def _run_init(self):
        """center -> starting tilt (pan stays at 0) -> starting pan (edge)"""
        if self._init_step == 0:
            self.get_logger().info('[INIT] At center (0, 0), settling...')
            self._init_step = 1
            self._init_step_t = time.time()

        elif self._init_step == 1:
            if time.time() - self._init_step_t >= self._settle * 2:
                self._init_step = 2
                self._init_step_t = None

        elif self._init_step == 2:
            start_tilt = self._v_pos[0]
            self.get_logger().info(f'[INIT] Moving to starting tilt: pan=0.0 tilt={start_tilt:+.0f}')
            self._move_turret(0.0, start_tilt)
            self._init_step = 3
            self._init_step_t = time.time()

        elif self._init_step == 3:
            if time.time() - self._init_step_t >= self._settle * 2:
                self._init_step = 4
                self._init_step_t = None

        elif self._init_step == 4:
            start_pan = self._grid[0][0]
            start_tilt = self._v_pos[0]
            self.get_logger().info(f'[INIT] Moving to first grid position: pan={start_pan:+.0f} tilt={start_tilt:+.0f}')
            self._move_turret(start_pan, start_tilt)
            self._init_step = 5
            self._init_step_t = time.time()

        elif self._init_step == 5:
            if time.time() - self._init_step_t >= self._settle * 2:
                self.get_logger().info('Init complete. Starting grid scan.')
                self._state = SCANNING
                self._grid_idx = 0
                self._init_step = 0

    def _run_scanning(self):
        if self._waiting_result:
            if hasattr(self, '_capture_sent_t') and time.time() - self._capture_sent_t > self._hold_time + self._settle + 1.0:
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
            elapsed = time.time() - self._capture_sent_t
            # Re-request capture a couple times while holding, in case the
            # recognition node needs more than one frame.
            if elapsed < self._hold_time and int(elapsed * 10) % 3 == 0:
                self._pub_capture.publish(Bool(data=True))

            if self._match_found:
                self.get_logger().info(f'TARGET FOUND at position {self._grid_idx+1}. Entering fine-tune.')
                self._state = FINE_TUNE
                self._ft_start = time.time()
                self._scan_step = 0
                self._pub_fine_tune_active.publish(Bool(data=True))
            elif elapsed >= self._hold_time:
                self._grid_idx += 1
                self._scan_step = 0
                self._waiting_result = False

    def _run_fine_tune(self):
        elapsed = time.time() - self._ft_start

        if not hasattr(self, '_ft_settle_until'):
            # First tick after entering FINE_TUNE: nothing has moved yet,
            # so there's nothing to wait on. Allow immediate capture.
            self._ft_settle_until = 0.0

        now = time.time()

        # ── Settle gate ────────────────────────────────────────────────────
        # Servos take real time to physically travel. Previously we fired a
        # fresh correction on every 10Hz tick regardless of whether the last
        # correction had actually landed, so the controller was reacting to
        # frames captured mid-motion — that's what caused the oscillation /
        # divergence (pan and tilt slamming into their clamps repeatedly).
        # Now: don't request a capture or apply a new correction until the
        # settle window from the LAST move has elapsed.
        if now < self._ft_settle_until:
            return

        if not hasattr(self, '_ft_last_capture'):
            self._ft_last_capture = 0.0
        if now - self._ft_last_capture >= 0.2:
            self._pub_capture.publish(Bool(data=True))
            self._ft_last_capture = now

        h_ok = abs(self._h_err) <= self._tol
        v_ok = abs(self._v_err) <= self._tol

        if (h_ok and v_ok) or elapsed >= self._ft_timeout:
            self.get_logger().info(f'Fine-tune complete. h_err={self._h_err:.1f} v_err={self._v_err:.1f}')
            self._pub_fine_tune_active.publish(Bool(data=False))
            self._state = FIRE
            self._fire_start = time.time()
            return

        # ── Delta clamp ───────────────────────────────────────────────────
        # Cap how much a single correction can move the turret, so one bad
        # frame (mid-motion blur, momentary misdetection) can't fling it
        # across the whole range in one step.
        max_step_deg = 8.0

        raw_pan_delta = self._gain_h * self._h_err
        raw_tilt_delta = -self._gain_v * self._v_err
        pan_delta = max(-max_step_deg, min(max_step_deg, raw_pan_delta))
        tilt_delta = max(-max_step_deg, min(max_step_deg, raw_tilt_delta))

        new_pan = max(-116.0, min(116.0, self._cur_pan + pan_delta))
        new_tilt = max(0.0, min(100.0, self._cur_tilt + tilt_delta))
        self._move_turret(new_pan, new_tilt)

        # Wait for the servo to physically arrive before reacting again.
        self._ft_settle_until = now + self._ft_settle_step

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
        for attr in ('_scan_step', '_ft_last_capture', '_capture_sent_t', '_ft_settle_until'):
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