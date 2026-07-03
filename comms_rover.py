"""
comms_rover.py — Mercury Rover Communication Bridge
=====================================================
Runs on:  Rover  (192.168.88.3)
Sends to: Base   (192.168.88.2)

Data sources:
  PRIMARY  — logger_node.py topics (start it before this)
  EXTRA    — direct ROS2 subs for data not in logger_node
  LANE CAM — direct USB webcam capture via OpenCV (NOT a ROS topic)

Port layout:
  5000  UDP  →Base   IMU
  5001  UDP  →Base   GPS
  5002  UDP  →Base   Odom + Encoders + Nav + System
  5003  UDP  ←Base   Drive commands  (recv → /cmd_vel)
  5004  UDP  →Base   Lane detection state
  5005  UDP  →Base   Face/turret detection state
  5006  UDP  →Base   Waypoint + mission events
  5007  UDP  →Base   Alerts + watchdog
  8554  UDP  →Base   Camera frames (JPEG, lane [webcam] + turret [ROS topic])

Usage:
  # Step 1 — start logger node (required)
  ros2 run logger logger_node

  # Step 2 — start this bridge
  source ~/mercury_venv/bin/activate
  source ~/mercury/install/setup.bash
  python3 comms_rover.py [--base-ip 192.168.88.2] [--webcam-device /dev/video0]

NOTE: cv_bridge is intentionally NOT used here — it was compiled against
NumPy 1.x and segfaults on NumPy 2.x. Turret image conversion is done
directly from the raw ROS Image message bytes using numpy + OpenCV.

NOTE ON LANE CAMERA: the lane camera is now captured directly from a USB
webcam via cv2.VideoCapture — it is NOT subscribed as a ROS2 topic. This
mirrors the working YUYV 640x480 @ ~18-20Hz config established earlier,
with explicit v4l2-ctl controls (including 50Hz power line frequency
correction) applied before capture starts.
"""

import argparse
import math
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, JointState, LaserScan, NavSatFix
from std_msgs.msg import Bool, Float32, String, Float32MultiArray

import json
import socket
import struct

# ── Config ────────────────────────────────────────────────────────────────────

BASE_IP      = "192.168.88.2"
PORT_IMU     = 5000
PORT_GPS     = 5001
PORT_ODOM    = 5002   # odom + encoders + nav + system stats
PORT_CMD_IN  = 5003   # recv drive commands from base
PORT_LANE    = 5004
PORT_FACE    = 5005
PORT_MISSION = 5006
PORT_ALERTS  = 5007
PORT_VIDEO   = 8554   # JPEG frames

MAX_UDP      = 65507
VIDEO_QUALITY = 60    # JPEG quality 0-100
VIDEO_WIDTH   = 320   # downscale for UDP
VIDEO_HEIGHT  = 240

# ── USB webcam (lane camera) config ─────────────────────────────────────────
WEBCAM_DEVICE     = "/dev/video0"
WEBCAM_CAP_WIDTH  = 640
WEBCAM_CAP_HEIGHT = 480
WEBCAM_FPS        = 20
WEBCAM_FOURCC     = "YUYV"

# ── Helpers ───────────────────────────────────────────────────────────────────

def _ts() -> float:
    return time.time()

def _now_str() -> str:
    return time.strftime("%H:%M:%S")

class UDPSender:
    """Thread-safe UDP sender with per-port sockets."""
    def __init__(self):
        self._socks = {}
        self._lock  = threading.Lock()

    def _sock(self, port: int) -> socket.socket:
        if port not in self._socks:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 262144)
            self._socks[port] = s
        return self._socks[port]

    def send(self, host: str, port: int, payload: dict):
        try:
            data = json.dumps(payload, separators=(',', ':')).encode()
            if len(data) > MAX_UDP:
                print(f"[WARN] Packet too large for port {port}: {len(data)} bytes")
                return
            with self._lock:
                self._sock(port).sendto(data, (host, port))
        except OSError as e:
            print(f"[SEND ERR] :{port} — {e}")

    def send_bytes(self, host: str, port: int, data: bytes):
        """For raw bytes (video frames)."""
        try:
            with self._lock:
                self._sock(port).sendto(data, (host, port))
        except OSError as e:
            print(f"[VIDEO ERR] :{port} — {e}")

    def close(self):
        for s in self._socks.values():
            s.close()


# ── Webcam capture thread (lane camera) ─────────────────────────────────────

class WebcamCapture:
    """
    Opens a USB webcam directly via OpenCV/V4L2 and continuously reads frames.
    Applies the same v4l2-ctl controls established during earlier debugging
    (YUYV @ 640x480 ~18-20Hz, 50Hz power line frequency correction) before
    capture starts. Runs on its own thread; latest frame is read via
    get_latest_frame().
    """

    def __init__(self, device: str, width: int, height: int, fps: int, fourcc: str):
        self._device = device
        self._width  = width
        self._height = height
        self._fps    = fps
        self._fourcc = fourcc

        self._cap = None
        self._lock = threading.Lock()
        self._frame = None
        self._frame_ts = 0.0
        self._running = False
        self._thread = None

    def _apply_v4l2_controls(self):
        """Explicit v4l2-ctl controls, matching the earlier working config."""
        controls = [
            f"power_line_frequency=1",   # 1 = 50Hz correction
        ]
        for c in controls:
            try:
                subprocess.run(
                    ["v4l2-ctl", "-d", self._device, "-c", c],
                    check=False, capture_output=True, timeout=2.0,
                )
            except Exception as e:
                print(f"[WEBCAM] v4l2-ctl control '{c}' failed: {e}")

    def start(self):
        self._apply_v4l2_controls()

        self._cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        if self._fourcc:
            fourcc_code = cv2.VideoWriter_fourcc(*self._fourcc)
            self._cap.set(cv2.CAP_PROP_FOURCC, fourcc_code)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv2.CAP_PROP_FPS,          self._fps)

        if not self._cap.isOpened():
            print(f"[WEBCAM] Failed to open {self._device}")
            return False

        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print(f"[WEBCAM] Capturing {self._device} "
              f"{self._width}x{self._height} @ {self._fps}fps ({self._fourcc})")
        return True

    def _read_loop(self):
        fail_count = 0
        while self._running:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                fail_count += 1
                if fail_count % 30 == 1:
                    print(f"[WEBCAM] Read failed ({fail_count} total) — retrying")
                time.sleep(0.05)
                continue
            fail_count = 0
            with self._lock:
                self._frame    = frame
                self._frame_ts = time.time()

    def get_latest_frame(self):
        with self._lock:
            if self._frame is None:
                return None, 0.0
            return self._frame.copy(), self._frame_ts

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._cap is not None:
            self._cap.release()


# ── ROS 2 Node ───────────────────────────────────────────────────────────────

class CommsRoverNode(Node):

    def __init__(self, base_ip: str, webcam_device: str):
        super().__init__("comms_rover")
        self._base  = base_ip
        self._udp   = UDPSender()
        # NOTE: CvBridge intentionally removed — compiled against NumPy 1.x,
        # segfaults on NumPy 2.x. Use _ros_img_to_bgr() instead (turret only).

        # Video frame throttle timestamps
        self._video_lane_frame_t   = 0.0
        self._video_turret_frame_t = 0.0
        self._video_fps            = 10.0   # UDP send throttle (independent of capture fps)

        # ── QoS profiles ─────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── Recv socket for drive commands ────────────────────────────────────
        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._recv_sock.bind(("0.0.0.0", PORT_CMD_IN))
        self._recv_sock.settimeout(1.0)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ════════════════════════════════════════════════════════════════════
        # SUBSCRIPTIONS
        # ════════════════════════════════════════════════════════════════════

        # ── [PORT 5000] IMU ───────────────────────────────────────────────────
        self.create_subscription(Imu, "/imu", self._imu_cb, sensor_qos)

        # ── [PORT 5001] GPS ───────────────────────────────────────────────────
        self.create_subscription(NavSatFix, "/gps", self._gps_cb, sensor_qos)

        # ── [PORT 5002] Odometry ──────────────────────────────────────────────
        self.create_subscription(
            Odometry, "/diff_drive_controller/odom", self._odom_cb, sensor_qos)

        # ── [PORT 5002] Joint states / encoders ───────────────────────────────
        self.create_subscription(
            JointState, "/joint_states", self._joint_cb, sensor_qos)

        # ── [PORT 5002] cmd_vel sent to robot ─────────────────────────────────
        self.create_subscription(Twist, "/cmd_vel", self._cmdvel_cb, reliable_qos)

        # ── [PORT 5002] Navigation goal ───────────────────────────────────────
        self.create_subscription(
            PoseStamped, "/goal_pose", self._goal_cb, reliable_qos)

        # ── [PORT 5002] Nav2 action status ────────────────────────────────────
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._navstatus_cb, reliable_qos)

        # ── [PORT 5004] Lane detection ────────────────────────────────────────
        self.create_subscription(
            Float32, "/lane_center_error", self._lane_err_cb, reliable_qos)
        self.create_subscription(
            Bool, "/lane_visible", self._lane_vis_cb, reliable_qos)
        self.create_subscription(
            Bool, "/lane/both_visible", self._lane_both_cb, reliable_qos)

        # ── [PORT 5005] Face/turret detection ─────────────────────────────────
        self.create_subscription(
            Bool,    "/match_found",       self._match_cb,       reliable_qos)
        self.create_subscription(
            Float32, "/horizontal_error",  self._herr_cb,        reliable_qos)
        self.create_subscription(
            Float32, "/vertical_error",    self._verr_cb,        reliable_qos)
        self.create_subscription(
            Bool,    "/start",             self._task_start_cb,  reliable_qos)
        self.create_subscription(
            Bool,    "/complete",          self._task_done_cb,   reliable_qos)
        self.create_subscription(
            Bool,    "/done",              self._mission_done_cb, reliable_qos)

        # ── [PORT 5006] Waypoint / mission ────────────────────────────────────
        self.create_subscription(
            String, "/waypoint_reached", self._wp_reached_cb, reliable_qos)
        self.create_subscription(
            String, "/waypoint_status",  self._wp_status_cb,  reliable_qos)

        # ── [PORT 5007] System health ─────────────────────────────────────────
        self.create_subscription(
            String, "/system_status", self._sysstat_cb, reliable_qos)
        self.create_subscription(
            String, "/system_alerts", self._alert_cb,   reliable_qos)

        # ── [PORT 8554] Turret camera feed — still ROS topic ───────────────────
        self.create_subscription(
            Image, "/turret_camera/image_raw",
            self._turret_cam_cb, sensor_qos)

        # ── [PORT 8554] Lane camera feed — direct USB webcam (NOT ROS) ─────────
        self._webcam = WebcamCapture(
            device=webcam_device,
            width=WEBCAM_CAP_WIDTH, height=WEBCAM_CAP_HEIGHT,
            fps=WEBCAM_FPS, fourcc=WEBCAM_FOURCC,
        )
        if not self._webcam.start():
            self.get_logger().error(
                f"Lane webcam failed to open at {webcam_device} — "
                f"lane video will not be sent"
            )
        # Timer polls the latest webcam frame and sends it, throttled to _video_fps
        self.create_timer(1.0 / self._video_fps, self._lane_webcam_tick)

        # ── [PORT 5002] LiDAR summary ─────────────────────────────────────────
        self.create_subscription(
            LaserScan, "/scan", self._scan_cb, sensor_qos)

        # ── System stats timer ────────────────────────────────────────────────
        try:
            import psutil
            self._psutil = psutil
        except ImportError:
            self._psutil = None
            self.get_logger().warn("psutil not installed — CPU/mem stats disabled")
        self.create_timer(2.0, self._sysres_cb)

        # ── Recv thread ───────────────────────────────────────────────────────
        self._running = True
        threading.Thread(target=self._recv_loop, daemon=True).start()

        # ── TX counters ───────────────────────────────────────────────────────
        self._tx = {k: 0 for k in
            ["imu","gps","odom","enc","lane","face","mission","alert","video_lane","video_turret"]}
        self.create_timer(10.0, self._log_stats)

        self.get_logger().info(
            f"\n{'='*55}\n"
            f"  comms_rover  READY\n"
            f"  Rover  : 192.168.88.3\n"
            f"  Base   : {base_ip}\n"
            f"{'─'*55}\n"
            f"  TX  IMU          → :{PORT_IMU}\n"
            f"  TX  GPS          → :{PORT_GPS}\n"
            f"  TX  ODOM/ENC/NAV → :{PORT_ODOM}\n"
            f"  TX  LANE STATE   → :{PORT_LANE}\n"
            f"  TX  FACE STATE   → :{PORT_FACE}\n"
            f"  TX  MISSION      → :{PORT_MISSION}\n"
            f"  TX  ALERTS       → :{PORT_ALERTS}\n"
            f"  TX  VIDEO        → :{PORT_VIDEO}  (lane=webcam {webcam_device}, turret=ROS topic)\n"
            f"  RX  DRIVE CMDS   ← :{PORT_CMD_IN}\n"
            f"{'='*55}\n"
            f"  NOTE: Start logger_node before this bridge!\n"
            f"  ros2 run logger logger_node\n"
            f"{'='*55}"
        )

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5000 — IMU
    # ══════════════════════════════════════════════════════════════════════════

    def _imu_cb(self, msg: Imu):
        siny = 2.0*(msg.orientation.w*msg.orientation.z + msg.orientation.x*msg.orientation.y)
        cosy = 1.0 - 2.0*(msg.orientation.y**2 + msg.orientation.z**2)
        yaw  = math.degrees(math.atan2(siny, cosy))

        sinp = 2.0*(msg.orientation.w*msg.orientation.y - msg.orientation.z*msg.orientation.x)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))

        sinr = 2.0*(msg.orientation.w*msg.orientation.x + msg.orientation.y*msg.orientation.z)
        cosr = 1.0 - 2.0*(msg.orientation.x**2 + msg.orientation.y**2)
        roll = math.degrees(math.atan2(sinr, cosr))

        self._udp.send(self._base, PORT_IMU, {
            "type":  "imu",
            "ts":    _ts(),
            "rpy_deg": {
                "roll":  round(roll,  2),
                "pitch": round(pitch, 2),
                "yaw":   round(yaw,   2),
            },
            "quaternion": {
                "x": round(msg.orientation.x, 6),
                "y": round(msg.orientation.y, 6),
                "z": round(msg.orientation.z, 6),
                "w": round(msg.orientation.w, 6),
            },
            "angular_velocity_rads": {
                "x": round(msg.angular_velocity.x, 4),
                "y": round(msg.angular_velocity.y, 4),
                "z": round(msg.angular_velocity.z, 4),
            },
            "linear_accel_ms2": {
                "x": round(msg.linear_acceleration.x, 4),
                "y": round(msg.linear_acceleration.y, 4),
                "z": round(msg.linear_acceleration.z, 4),
            },
        })
        self._tx["imu"] += 1

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5001 — GPS
    # ══════════════════════════════════════════════════════════════════════════

    def _gps_cb(self, msg: NavSatFix):
        fix_map = {-1: "NO_FIX", 0: "FIX", 1: "SBAS_DGPS", 2: "RTK"}
        self._udp.send(self._base, PORT_GPS, {
            "type":      "gps",
            "ts":        _ts(),
            "latitude":  round(msg.latitude,  8),
            "longitude": round(msg.longitude, 8),
            "altitude_m": round(msg.altitude, 3),
            "fix_type":  fix_map.get(int(msg.status.status), "UNKNOWN"),
            "fix_code":  int(msg.status.status),
            "covariance_m2": {
                "xx": round(msg.position_covariance[0], 6),
                "yy": round(msg.position_covariance[4], 6),
                "zz": round(msg.position_covariance[8], 6),
            },
        })
        self._tx["gps"] += 1

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5002 — Odometry
    # ══════════════════════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0*(q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0*(q.y**2 + q.z**2)
        yaw  = math.atan2(siny, cosy)

        self._udp.send(self._base, PORT_ODOM, {
            "type": "odometry",
            "ts":   _ts(),
            "position_m": {
                "x": round(msg.pose.pose.position.x, 4),
                "y": round(msg.pose.pose.position.y, 4),
            },
            "yaw_deg":   round(math.degrees(yaw), 3),
            "yaw_rad":   round(yaw, 5),
            "velocity": {
                "linear_ms":    round(msg.twist.twist.linear.x,  4),
                "angular_rads": round(msg.twist.twist.angular.z, 4),
            },
        })
        self._tx["odom"] += 1

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5002 — Joint states / encoders
    # ══════════════════════════════════════════════════════════════════════════

    def _joint_cb(self, msg: JointState):
        self._udp.send(self._base, PORT_ODOM, {
            "type":   "encoders",
            "ts":     _ts(),
            "joints": msg.name,
            "position_rad":  [round(float(p), 4) for p in msg.position],
            "velocity_rads": [round(float(v), 4) for v in msg.velocity],
        })
        self._tx["enc"] += 1

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5002 — cmd_vel being sent to robot
    # ══════════════════════════════════════════════════════════════════════════

    def _cmdvel_cb(self, msg: Twist):
        self._udp.send(self._base, PORT_ODOM, {
            "type":         "cmd_vel",
            "ts":           _ts(),
            "linear_ms":    round(msg.linear.x,  4),
            "angular_rads": round(msg.angular.z, 4),
        })

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5002 — Navigation goal
    # ══════════════════════════════════════════════════════════════════════════

    def _goal_cb(self, msg: PoseStamped):
        self._udp.send(self._base, PORT_ODOM, {
            "type":  "nav_goal",
            "ts":    _ts(),
            "frame": msg.header.frame_id,
            "goal_m": {
                "x": round(msg.pose.position.x, 4),
                "y": round(msg.pose.position.y, 4),
            },
        })

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5002 — Nav2 status
    # ══════════════════════════════════════════════════════════════════════════

    _NAV_STATUS = {0:"UNKNOWN",1:"ACCEPTED",2:"EXECUTING",3:"CANCELING",
                   4:"SUCCEEDED",5:"CANCELED",6:"ABORTED"}

    def _navstatus_cb(self, msg: GoalStatusArray):
        statuses = [
            {"status": self._NAV_STATUS.get(s.status, str(s.status)),
             "code":   int(s.status)}
            for s in msg.status_list
        ]
        if statuses:
            self._udp.send(self._base, PORT_ODOM, {
                "type":   "nav_status",
                "ts":     _ts(),
                "goals":  statuses,
                "active": statuses[-1]["status"] if statuses else "NONE",
            })

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5002 — LiDAR summary
    # ══════════════════════════════════════════════════════════════════════════

    def _scan_cb(self, msg: LaserScan):
        ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if not ranges:
            return
        self._udp.send(self._base, PORT_ODOM, {
            "type":        "lidar",
            "ts":          _ts(),
            "min_dist_m":  round(min(ranges), 3),
            "max_dist_m":  round(max(ranges), 3),
            "mean_dist_m": round(sum(ranges)/len(ranges), 3),
            "num_points":  len(ranges),
            "front_m":     round(ranges[len(ranges)//2], 3),
        })

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5002 — System resource stats
    # ══════════════════════════════════════════════════════════════════════════

    def _sysres_cb(self):
        if self._psutil is None:
            return
        vm = self._psutil.virtual_memory()
        self._udp.send(self._base, PORT_ODOM, {
            "type":            "system_resources",
            "ts":              _ts(),
            "cpu_pct":         round(self._psutil.cpu_percent(), 1),
            "memory_pct":      round(vm.percent, 1),
            "memory_used_mb":  round(vm.used / 1e6, 1),
            "memory_total_mb": round(vm.total / 1e6, 1),
        })

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5004 — Lane detection state
    # ══════════════════════════════════════════════════════════════════════════

    _lane = {"error_px": 0.0, "visible": False, "both_visible": False}

    def _lane_err_cb(self, msg: Float32):
        self._lane["error_px"] = round(msg.data, 2)
        self._send_lane()

    def _lane_vis_cb(self, msg: Bool):
        self._lane["visible"] = msg.data
        self._send_lane()

    def _lane_both_cb(self, msg: Bool):
        self._lane["both_visible"] = msg.data
        self._send_lane()

    def _send_lane(self):
        self._udp.send(self._base, PORT_LANE, {
            "type":         "lane_detection",
            "ts":           _ts(),
            "error_px":     self._lane["error_px"],
            "visible":      self._lane["visible"],
            "both_visible": self._lane["both_visible"],
            "drift":        "LEFT"   if self._lane["error_px"] >  10 else
                            "RIGHT"  if self._lane["error_px"] < -10 else "CENTRE",
        })
        self._tx["lane"] += 1

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5005 — Face / turret detection state
    # ══════════════════════════════════════════════════════════════════════════

    _face = {"match": False, "h_err": 0.0, "v_err": 0.0,
             "task_active": False, "task_complete": False}

    def _match_cb(self, msg: Bool):
        self._face["match"] = msg.data;  self._send_face()

    def _herr_cb(self, msg: Float32):
        self._face["h_err"] = round(msg.data, 2);  self._send_face()

    def _verr_cb(self, msg: Float32):
        self._face["v_err"] = round(msg.data, 2);  self._send_face()

    def _task_start_cb(self, msg: Bool):
        if msg.data:
            self._face["task_active"]   = True
            self._face["task_complete"] = False
            self._send_face()

    def _task_done_cb(self, msg: Bool):
        self._face["task_active"]   = False
        self._face["task_complete"] = msg.data
        self._send_face()

    def _mission_done_cb(self, msg: Bool):
        self._udp.send(self._base, PORT_FACE, {
            "type":           "mission_done",
            "ts":             _ts(),
            "face_task_done": msg.data,
        })

    def _send_face(self):
        self._udp.send(self._base, PORT_FACE, {
            "type":          "face_detection",
            "ts":            _ts(),
            "task_active":   self._face["task_active"],
            "match_found":   self._face["match"],
            "h_error_px":    self._face["h_err"],
            "v_error_px":    self._face["v_err"],
            "task_complete": self._face["task_complete"],
        })
        self._tx["face"] += 1

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5006 — Waypoint / mission events
    # ══════════════════════════════════════════════════════════════════════════

    def _wp_reached_cb(self, msg: String):
        try:    data = json.loads(msg.data)
        except: data = {"raw": msg.data}
        data["type"] = "waypoint_reached"
        data["ts"]   = _ts()
        self._udp.send(self._base, PORT_MISSION, data)
        self._tx["mission"] += 1

    def _wp_status_cb(self, msg: String):
        try:    data = json.loads(msg.data)
        except: data = {"raw": msg.data}
        data["type"] = "waypoint_status"
        data["ts"]   = _ts()
        self._udp.send(self._base, PORT_MISSION, data)

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5007 — System health / alerts
    # ══════════════════════════════════════════════════════════════════════════

    def _sysstat_cb(self, msg: String):
        try:    data = json.loads(msg.data)
        except: data = {"raw": msg.data}
        data["type"] = "system_status"
        data["ts"]   = _ts()
        self._udp.send(self._base, PORT_ALERTS, data)

    def _alert_cb(self, msg: String):
        try:    data = json.loads(msg.data)
        except: data = {"raw": msg.data}
        if data.get("alert_count", 0) > 0 or "raw" in data:
            data["type"] = "system_alert"
            data["ts"]   = _ts()
            self._udp.send(self._base, PORT_ALERTS, data)
            self._tx["alert"] += 1

    # ══════════════════════════════════════════════════════════════════════════
    # Image conversion — cv_bridge replacement (turret only now)
    # ══════════════════════════════════════════════════════════════════════════

    @staticmethod
    def _ros_img_to_bgr(ros_img: Image):
        """
        Convert a sensor_msgs/Image to a BGR numpy array without cv_bridge.
        Handles the encodings typically used by ROS 2 cameras.
        Raises ValueError on unsupported encoding.
        """
        enc = ros_img.encoding.lower()
        raw = np.frombuffer(ros_img.data, dtype=np.uint8)

        if enc in ("bgr8", "rgb8"):
            frame = raw.reshape((ros_img.height, ros_img.width, 3))
            if enc == "rgb8":
                frame = frame[:, :, ::-1].copy()   # RGB → BGR
        elif enc in ("bgra8", "rgba8"):
            frame = raw.reshape((ros_img.height, ros_img.width, 4))
            if enc == "rgba8":
                frame = frame[:, :, ::-1].copy()
            frame = frame[:, :, :3]                 # drop alpha
        elif enc in ("mono8", "8uc1"):
            frame = raw.reshape((ros_img.height, ros_img.width))
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif enc in ("mono16", "16uc1"):
            raw16 = np.frombuffer(ros_img.data, dtype=np.uint16)
            frame = raw16.reshape((ros_img.height, ros_img.width))
            frame = (frame >> 8).astype(np.uint8)
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif enc in ("yuv422", "yuv422_yuy2", "yuyv"):
            yuv = raw.reshape((ros_img.height, ros_img.width, 2))
            frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUY2)
        elif enc in ("bayer_rggb8", "bayer_bggr8", "bayer_gbrg8", "bayer_grbg8"):
            codes = {
                "bayer_rggb8": cv2.COLOR_BayerBG2BGR,
                "bayer_bggr8": cv2.COLOR_BayerRG2BGR,
                "bayer_gbrg8": cv2.COLOR_BayerGR2BGR,
                "bayer_grbg8": cv2.COLOR_BayerGB2BGR,
            }
            frame = raw.reshape((ros_img.height, ros_img.width))
            frame = cv2.cvtColor(frame, codes[enc])
        else:
            raise ValueError(f"Unsupported encoding: '{ros_img.encoding}'")

        return frame

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 8554 — Video frames (lane camera = webcam, turret camera = ROS topic)
    # ══════════════════════════════════════════════════════════════════════════

    def _encode_and_send_frame(self, frame: np.ndarray, stream_label: str, tx_key: str):
        """Shared core: downscale a BGR frame → JPEG → UDP with header."""
        now = _ts()

        # Downscale
        frame = cv2.resize(frame, (VIDEO_WIDTH, VIDEO_HEIGHT),
                           interpolation=cv2.INTER_LINEAR)

        # JPEG encode
        ok, buf = cv2.imencode(".jpg", frame,
                               [cv2.IMWRITE_JPEG_QUALITY, VIDEO_QUALITY])
        if not ok:
            return

        jpeg_bytes = buf.tobytes()

        # Header: 4-byte magic + 1-byte stream_id + 8-byte timestamp + 4-byte size
        stream_id = 0 if stream_label == "lane" else 1
        header = struct.pack(">4sBdI", b"MRCV", stream_id, now, len(jpeg_bytes))
        packet = header + jpeg_bytes

        if len(packet) <= MAX_UDP:
            self._udp.send_bytes(self._base, PORT_VIDEO, packet)
            self._tx[tx_key] += 1
        else:
            self.get_logger().warn(
                f"[{stream_label}] Frame too large for UDP: {len(packet)} bytes. "
                f"Reduce VIDEO_WIDTH/HEIGHT or VIDEO_QUALITY."
            )

    def _lane_webcam_tick(self):
        """Timer callback — pulls the latest webcam frame and sends it."""
        frame, frame_ts = self._webcam.get_latest_frame()
        if frame is None:
            return
        # Skip stale/duplicate reads if the capture thread hasn't produced
        # a new frame since last send.
        if frame_ts <= self._video_lane_frame_t:
            return
        self._video_lane_frame_t = frame_ts
        self._encode_and_send_frame(frame, "lane", "video_lane")

    def _turret_cam_cb(self, msg: Image):
        """Still ROS-topic based."""
        now = _ts()
        if now - self._video_turret_frame_t < 1.0 / self._video_fps:
            return
        self._video_turret_frame_t = now

        try:
            frame = self._ros_img_to_bgr(msg)
        except ValueError as e:
            self.get_logger().warn(f"[turret] {e}")
            return
        except Exception as e:
            self.get_logger().warn(f"[turret] Frame decode error: {e}")
            return

        self._encode_and_send_frame(frame, "turret", "video_turret")

    # ══════════════════════════════════════════════════════════════════════════
    # PORT 5003 — Drive command receiver
    # ══════════════════════════════════════════════════════════════════════════

    def _recv_loop(self):
        self.get_logger().info(f"Drive CMD recv active on :{PORT_CMD_IN}")
        while self._running:
            try:
                data, addr = self._recv_sock.recvfrom(MAX_UDP)
                msg = json.loads(data.decode())

                if msg.get("type") != "cmd_vel":
                    continue

                vx = float(msg.get("linear_ms",   0.0))
                wz = float(msg.get("angular_rads", 0.0))

                # Safety clamp
                vx = max(-1.0, min(1.0, vx))
                wz = max(-2.0, min(2.0, wz))

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

    # ══════════════════════════════════════════════════════════════════════════
    # Stats logger
    # ══════════════════════════════════════════════════════════════════════════

    def _log_stats(self):
        self.get_logger().info(
            f"[10s TX stats] "
            f"IMU={self._tx['imu']}  "
            f"GPS={self._tx['gps']}  "
            f"ODOM={self._tx['odom']}  "
            f"ENC={self._tx['enc']}  "
            f"LANE={self._tx['lane']}  "
            f"FACE={self._tx['face']}  "
            f"MISSION={self._tx['mission']}  "
            f"ALERT={self._tx['alert']}  "
            f"VID_LANE={self._tx['video_lane']}  "
            f"VID_TURRET={self._tx['video_turret']}"
        )
        self._tx = {k: 0 for k in self._tx}

    def destroy_node(self):
        self._running = False
        self._udp.close()
        self._recv_sock.close()
        self._webcam.stop()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Mercury comms bridge — rover side")
    parser.add_argument("--base-ip", default=BASE_IP)
    parser.add_argument("--webcam-device", default=WEBCAM_DEVICE,
                        help="USB webcam device for the lane camera feed "
                             "(default: /dev/video0)")
    args = parser.parse_args()

    rclpy.init()
    node = CommsRoverNode(base_ip=args.base_ip, webcam_device=args.webcam_device)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()