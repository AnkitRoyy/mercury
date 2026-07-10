#!/usr/bin/env python3
"""
comms_base.py — Mercury Base Station Dashboard
===============================================
Runs on:  Base station  (192.168.88.2)
Receives: Rover         (192.168.88.3)

Ports listened:
  5000  UDP  IMU
  5001  UDP  GPS
  5002  UDP  Odom / Encoders / Nav / System
  5003  UDP  → send drive commands to rover
  5004  UDP  Lane detection
  5005  UDP  Face detection
  5006  UDP  Mission / waypoints
  5007  UDP  System alerts
  8554  UDP  Video frames (lane + turret)

Usage:
  python3 comms_base.py

Controls (always active — focus the video window, no flag needed):
  W/S       forward / back
  A/D       turn left / right
  Space     full stop
  E         emergency stop (sends 0 continuously for 1s)
  Q/ESC     quit

NOTE ON CONTROLS:
  Keyboard capture happens through the OpenCV video window (cv2.waitKey),
  not the raw terminal. This means WASD works as soon as either the
  "Lane Camera" or "Turret Camera" window has OS focus — no special
  terminal mode required. Click the video window, then drive.
  Both video windows open automatically on startup, even before the
  rover starts streaming (they show a "waiting for feed" placeholder).
"""

import json
import math
import os
import socket
import struct
import threading
import time
from datetime import datetime

import cv2
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────

ROVER_IP     = "192.168.88.3"
PORT_IMU     = 5000
PORT_GPS     = 5001
PORT_ODOM    = 5002
PORT_CMD_OUT = 5003
PORT_LANE    = 5004
PORT_FACE    = 5005
PORT_MISSION = 5006
PORT_ALERTS  = 5007
PORT_VIDEO   = 8554

MAX_UDP      = 65507
DISPLAY_HZ   = 4      # terminal refresh rate
CMD_HZ       = 10     # drive command send rate
VIDEO_FPS    = 30      # cv2.waitKey poll rate (also the key-read rate)
VX_STEP      = 0.05
WZ_STEP      = 0.15
VX_MAX       = 0.5
WZ_MAX       = 1.5
STALE_SEC    = 3.0    # mark data stale after this many seconds

WIN_LANE     = "Lane Camera"
WIN_TURRET   = "Turret Camera"

# ══════════════════════════════════════════════════════════════════════════════
# Shared state
# ══════════════════════════════════════════════════════════════════════════════

_lock = threading.Lock()

tele = {
    # IMU
    "imu_roll":    None, "imu_pitch": None, "imu_yaw": None,
    "imu_ax": None, "imu_ay": None, "imu_az": None,
    "imu_wx": None, "imu_wy": None, "imu_wz": None,
    "imu_ts": 0.0,

    # GPS
    "gps_lat": None, "gps_lon": None, "gps_alt": None,
    "gps_fix": "—", "gps_ts":  0.0,

    # Odometry
    "odom_x": None, "odom_y": None, "odom_yaw": None,
    "odom_vx": None, "odom_wz": None, "odom_ts": 0.0,

    # Encoders
    "enc_names": [], "enc_pos": [], "enc_vel": [], "enc_ts": 0.0,

    # cmd_vel being sent on rover
    "rover_vx": None, "rover_wz": None, "rover_cmd_ts": 0.0,

    # LiDAR
    "lidar_min": None, "lidar_max": None,
    "lidar_mean": None, "lidar_front": None, "lidar_ts": 0.0,

    # Navigation
    "nav_goal_x": None, "nav_goal_y": None, "nav_status": "—", "nav_ts": 0.0,

    # System resources
    "cpu_pct": None, "mem_pct": None, "mem_used_mb": None, "sys_ts": 0.0,

    # Lane detection
    "lane_err": None, "lane_visible": False,
    "lane_both": False, "lane_drift": "—", "lane_ts": 0.0,

    # Face detection
    "face_active": False, "face_match": False,
    "face_h_err": None,  "face_v_err": None,
    "face_complete": False, "face_ts": 0.0,

    # Mission
    "wp_name": "—", "wp_idx": "—", "wp_dist": None,
    "wp_all_done": False, "mission_ts": 0.0,

    # Alerts
    "alert_count": 0, "alerts": [], "alert_ts": 0.0,
    "sys_healthy": None, "sys_missing": [],
}

drive = {"vx": 0.0, "wz": 0.0}
video = {"lane": None, "turret": None,   # latest decoded frames (numpy BGR)
         "lane_ts": 0.0, "turret_ts": 0.0}

rx_counts = {k: 0 for k in
    ["imu","gps","odom","enc","lane","face","mission","alert","video"]}

_running = True

# ══════════════════════════════════════════════════════════════════════════════
# UDP helpers
# ══════════════════════════════════════════════════════════════════════════════

def _send_cmd(sock: socket.socket, vx: float, wz: float):
    payload = json.dumps({
        "type":         "cmd_vel",
        "ts":           time.time(),
        "linear_ms":    round(float(vx), 4),
        "angular_rads": round(float(wz), 4),
    }, separators=(',', ':')).encode()
    sock.sendto(payload, (ROVER_IP, PORT_CMD_OUT))


def _listen(port: int, handler, label: str):
    """Generic UDP listener thread."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(1.0)
    while _running:
        try:
            data, _ = sock.recvfrom(MAX_UDP)
            handler(data)
        except socket.timeout:
            continue
        except Exception:
            pass  # keep thread alive


# ══════════════════════════════════════════════════════════════════════════════
# Packet handlers
# ══════════════════════════════════════════════════════════════════════════════

def _h_imu(data: bytes):
    try:
        m = json.loads(data)
        with _lock:
            rpy = m.get("rpy_deg", {})
            tele["imu_roll"]  = rpy.get("roll")
            tele["imu_pitch"] = rpy.get("pitch")
            tele["imu_yaw"]   = rpy.get("yaw")
            av = m.get("angular_velocity_rads", {})
            tele["imu_wx"] = av.get("x")
            tele["imu_wy"] = av.get("y")
            tele["imu_wz"] = av.get("z")
            la = m.get("linear_accel_ms2", {})
            tele["imu_ax"] = la.get("x")
            tele["imu_ay"] = la.get("y")
            tele["imu_az"] = la.get("z")
            tele["imu_ts"] = time.time()
        rx_counts["imu"] += 1
    except Exception: pass


def _h_gps(data: bytes):
    try:
        m = json.loads(data)
        with _lock:
            tele["gps_lat"] = m.get("latitude")
            tele["gps_lon"] = m.get("longitude")
            tele["gps_alt"] = m.get("altitude_m")
            tele["gps_fix"] = m.get("fix_type", "—")
            tele["gps_ts"]  = time.time()
        rx_counts["gps"] += 1
    except Exception: pass


def _h_odom(data: bytes):
    try:
        m = json.loads(data)
        t = m.get("type")
        with _lock:
            if t == "odometry":
                pos = m.get("position_m", {})
                vel = m.get("velocity", {})
                tele["odom_x"]   = pos.get("x")
                tele["odom_y"]   = pos.get("y")
                tele["odom_yaw"] = m.get("yaw_deg")
                tele["odom_vx"]  = vel.get("linear_ms")
                tele["odom_wz"]  = vel.get("angular_rads")
                tele["odom_ts"]  = time.time()
                rx_counts["odom"] += 1

            elif t == "encoders":
                tele["enc_names"] = m.get("joints", [])
                tele["enc_pos"]   = m.get("position_rad", [])
                tele["enc_vel"]   = m.get("velocity_rads", [])
                tele["enc_ts"]    = time.time()
                rx_counts["enc"] += 1

            elif t == "cmd_vel":
                tele["rover_vx"]     = m.get("linear_ms")
                tele["rover_wz"]     = m.get("angular_rads")
                tele["rover_cmd_ts"] = time.time()

            elif t == "lidar":
                tele["lidar_min"]   = m.get("min_dist_m")
                tele["lidar_max"]   = m.get("max_dist_m")
                tele["lidar_mean"]  = m.get("mean_dist_m")
                tele["lidar_front"] = m.get("front_m")
                tele["lidar_ts"]    = time.time()

            elif t == "nav_goal":
                g = m.get("goal_m", {})
                tele["nav_goal_x"] = g.get("x")
                tele["nav_goal_y"] = g.get("y")
                tele["nav_ts"]     = time.time()

            elif t == "nav_status":
                tele["nav_status"] = m.get("active", "—")
                tele["nav_ts"]     = time.time()

            elif t == "system_resources":
                tele["cpu_pct"]     = m.get("cpu_pct")
                tele["mem_pct"]     = m.get("memory_pct")
                tele["mem_used_mb"] = m.get("memory_used_mb")
                tele["sys_ts"]      = time.time()

    except Exception: pass


def _h_lane(data: bytes):
    try:
        m = json.loads(data)
        with _lock:
            tele["lane_err"]     = m.get("error_px")
            tele["lane_visible"] = m.get("visible", False)
            tele["lane_both"]    = m.get("both_visible", False)
            tele["lane_drift"]   = m.get("drift", "—")
            tele["lane_ts"]      = time.time()
        rx_counts["lane"] += 1
    except Exception: pass


def _h_face(data: bytes):
    try:
        m = json.loads(data)
        with _lock:
            t = m.get("type")
            if t == "face_detection":
                tele["face_active"]   = m.get("task_active", False)
                tele["face_match"]    = m.get("match_found", False)
                tele["face_h_err"]    = m.get("h_error_px")
                tele["face_v_err"]    = m.get("v_error_px")
                tele["face_complete"] = m.get("task_complete", False)
                tele["face_ts"]       = time.time()
                rx_counts["face"] += 1
    except Exception: pass


def _h_mission(data: bytes):
    try:
        m = json.loads(data)
        with _lock:
            t = m.get("type")
            if t == "waypoint_reached":
                wp = m.get("waypoint", {})
                tele["wp_name"]  = wp.get("name", "—")
                tele["wp_idx"]   = wp.get("index", "—")
                tele["mission_ts"] = time.time()
                rx_counts["mission"] += 1
            elif t == "waypoint_status":
                tele["wp_all_done"] = m.get("all_completed", False)
                tele["mission_ts"]  = time.time()
    except Exception: pass


def _h_alerts(data: bytes):
    try:
        m = json.loads(data)
        with _lock:
            t = m.get("type")
            if t == "system_alert":
                tele["alert_count"] = m.get("alert_count", 0)
                tele["alerts"]      = [
                    a.get("message", "") for a in m.get("alerts", [])[:5]
                ]
                tele["alert_ts"]    = time.time()
                rx_counts["alert"] += 1
            elif t == "system_status":
                tele["sys_healthy"] = m.get("all_ok")
                tele["sys_missing"] = m.get("missing", [])
    except Exception: pass


HEADER_FMT  = ">4sBdI"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

def _h_video(data: bytes):
    try:
        if len(data) < HEADER_SIZE:
            return
        magic, stream_id, ts, size = struct.unpack_from(HEADER_FMT, data)
        if magic != b"MRCV":
            return
        jpeg = data[HEADER_SIZE:HEADER_SIZE + size]
        arr  = np.frombuffer(jpeg, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        with _lock:
            if stream_id == 0:
                video["lane"]    = frame
                video["lane_ts"] = time.time()
            else:
                video["turret"]    = frame
                video["turret_ts"] = time.time()
        rx_counts["video"] += 1
    except Exception: pass


# ══════════════════════════════════════════════════════════════════════════════
# Terminal display  (telemetry only — no keyboard reading here anymore)
# ══════════════════════════════════════════════════════════════════════════════

def _stale(ts: float) -> str:
    return " \033[33m[STALE]\033[0m" if ts > 0 and (time.time() - ts) > STALE_SEC else ""

def _ok(val) -> str:
    return "\033[32m✓\033[0m" if val else "\033[31m✗\033[0m"

def _f(val, fmt=".3f", unit="") -> str:
    if val is None: return "\033[90m—\033[0m"
    return f"{val:{fmt}}{unit}"

def _color_nav(status: str) -> str:
    colors = {
        "EXECUTING":  "\033[32m",
        "SUCCEEDED":  "\033[36m",
        "ABORTED":    "\033[31m",
        "CANCELED":   "\033[33m",
    }
    c = colors.get(status, "\033[0m")
    return f"{c}{status}\033[0m"

RESET = "\033[0m"
BOLD  = "\033[1m"
DIM   = "\033[2m"
RED   = "\033[31m"
GRN   = "\033[32m"
YLW   = "\033[33m"
CYN   = "\033[36m"
WHT   = "\033[97m"


def _display_loop():
    while _running:
        time.sleep(1.0 / DISPLAY_HZ)
        with _lock:
            t = dict(tele)
            d = dict(drive)

        lines = []
        W = 70
        def _sec(title):
            lines.append(f"\033[34m{'─'*3} {BOLD}{title}{RESET}\033[34m {'─'*(W-5-len(title))}{RESET}")

        # Header
        lines.append(f"\033[2J\033[H")   # clear
        lines.append(f"{BOLD}\033[34m{'═'*W}{RESET}")
        lines.append(
            f"{BOLD}  ⚡ MERCURY BASE STATION{RESET}  "
            f"{DIM}{datetime.now().strftime('%Y-%m-%d  %H:%M:%S')}{RESET}"
        )
        lines.append(f"{BOLD}\033[34m{'═'*W}{RESET}")

        # ── Drive command ─────────────────────────────────────────────────────
        _sec("DRIVE COMMAND  [focus video window → W/S=fwd  A/D=turn  SPACE=stop  E=e-stop  Q=quit]")
        vx_bar = "█" * int(abs(d["vx"]) / VX_MAX * 10)
        wz_bar = "█" * int(abs(d["wz"]) / WZ_MAX * 10)
        vx_col = GRN if d["vx"] > 0 else (RED if d["vx"] < 0 else DIM)
        wz_col = CYN if d["wz"] > 0 else (YLW if d["wz"] < 0 else DIM)
        lines.append(
            f"  Forward  {vx_col}{d['vx']:+.2f} m/s{RESET}  [{vx_col}{vx_bar:<10}{RESET}]   "
            f"Turn  {wz_col}{d['wz']:+.2f} rad/s{RESET}  [{wz_col}{wz_bar:<10}{RESET}]"
        )
        if t["rover_vx"] is not None:
            lines.append(
                f"  Rover /cmd_vel →  vx={t['rover_vx']:+.3f} m/s  "
                f"wz={t['rover_wz']:+.3f} rad/s{_stale(t['rover_cmd_ts'])}"
            )

        # ── IMU ───────────────────────────────────────────────────────────────
        _sec(f"IMU{_stale(t['imu_ts'])}")
        lines.append(
            f"  Roll {_f(t['imu_roll'], '+.1f', '°'):>10}   "
            f"Pitch {_f(t['imu_pitch'], '+.1f', '°'):>10}   "
            f"Yaw {_f(t['imu_yaw'], '+.1f', '°'):>10}"
        )
        lines.append(
            f"  ω  x={_f(t['imu_wx'], '+.3f', ' rad/s')}  "
            f"y={_f(t['imu_wy'], '+.3f', ' rad/s')}  "
            f"z={_f(t['imu_wz'], '+.3f', ' rad/s')}"
        )
        lines.append(
            f"  a  x={_f(t['imu_ax'], '+.3f', ' m/s²')}  "
            f"y={_f(t['imu_ay'], '+.3f', ' m/s²')}  "
            f"z={_f(t['imu_az'], '+.3f', ' m/s²')}"
        )

        # ── GPS ───────────────────────────────────────────────────────────────
        fix_col = GRN if "FIX" in str(t["gps_fix"]) else RED
        _sec(f"GPS  [{fix_col}{t['gps_fix']}{RESET}]{_stale(t['gps_ts'])}")
        lines.append(
            f"  Lat  {_f(t['gps_lat'], '.8f', '°'):>18}   "
            f"Lon  {_f(t['gps_lon'], '.8f', '°'):>18}   "
            f"Alt  {_f(t['gps_alt'], '.1f', ' m')}"
        )

        # ── Odometry ─────────────────────────────────────────────────────────
        _sec(f"ODOMETRY{_stale(t['odom_ts'])}")
        lines.append(
            f"  X  {_f(t['odom_x'], '+.4f', ' m'):>14}   "
            f"Y  {_f(t['odom_y'], '+.4f', ' m'):>14}   "
            f"Yaw  {_f(t['odom_yaw'], '+.2f', '°')}"
        )
        lines.append(
            f"  Vel linear  {_f(t['odom_vx'], '+.4f', ' m/s'):>12}   "
            f"angular  {_f(t['odom_wz'], '+.4f', ' rad/s'):>14}"
        )

        # ── LiDAR ────────────────────────────────────────────────────────────
        _sec(f"LIDAR{_stale(t['lidar_ts'])}")
        lines.append(
            f"  Front {_f(t['lidar_front'], '.3f', ' m'):>8}   "
            f"Min {_f(t['lidar_min'], '.3f', ' m'):>8}   "
            f"Mean {_f(t['lidar_mean'], '.3f', ' m'):>8}   "
            f"Max {_f(t['lidar_max'], '.3f', ' m'):>8}"
        )

        # ── Navigation ───────────────────────────────────────────────────────
        _sec(f"NAVIGATION{_stale(t['nav_ts'])}")
        gx = _f(t['nav_goal_x'], '.3f', ' m')
        gy = _f(t['nav_goal_y'], '.3f', ' m')
        lines.append(
            f"  Goal  x={gx}  y={gy}   "
            f"Status  {_color_nav(t['nav_status'])}"
        )
        lines.append(
            f"  Mission complete  {_ok(t['wp_all_done'])}   "
            f"Last WP  {WHT}{t['wp_name']}{RESET}  (idx {t['wp_idx']})"
            f"{_stale(t['mission_ts'])}"
        )

        # ── Lane detection ────────────────────────────────────────────────────
        _sec(f"LANE DETECTION{_stale(t['lane_ts'])}")
        drift_col = (GRN if t["lane_drift"] == "CENTRE" else
                     YLW if t["lane_drift"] in ("LEFT","RIGHT") else DIM)
        lines.append(
            f"  Visible {_ok(t['lane_visible'])}  "
            f"Both lanes {_ok(t['lane_both'])}  "
            f"Error {_f(t['lane_err'], '+.1f', ' px'):>10}  "
            f"Drift  {drift_col}{t['lane_drift']}{RESET}"
        )

        # ── Face / turret detection ───────────────────────────────────────────
        _sec(f"FACE DETECTION  (turret){_stale(t['face_ts'])}")
        lines.append(
            f"  Task active {_ok(t['face_active'])}  "
            f"Match found {_ok(t['face_match'])}  "
            f"Task done {_ok(t['face_complete'])}"
        )
        if t["face_active"]:
            lines.append(
                f"  H-error {_f(t['face_h_err'], '+.1f', ' px'):>10}   "
                f"V-error {_f(t['face_v_err'], '+.1f', ' px'):>10}"
            )

        # ── System resources ─────────────────────────────────────────────────
        _sec(f"ROVER SYSTEM{_stale(t['sys_ts'])}")
        cpu_col = RED if (t["cpu_pct"] or 0) > 80 else YLW if (t["cpu_pct"] or 0) > 60 else GRN
        mem_col = RED if (t["mem_pct"] or 0) > 80 else YLW if (t["mem_pct"] or 0) > 60 else GRN
        lines.append(
            f"  CPU  {cpu_col}{_f(t['cpu_pct'], '.1f', '%'):>6}{RESET}   "
            f"RAM  {mem_col}{_f(t['mem_pct'], '.1f', '%'):>6}{RESET}  "
            f"({_f(t['mem_used_mb'], '.0f', ' MB')} used)"
        )
        if t["sys_missing"]:
            lines.append(f"  {RED}Missing nodes: {', '.join(t['sys_missing'][:4])}{RESET}")

        # ── Alerts ────────────────────────────────────────────────────────────
        if t["alert_count"] > 0:
            _sec(f"⚠  ALERTS  ({t['alert_count']}){_stale(t['alert_ts'])}")
            for a in t["alerts"][:3]:
                lines.append(f"  {RED}• {a}{RESET}")

        # ── RX stats ─────────────────────────────────────────────────────────
        lines.append(f"{DIM}{'─'*W}{RESET}")
        lines.append(
            f"{DIM}  RX pkts  "
            f"IMU={rx_counts['imu']}  GPS={rx_counts['gps']}  "
            f"ODOM={rx_counts['odom']}  ENC={rx_counts['enc']}  "
            f"LANE={rx_counts['lane']}  FACE={rx_counts['face']}  "
            f"VID={rx_counts['video']}{RESET}"
        )
        lines.append(f"{BOLD}\033[34m{'═'*W}{RESET}")

        print("\n".join(lines), end="", flush=True)
        for k in rx_counts:
            rx_counts[k] = 0


# ══════════════════════════════════════════════════════════════════════════════
# Video + keyboard — single main-thread loop
#
# OpenCV's HighGUI must be driven from one thread (ideally the main thread) or
# windows can silently fail to appear / refresh on some platforms (Wayland,
# some X11/WSL setups, certain camera backends). Both video windows are
# created up-front so they appear immediately at startup, before any frames
# arrive. cv2.waitKey() inside this same loop is also what makes WASD work —
# it reads keys from whichever OpenCV window currently has OS focus, so
# clicking either video window is enough to drive the rover.
# ══════════════════════════════════════════════════════════════════════════════

def _video_and_keyboard_loop(cmd_sock: socket.socket):
    global _running

    cv2.namedWindow(WIN_LANE,   cv2.WINDOW_NORMAL)
    cv2.namedWindow(WIN_TURRET, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_LANE,   480, 360)
    cv2.resizeWindow(WIN_TURRET, 480, 360)
    # Force them to the foreground immediately, even with zero frames yet.
    cv2.moveWindow(WIN_LANE,     60,  60)
    cv2.moveWindow(WIN_TURRET, 580,  60)

    blank_lane   = np.zeros((240, 320, 3), dtype=np.uint8)
    blank_turret = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.putText(blank_lane,   "Waiting for lane feed...",
                (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
    cv2.putText(blank_turret, "Waiting for turret feed...",
                (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
    cv2.imshow(WIN_LANE,   blank_lane)
    cv2.imshow(WIN_TURRET, blank_turret)
    cv2.waitKey(1)   # flush the initial paint so windows show up right away

    vx, wz       = 0.0, 0.0
    e_stop_until = 0.0
    last_send    = 0.0

    while _running:
        with _lock:
            lane_frame   = video["lane"]
            turret_frame = video["turret"]
            lane_ts      = video["lane_ts"]
            turret_ts    = video["turret_ts"]

        now = time.time()

        if lane_frame is not None:
            disp = lane_frame.copy()
            age  = now - lane_ts
            cv2.putText(disp, f"Lane  {age*1000:.0f}ms ago",
                        (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow(WIN_LANE, disp)
        else:
            cv2.imshow(WIN_LANE, blank_lane)

        if turret_frame is not None:
            disp = turret_frame.copy()
            age  = now - turret_ts
            cv2.putText(disp, f"Turret  {age*1000:.0f}ms ago",
                        (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.imshow(WIN_TURRET, disp)
        else:
            cv2.imshow(WIN_TURRET, blank_turret)

        # waitKey both renders the GUI event loop AND reads the next keypress
        # from whichever cv2 window currently has focus.
        key = cv2.waitKey(int(1000 / VIDEO_FPS)) & 0xFF

        if key != 255:  # 255 == no key pressed
            k = chr(key).lower() if 32 <= key < 127 else ''

            if k == 'q' or key == 27:   # 'q' or ESC
                vx, wz = 0.0, 0.0
                _send_cmd(cmd_sock, 0.0, 0.0)
                print("\n\nStopped. Bye.\n")
                _running = False
                break

            elif k == 'e':
                vx, wz       = 0.0, 0.0
                e_stop_until = time.time() + 1.0   # hold zero for 1s

            elif k == ' ':
                vx, wz = 0.0, 0.0

            elif k == 'w':
                vx = round(min(VX_MAX,  vx + VX_STEP), 3)
            elif k == 's':
                vx = round(max(-VX_MAX, vx - VX_STEP), 3)
            elif k == 'a':
                wz = round(min(WZ_MAX,  wz + WZ_STEP), 3)
            elif k == 'd':
                wz = round(max(-WZ_MAX, wz - WZ_STEP), 3)

        # E-stop override
        if time.time() < e_stop_until:
            vx, wz = 0.0, 0.0

        # Send at CMD_HZ regardless of keypress timing
        if now - last_send >= 1.0 / CMD_HZ:
            _send_cmd(cmd_sock, vx, wz)
            with _lock:
                drive["vx"] = vx
                drive["wz"] = wz
            last_send = now

    cv2.destroyAllWindows()
    os._exit(0)


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Start all UDP listener threads
    listeners = [
        (PORT_IMU,     _h_imu,     "IMU"),
        (PORT_GPS,     _h_gps,     "GPS"),
        (PORT_ODOM,    _h_odom,    "ODOM"),
        (PORT_LANE,    _h_lane,    "LANE"),
        (PORT_FACE,    _h_face,    "FACE"),
        (PORT_MISSION, _h_mission, "MISSION"),
        (PORT_ALERTS,  _h_alerts,  "ALERTS"),
        (PORT_VIDEO,   _h_video,   "VIDEO"),
    ]
    for port, handler, label in listeners:
        threading.Thread(
            target=_listen, args=(port, handler, label), daemon=True).start()

    # Terminal telemetry display thread (no input handling — display only)
    threading.Thread(target=_display_loop, daemon=True).start()

    print(f"\nMercury Base Station started. Connecting to rover {ROVER_IP}...")
    print("Opening video windows — click one to enable WASD control.\n")
    time.sleep(0.3)

    # Video + keyboard on the MAIN thread (required for reliable OpenCV GUI)
    _video_and_keyboard_loop(cmd_sock)


if __name__ == "__main__":
    main()
