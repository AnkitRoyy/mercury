#!/usr/bin/env python3
"""
ugv_gcs_bridge.py  —  Rover side, NO ROS 2 dependency
Reads from log files written by ugv_logger.py and forwards telemetry to GCS.
Camera feeds are captured DIRECTLY from USB devices via OpenCV/V4L2 —
no ROS image topics involved.

Mental model:
  ├── File tailer threads (one per log file)
  │   ├── state_log.json     → IMU  → UDP :5000
  │   │                      → GPS  → UDP :5001
  │   │                      → ODOM → UDP :5002
  │   ├── encoder_log.json   → Encoder → UDP :5003
  │   ├── control_log.json   → CMD_VEL → UDP :5004
  │   ├── system_log.json    → SYS_STATUS → UDP :5005
  │   ├── alerts_log.json    → ALERT → TCP :6000 (with ACK)
  │   └── navigation_log.json→ MODE/ESTOP/FACE_TASK → TCP :6000 (with ACK)
  ├── Heartbeat thread       → TCP :6000 at 10 Hz
  └── Camera threads (one per USB device, no ROS)
      ├── Main camera   → MAIN_CAM   (see CONFIG below) → UDP :5600
      └── Turret camera → TURRET_CAM (see CONFIG below) → UDP :5601

All settings live in the CONFIG block below — edit and run, no CLI flags.

Usage:
    pip install msgpack opencv-python numpy
    python3 ugv_gcs_bridge.py
"""
import glob
import json
import os
import socket
import struct
import subprocess
import threading
import time
import logging

import msgpack

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("ugv_bridge")

# ═══════════════════════════════════════════════════════════════════════════
# CONFIG — edit these instead of passing CLI flags
# ═══════════════════════════════════════════════════════════════════════════

GCS_IP  = "192.168.88.2"                       # GCS laptop IP
LOG_DIR = os.path.expanduser("~/robot_logs")   # telemetry log directory

ENABLE_VIDEO       = True   # set False to disable both camera streams
ENABLE_TURRET_CAM  = True   # set False to stream only the main camera

# Stable by-id paths — survive reboots and USB port swaps, unlike /dev/videoN.
# Always use the "-video-index0" symlink (index1 is a metadata-only node on
# most UVC webcams and won't produce readable frames via OpenCV).
# MAIN_CAM   = "/dev/v4l/by-id/usb-EMEET_EMEET_SmartCam_C950_4K_A260131000702152-video-index0"
TURRET_CAM = "/dev/v4l/by-id/usb-Image+_Angetube Live Camera_HU1234567898-video-index0"
MAIN_CAM = glob.glob("/dev/v4l/by-id/*Angetube*video-index0")[0]

CAM_WIDTH    = 640
CAM_HEIGHT   = 480
CAM_FPS      = 20
CAM_FOURCC   = "MJPG"
CAM_QUALITY  = 60   # JPEG quality 0-100

# ═══════════════════════════════════════════════════════════════════════════

# ─── Ports (must match gcs_receiver.py) ──────────────────────────────────────
UDP_IMU_PORT     = 5000
UDP_GPS_PORT     = 5001
UDP_ODOM_PORT    = 5002
UDP_ENCODER_PORT = 5003
UDP_CMDVEL_PORT  = 5004
UDP_SYSSTAT_PORT = 5005
UDP_VIDEO_PORT   = 5600
UDP_TURRET_VIDEO_PORT = 5601
TCP_CMD_PORT     = 6000
TCP_FILE_PORT    = 7000

CMD_RETRANSMIT_INTERVAL = 0.5
CMD_MAX_RETRIES         = 10
CHUNK_SIZE              = 60000   # bytes per UDP video packet
TAIL_INTERVAL           = 0.05    # seconds between file tail polls (50ms)


# ─── UDP sender ───────────────────────────────────────────────────────────────
class UdpSender:
    def __init__(self, gcs_ip: str):
        self._gcs_ip = gcs_ip
        self._sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._seq    = {}

    def send(self, port: int, topic: str, payload: dict):
        seq = self._seq.get(topic, 0) + 1
        self._seq[topic] = seq
        payload["_seq"] = seq
        payload["_t"]   = time.time()
        data = msgpack.packb(payload, use_bin_type=True)
        try:
            self._sock.sendto(data, (self._gcs_ip, port))
        except OSError as e:
            log.warning("UDP send error on %s: %s", topic, e)

    def close(self):
        self._sock.close()


# ─── TCP command sender (with ACK) ───────────────────────────────────────────
class TcpCmdSender:
    def __init__(self, gcs_ip: str, port: int = TCP_CMD_PORT):
        self._gcs_ip       = gcs_ip
        self._port         = port
        self._sock         = None
        self._lock         = threading.Lock()
        self._seq          = 0
        self._last_attempt = 0.0
        self._connect()

    def _connect(self):
        now = time.time()
        if now - self._last_attempt < 3.0:
            return
        self._last_attempt = now
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3)
            s.connect((self._gcs_ip, self._port))
            s.settimeout(None)
            self._sock = s
            log.info("TCP command channel connected to %s:%d", self._gcs_ip, self._port)
        except OSError as e:
            log.warning("TCP not ready, will retry in 3s: %s", e)
            self._sock = None

    def send_reliable(self, msg_type: str, data: dict):
        with self._lock:
            self._seq += 1
            seq = self._seq
            payload = {"type": msg_type, "seq": seq, **data}
            raw   = msgpack.packb(payload, use_bin_type=True)
            frame = struct.pack(">I", len(raw)) + raw

            for attempt in range(CMD_MAX_RETRIES):
                if self._sock is None:
                    self._connect()
                if self._sock is None:
                    log.error("No TCP connection — cannot send %s (attempt %d)", msg_type, attempt + 1)
                    time.sleep(CMD_RETRANSMIT_INTERVAL)
                    continue
                try:
                    self._sock.sendall(frame)
                    self._sock.settimeout(CMD_RETRANSMIT_INTERVAL)
                    ack_raw = self._sock.recv(64)
                    self._sock.settimeout(None)
                    if ack_raw and ack_raw.strip() == f"ACK:{seq}".encode():
                        log.info("ACK received for %s seq=%d", msg_type, seq)
                        return True
                    else:
                        log.warning("Bad ACK for %s seq=%d: %r", msg_type, seq, ack_raw)
                except (socket.timeout, OSError) as e:
                    log.warning("Retransmit %s seq=%d attempt %d: %s", msg_type, seq, attempt + 1, e)
                    self._sock = None

            log.error("FAILED to deliver %s after %d attempts", msg_type, CMD_MAX_RETRIES)
            return False

    def send_heartbeat(self):
        if self._sock is None:
            self._connect()
        if self._sock is None:
            return
        try:
            hb    = msgpack.packb({"type": "HB", "t": time.time()}, use_bin_type=True)
            frame = struct.pack(">I", len(hb)) + hb
            self._sock.sendall(frame)
        except OSError:
            self._sock = None

    def close(self):
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass


# ─── TCP file sender (target photos) ─────────────────────────────────────────
class TcpFileSender:
    def __init__(self, gcs_ip: str, port: int = TCP_FILE_PORT):
        self._gcs_ip = gcs_ip
        self._port   = port

    def send_photo(self, data: bytes, filename: str = "target.jpg"):
        try:
            with socket.create_connection((self._gcs_ip, self._port), timeout=5) as s:
                name_enc = filename.encode()
                header   = struct.pack(">I", len(name_enc)) + name_enc + \
                           struct.pack(">Q", len(data))
                s.sendall(header + data)
                log.info("Photo sent: %s (%d bytes)", filename, len(data))
        except OSError as e:
            log.error("Photo transfer failed: %s", e)


# ─── Video sender (JPEG chunks over UDP) ─────────────────────────────────────
class VideoSender:
    def __init__(self, gcs_ip: str, port: int = UDP_VIDEO_PORT, quality: int = 60):
        self._addr     = (gcs_ip, port)
        self._quality  = quality
        self._sock     = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        self._frame_id = 0

    def send_frame(self, jpeg_bytes: bytes):
        self._frame_id = (self._frame_id + 1) & 0xFFFFFFFF
        chunks = [jpeg_bytes[i:i+CHUNK_SIZE] for i in range(0, len(jpeg_bytes), CHUNK_SIZE)]
        total  = len(chunks)
        for idx, chunk in enumerate(chunks):
            header = struct.pack(">IHH", self._frame_id, idx, total)
            try:
                self._sock.sendto(header + chunk, self._addr)
            except OSError:
                pass

    def close(self):
        self._sock.close()


# ─── USB camera streamer (direct V4L2 capture, no ROS) ───────────────────────
class UsbCameraStreamer:
    """
    Opens a USB webcam directly via OpenCV/V4L2, continuously reads frames,
    JPEG-encodes them, and pushes them straight into a VideoSender.
    No file intermediary, no ROS image topic — this replaces the old
    rover_logger.py -> video_frames/ -> file-watcher path entirely.
    """

    def __init__(self, device: str, sender: "VideoSender", label: str,
                 width: int = 640, height: int = 480, fps: int = 20,
                 fourcc: str = "MJPG", quality: int = 60,
                 power_line_freq: int = 1):
        self._device   = device
        self._sender   = sender
        self._label    = label
        self._width    = width
        self._height   = height
        self._fps      = fps
        self._fourcc   = fourcc
        self._quality  = quality
        self._power_line_freq = power_line_freq  # 1 = 50Hz, 2 = 60Hz, 0 = off

        self._cap      = None
        self._running  = False
        self._thread   = None
        self._frame_interval = 1.0 / fps if fps > 0 else 0.0

    def _apply_v4l2_controls(self):
        if self._power_line_freq is None:
            return
        try:
            subprocess.run(
                ["v4l2-ctl", "-d", self._device,
                 "-c", f"power_line_frequency={self._power_line_freq}"],
                check=False, capture_output=True, timeout=2.0,
            )
        except Exception as e:
            log.warning("[%s] v4l2-ctl control failed: %s", self._label, e)

    def start(self):
        import cv2
        self._apply_v4l2_controls()

        # OpenCV's V4L2 backend can fail to open a device "by name" when the
        # path contains characters like spaces (common in by-id symlinks
        # built from a camera's USB product-name string, e.g. cameras whose
        # descriptor is "Angetube Live Camera"). Resolving the symlink to
        # its real /dev/videoN target sidesteps that backend limitation
        # while still letting us select the camera by its stable by-id path.
        resolved = os.path.realpath(self._device)
        if resolved != self._device:
            log.info("[%s] Resolved %s → %s", self._label, self._device, resolved)

        self._cap = cv2.VideoCapture(resolved, cv2.CAP_V4L2)
        if self._fourcc:
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self._fourcc))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv2.CAP_PROP_FPS,          self._fps)

        if not self._cap.isOpened():
            log.error("[%s] Failed to open USB camera %s (resolved: %s) — check the "
                       "device path (try `v4l2-ctl --list-devices` or `ls /dev/video*`)",
                       self._label, self._device, resolved)
            return False

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("[%s] Streaming %s (%dx%d @ %dfps, %s) → UDP :%d",
                  self._label, self._device, self._width, self._height,
                  self._fps, self._fourcc, self._sender._addr[1])
        return True

    def _loop(self):
        import cv2
        fail_count = 0
        last_sent = 0.0
        while self._running:
            ok, frame = self._cap.read()
            if not ok or frame is None:
                fail_count += 1
                if fail_count % 30 == 1:
                    log.warning("[%s] Frame read failed (%d total) — retrying",
                                self._label, fail_count)
                time.sleep(0.05)
                continue
            fail_count = 0

            now = time.time()
            if now - last_sent < self._frame_interval:
                continue
            last_sent = now

            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
            if not ok:
                continue
            self._sender.send_frame(buf.tobytes())

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._cap is not None:
            self._cap.release()


# ─── File tailer ─────────────────────────────────────────────────────────────
class FileTailer:
    """
    Tails a JSONL file and yields new lines as they are appended.
    Seeks to the end on startup so only new data is forwarded.
    """
    def __init__(self, path: str):
        self._path = path
        self._fh   = None

    def _open(self):
        try:
            self._fh = open(self._path, "r")
            self._fh.seek(0, 2)   # seek to end — don't replay old data
            log.info("Tailing %s", self._path)
            return True
        except OSError:
            return False

    def readlines(self):
        """Call repeatedly. Returns list of new JSON strings since last call."""
        if self._fh is None:
            if not self._open():
                return []
        lines = []
        while True:
            line = self._fh.readline()
            if not line:
                break
            line = line.strip()
            if line:
                lines.append(line)
        return lines


# ─── Log dispatch — maps log entries to network sends ────────────────────────
def _dispatch_state(entry: dict, udp: UdpSender):
    """state_log.json contains imu / gps / odom entries."""
    if "imu" in entry:
        d = entry["imu"]
        udp.send(UDP_IMU_PORT, "imu", {
            "ax": d["linear_acceleration"][0],
            "ay": d["linear_acceleration"][1],
            "az": d["linear_acceleration"][2],
            "wx": d["angular_velocity"][0],
            "wy": d["angular_velocity"][1],
            "wz": d["angular_velocity"][2],
            "ox": d["orientation"][0],
            "oy": d["orientation"][1],
            "oz": d["orientation"][2],
            "ow": d["orientation"][3],
        })
    elif "gps" in entry:
        d = entry["gps"]
        udp.send(UDP_GPS_PORT, "gps", {
            "lat": d["lat"],
            "lon": d["lon"],
            "alt": d["alt"],
        })
    elif "odom" in entry:
        d = entry["odom"]
        udp.send(UDP_ODOM_PORT, "odom", {
            "x":  d["pos"][0],
            "y":  d["pos"][1],
            "vx": d["vel"][0],
            "wz": d["vel"][1],
        })


def _dispatch_encoder(entry: dict, udp: UdpSender):
    if "encoder" in entry:
        d = entry["encoder"]
        udp.send(UDP_ENCODER_PORT, "encoder", {
            "names":    d["names"],
            "position": d["position"],
            "velocity": d["velocity"],
        })


def _dispatch_control(entry: dict, udp: UdpSender):
    if "cmd_vel" in entry:
        d = entry["cmd_vel"]
        udp.send(UDP_CMDVEL_PORT, "cmd_vel", {
            "linear":  d["linear"],
            "angular": d["angular"],
        })


def _dispatch_system(entry: dict, udp: UdpSender):
    if "system_status" in entry:
        udp.send(UDP_SYSSTAT_PORT, "sys_status", {"status": entry["system_status"]})


def _dispatch_alert(entry: dict, tcp: TcpCmdSender):
    if "alert" in entry:
        threading.Thread(
            target=tcp.send_reliable,
            args=("ALERT", {"msg": entry["alert"]}),
            daemon=True,
        ).start()


_last_nav_mode = {"v": None}   # track last sent mode to avoid repeats

def _dispatch_nav(entry: dict, tcp: TcpCmdSender):
    """navigation_log.json — forward goal only, skip repeated mode spam."""
    if "goal" in entry:
        new_mode = f"NAVIGATING:{entry['goal']}"
        if _last_nav_mode["v"] == new_mode:
            return   # already sent this
        _last_nav_mode["v"] = new_mode
        threading.Thread(
            target=tcp.send_reliable,
            args=("MODE", {"mode": "NAVIGATING", "goal": entry["goal"]}),
            daemon=True,
        ).start()


# ─── Tailer thread factory ────────────────────────────────────────────────────
def _start_tailer(path: str, dispatch_fn, stop_event: threading.Event):
    def _run():
        tailer = FileTailer(path)
        while not stop_event.is_set():
            for line in tailer.readlines():
                try:
                    entry = json.loads(line)
                    dispatch_fn(entry)
                except json.JSONDecodeError:
                    pass
            time.sleep(TAIL_INTERVAL)
    t = threading.Thread(target=_run, daemon=True)
    t.start()
    return t


# ─── Heartbeat thread ─────────────────────────────────────────────────────────
def _start_heartbeat(tcp: TcpCmdSender, stop_event: threading.Event):
    def _run():
        while not stop_event.is_set():
            tcp.send_heartbeat()
            time.sleep(0.1)   # 10 Hz
    threading.Thread(target=_run, daemon=True).start()


# ─── Entry point ──────────────────────────────────────────────────────────────
def main():
    log_dir = LOG_DIR
    log.info("Reading telemetry logs from %s", log_dir)
    log.info("Sending to GCS at %s", GCS_IP)

    udp  = UdpSender(GCS_IP)
    tcp  = TcpCmdSender(GCS_IP)
    file = TcpFileSender(GCS_IP)

    stop = threading.Event()

    # Start heartbeat immediately so TCP connects
    _start_heartbeat(tcp, stop)

    # Give TCP 3s to connect before processing log files
    log.info("Waiting 3s for TCP to connect...")
    time.sleep(3)

    # Start one tailer thread per telemetry log file (unrelated to video)
    tailers = [
        (os.path.join(log_dir, "state_log.json"),
            lambda e: _dispatch_state(e, udp)),
        (os.path.join(log_dir, "encoder_log.json"),
            lambda e: _dispatch_encoder(e, udp)),
        (os.path.join(log_dir, "control_log.json"),
            lambda e: _dispatch_control(e, udp)),
        (os.path.join(log_dir, "system_log.json"),
            lambda e: _dispatch_system(e, udp)),
        (os.path.join(log_dir, "alerts_log.json"),
            lambda e: _dispatch_alert(e, tcp)),
        (os.path.join(log_dir, "navigation_log.json"),
            lambda e: _dispatch_nav(e, tcp)),
    ]

    for path, fn in tailers:
        _start_tailer(path, fn, stop)

    # ── USB camera streams — direct capture, no ROS, no file intermediary ────
    main_streamer = None
    turret_streamer = None
    if ENABLE_VIDEO:
        try:
            import cv2  # noqa: F401 — just checking it's importable
        except ImportError:
            log.warning("opencv-python not installed — video disabled")
        else:
            vsend_main = VideoSender(GCS_IP, port=UDP_VIDEO_PORT)
            main_streamer = UsbCameraStreamer(
                MAIN_CAM, vsend_main, "main",
                width=CAM_WIDTH, height=CAM_HEIGHT,
                fps=CAM_FPS, fourcc=CAM_FOURCC, quality=CAM_QUALITY,
            )
            if not main_streamer.start():
                main_streamer = None

            if ENABLE_TURRET_CAM:
                vsend_turret = VideoSender(GCS_IP, port=UDP_TURRET_VIDEO_PORT)
                turret_streamer = UsbCameraStreamer(
                    TURRET_CAM, vsend_turret, "turret",
                    width=CAM_WIDTH, height=CAM_HEIGHT,
                    fps=CAM_FPS, fourcc=CAM_FOURCC, quality=CAM_QUALITY,
                )
                if not turret_streamer.start():
                    turret_streamer = None

    log.info("Bridge running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log.info("Shutting down")
        stop.set()
        udp.close()
        tcp.close()
        if main_streamer:
            main_streamer.stop()
        if turret_streamer:
            turret_streamer.stop()


if __name__ == "__main__":
    main()