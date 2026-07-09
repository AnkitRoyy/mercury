#!/usr/bin/env python3
"""
teleop_base.py — Mercury Base Station Teleop Sender
=====================================================
Runs on:  Base station  (192.168.88.2)
Sends to: Rover         (192.168.88.3:5003)

Captures WASD keyboard input via an OpenCV window and streams
cmd_vel drive commands to the rover over UDP at a fixed rate.

Controls (focus the video/control window):
  W/S     forward / back
  A/D     turn left / right
  Space   full stop
  E       emergency stop (holds zero for 1s)
  Q/ESC   quit
"""

import json
import socket
import time

import cv2
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────
ROVER_IP     = "192.168.88.3"
PORT_CMD_OUT = 5003

CMD_HZ    = 10     # rate at which commands are sent to the rover
POLL_FPS  = 30     # cv2.waitKey poll rate (also the key-read rate)
VX_STEP   = 0.05
WZ_STEP   = 0.15
VX_MAX    = 0.5
WZ_MAX    = 1.5

WIN_NAME  = "Teleop Control (focus this window)"


def send_cmd(sock: socket.socket, vx: float, wz: float):
    payload = json.dumps({
        "type":         "cmd_vel",
        "ts":           time.time(),
        "linear_ms":    round(float(vx), 4),
        "angular_rads": round(float(wz), 4),
    }, separators=(',', ':')).encode()
    sock.sendto(payload, (ROVER_IP, PORT_CMD_OUT))


def main():
    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_NAME, 400, 200)

    blank = np.zeros((200, 400, 3), dtype=np.uint8)
    cv2.putText(blank, "W/S fwd-back  A/D turn", (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.putText(blank, "SPACE stop  E e-stop  Q quit", (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    vx, wz       = 0.0, 0.0
    e_stop_until = 0.0
    last_send    = 0.0
    running      = True

    print(f"Teleop sender started. Sending cmd_vel → {ROVER_IP}:{PORT_CMD_OUT}")
    print("Click the control window, then use WASD to drive.\n")

    while running:
        disp = blank.copy()
        cv2.putText(disp, f"vx={vx:+.2f} m/s  wz={wz:+.2f} rad/s", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imshow(WIN_NAME, disp)

        key = cv2.waitKey(int(1000 / POLL_FPS)) & 0xFF
        now = time.time()

        if key != 255:  # a key was pressed
            k = chr(key).lower() if 32 <= key < 127 else ''

            if k == 'q' or key == 27:   # 'q' or ESC
                vx, wz = 0.0, 0.0
                send_cmd(cmd_sock, 0.0, 0.0)
                print("\nStopped. Bye.")
                running = False
                break

            elif k == 'e':
                vx, wz       = 0.0, 0.0
                e_stop_until = now + 1.0   # hold zero for 1s

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
        if now < e_stop_until:
            vx, wz = 0.0, 0.0

        # Send at CMD_HZ regardless of keypress timing
        if now - last_send >= 1.0 / CMD_HZ:
            send_cmd(cmd_sock, vx, wz)
            last_send = now

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
