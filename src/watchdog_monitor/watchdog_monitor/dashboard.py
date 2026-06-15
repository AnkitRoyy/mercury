#!/usr/bin/env python3
"""
dashboard.py
============
Terminal dashboard for monitoring Mercury robot status.

Subscribes to:
  /system_status
  /system_alerts
  /waypoint_reached
  /waypoint_status
"""

import json
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── ANSI color codes ──────────────────────────────────────────────────────────
R = '\033[0m'
B = '\033[1m'
DIM = '\033[2m'

BLACK = '\033[30m'
RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
BLUE = '\033[34m'
CYAN = '\033[36m'
WHITE = '\033[37m'

BG_RED = '\033[41m'
BG_GREEN = '\033[42m'
BG_YELLOW = '\033[43m'
BG_BLUE = '\033[44m'
BG_CYAN = '\033[46m'

# ── Box-drawing chars ─────────────────────────────────────────────────────────
TL, TR, BL, BR = '╔', '╗', '╚', '╝'
H, V = '═', '║'
LM, RM = '╠', '╣'

WIDTH = 72


def box_top(title='', width=WIDTH):
    if title:
        pad = width - len(title) - 4
        left = pad // 2
        right = pad - left
        return f'{TL}{H * left} {B}{title}{R} {H * right}{TR}'
    return f'{TL}{H * (width - 2)}{TR}'


def box_mid(width=WIDTH):
    return f'{LM}{H * (width - 2)}{RM}'


def box_bot(width=WIDTH):
    return f'{BL}{H * (width - 2)}{BR}'


def box_row(text='', width=WIDTH):
    import re
    ansi_escape = re.compile(r'\033\[[0-9;]*m')
    visible = ansi_escape.sub('', text)
    pad = width - 2 - len(visible)
    return f'{V} {text}{" " * max(pad, 0)}{V}'


def fmt_time(ts):
    return datetime.fromtimestamp(ts).strftime('%H:%M:%S') if ts else '—'


def status_badge(ok: bool):
    if ok:
        return f'{B}{BG_GREEN}{BLACK}  OK  {R}'
    return f'{B}{BG_RED}{WHITE}  !!  {R}'


def alert_badge(level: str):
    if level == 'ERROR':
        return f'{B}{BG_RED}{WHITE} ERROR {R}'
    if level == 'WARN':
        return f'{B}{BG_YELLOW}{BLACK}  WARN {R}'
    return f'{B}{BG_BLUE}{WHITE}  INFO {R}'


def wp_badge(reached: bool):
    if reached:
        return f'{B}{GREEN}✔ REACHED{R}'
    return f'{DIM}{YELLOW}◌ pending{R}'


class DashboardNode(Node):

    def __init__(self):
        super().__init__('dashboard')

        self.declare_parameter('refresh_rate', 1.0)

        self._system_status = None
        self._system_alerts = None
        self._waypoint_status = None
        self._recent_events: list[dict] = []

        self.create_subscription(String, '/system_status', self._on_status, 10)
        self.create_subscription(String, '/system_alerts', self._on_alerts, 10)
        self.create_subscription(String, '/waypoint_reached', self._on_wp_event, 10)
        self.create_subscription(String, '/waypoint_status', self._on_wp_status, 10)

        rate = self.get_parameter('refresh_rate').value
        self.create_timer(1.0 / rate, self._draw)

    def _on_status(self, msg):
        try:
            self._system_status = json.loads(msg.data)
        except Exception:
            pass

    def _on_alerts(self, msg):
        try:
            self._system_alerts = json.loads(msg.data)
        except Exception:
            pass

    def _on_wp_event(self, msg):
        try:
            self._recent_events.append(json.loads(msg.data))
            self._recent_events = self._recent_events[-5:]
        except Exception:
            pass

    def _on_wp_status(self, msg):
        try:
            self._waypoint_status = json.loads(msg.data)
        except Exception:
            pass

    def _draw(self):
        lines = []
        now_str = datetime.now().strftime('%Y-%m-%d  %H:%M:%S')

        lines.append(box_top('MERCURY ROBOT  —  MONITORING DASHBOARD'))
        lines.append(box_row(f'{DIM}Updated: {now_str}{R}'))
        lines.append(box_mid())

        # ── System Health ──────────────────────────────────────────────────────
        lines.append(box_row(f'{B}{CYAN}  SYSTEM HEALTH{R}'))
        lines.append(box_row())

        if self._system_status is None:
            lines.append(box_row(f'  {YELLOW}Waiting for /system_status ...{R}'))
        else:
            s = self._system_status
            ok = s.get('all_ok', False)
            running = s.get('total_running', 0)
            expected = s.get('total_expected', 0)
            missing = s.get('missing', [])

            badge = status_badge(ok)
            lines.append(box_row(f'  {badge}  Nodes: {B}{GREEN}{running}{R} / {expected}'))

            if missing:
                lines.append(box_row())
                lines.append(box_row(f'  {RED}{B}Missing nodes:{R}'))
                for n in missing[:5]:
                    lines.append(box_row(f'    {RED}✖ {n}{R}'))
                if len(missing) > 5:
                    lines.append(box_row(f'    {DIM}... and {len(missing)-5} more{R}'))
            else:
                lines.append(box_row(f'  {GREEN}✔ All expected nodes running{R}'))

        lines.append(box_mid())

        # ── Alerts ────────────────────────────────────────────────────────────
        lines.append(box_row(f'{B}{CYAN}  ALERTS{R}'))
        lines.append(box_row())

        if self._system_alerts is None:
            lines.append(box_row(f'  {YELLOW}Waiting for /system_alerts ...{R}'))
        else:
            alerts = self._system_alerts.get('alerts', [])
            if not alerts:
                lines.append(box_row(f'  {GREEN}✔ No active alerts{R}'))
            else:
                for alert in alerts[:3]:
                    badge = alert_badge(alert.get('level', 'WARN'))
                    subj = alert.get('subject', '')
                    msg = alert.get('message', '')[:50]
                    lines.append(box_row(f'  {badge} {subj}'))
                    lines.append(box_row(f'       {DIM}{msg}{R}'))
                if len(alerts) > 3:
                    lines.append(box_row(f'       {DIM}... and {len(alerts)-3} more{R}'))

        lines.append(box_mid())

        # ── Waypoints ─────────────────────────────────────────────────────────
        lines.append(box_row(f'{B}{CYAN}  WAYPOINTS{R}'))
        lines.append(box_row())

        if self._waypoint_status is None:
            lines.append(box_row(f'  {YELLOW}Waiting for /waypoint_status ...{R}'))
        else:
            ws = self._waypoint_status
            total = ws.get('total', 0)
            reached = ws.get('reached_at_least_once', 0)
            rx = ws.get('robot_x', 0.0)
            ry = ws.get('robot_y', 0.0)

            filled = int((reached / total * 20)) if total else 0
            bar = f'{GREEN}{"█" * filled}{DIM}{"░" * (20 - filled)}{R}'
            pct = int(reached / total * 100) if total else 0

            lines.append(box_row(f'  Robot: ({rx:6.2f}, {ry:6.2f})'))
            lines.append(box_row(f'  Progress: [{bar}] {pct}%  ({reached}/{total})'))
            lines.append(box_row())

            for wp in ws.get('waypoints', []):
                badge = wp_badge(wp.get('reach_count', 0) > 0)
                name = wp.get('name', '?')
                x, y = wp.get('x', 0.0), wp.get('y', 0.0)
                count = wp.get('reach_count', 0)
                ts = wp.get('reached_at')
                count_str = f'{DIM}(×{count}){R}' if count > 0 else ''
                time_str = f'{DIM}@{fmt_time(ts)}{R}' if ts else ''
                lines.append(box_row(f'    {badge}  {B}{name}{R}  ({x:.1f}, {y:.1f}) {count_str} {time_str}'))

            if ws.get('all_completed', False):
                lines.append(box_row())
                lines.append(box_row(f'  {B}{BG_GREEN}{BLACK}  ✔ ALL WAYPOINTS COMPLETED!  {R}'))

        lines.append(box_bot())

        os.system('clear')
        print('\n'.join(lines))
        print()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()