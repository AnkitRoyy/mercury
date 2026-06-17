#!/usr/bin/env python3
"""
Localization Diagnostics Node
=============================
Compares EKF output against Gazebo ground truth to verify GPS fusion is working.

Subscribes:
  /odometry/filtered   — EKF fused output (what Nav2 uses)
  /odometry/gps        — navsat_transform output (GPS → local odom)
  /gps_fixed           — raw GPS with covariance
  /diff_drive_controller/odom     — wheel odometry (ground truth in sim)

Prints a live dashboard every 2 seconds showing:
  - GPS lat/lon and distance from datum
  - navsat /odometry/gps position
  - EKF /odometry/filtered position
  - Gazebo ground truth position
  - ERROR between EKF and ground truth (this is what matters!)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# Datum from mercury.sdf spherical_coordinates
DATUM_LAT = 30.0444
DATUM_LON = 31.2357


def haversine_m(lat1, lon1, lat2, lon2):
    """Approximate distance in meters between two lat/lon points."""
    R = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class LocalizationDiagnostics(Node):
    def __init__(self):
        super().__init__('localization_diagnostics')

        self.gps_data = None
        self.odom_gps = None
        self.ekf_odom = None
        self.gz_truth = None

        self.create_subscription(NavSatFix, '/gps_fixed', self._cb_gps, 10)
        self.create_subscription(Odometry, '/odometry/gps', self._cb_odom_gps, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._cb_ekf, 10)
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self._cb_gz, 10)

        self.timer = self.create_timer(2.0, self._print_dashboard)
        self.get_logger().info('localization diagnostics started. dashboard prints every 2s...')

    def _cb_gps(self, msg):
        self.gps_data = msg

    def _cb_odom_gps(self, msg):
        self.odom_gps = msg

    def _cb_ekf(self, msg):
        self.ekf_odom = msg

    def _cb_gz(self, msg):
        self.gz_truth = msg

    def _print_dashboard(self):
        lines = []
        lines.append('\n' + '=' * 65)
        lines.append('       localization diagonstics dashboard  ')
        lines.append('=' * 65)

        # --- GPS Raw ---
        if self.gps_data:
            g = self.gps_data
            dist = haversine_m(DATUM_LAT, DATUM_LON, g.latitude, g.longitude)
            lines.append(f'\ngps raw (/gps_fixed)  frame: {g.header.frame_id}')
            lines.append(f'   lat: {g.latitude:.7f}   lon: {g.longitude:.7f}')
            lines.append(f'   distance from datum: {dist:.2f} m')
            cov_diag = [g.position_covariance[0],
                        g.position_covariance[4],
                        g.position_covariance[8]]
            lines.append(f'   covariance diag: {cov_diag}  type: {g.position_covariance_type}')
            if dist > 500:
                lines.append('    GPS > 500m from datum — noise too high or wrong datum!')
        else:
            lines.append('\n GPS Raw (/gps_fixed):   NO DATA')

        # --- Navsat Odom ---
        if self.odom_gps:
            p = self.odom_gps.pose.pose.position
            lines.append(f'\n  Navsat Odom (/odometry/gps)  frame: {self.odom_gps.header.frame_id}')
            lines.append(f'   x: {p.x:8.3f}   y: {p.y:8.3f}   z: {p.z:8.3f}')
        else:
            lines.append('\n  Navsat Odom (/odometry/gps):   NO DATA')
            lines.append('   → navsat_transform_node is NOT producing output!')

        # --- EKF ---
        if self.ekf_odom:
            p = self.ekf_odom.pose.pose.position
            lines.append(f'\n EKF Output (/odometry/filtered)  frame: {self.ekf_odom.header.frame_id}')
            lines.append(f'   x: {p.x:8.3f}   y: {p.y:8.3f}   z: {p.z:8.3f}')
        else:
            lines.append('\n EKF Output (/odometry/filtered):   NO DATA')

        # --- Gazebo Ground Truth ---
        if self.gz_truth:
            p = self.gz_truth.pose.pose.position
            lines.append(f'\n Wheel Odom Ground Truth (/diff_drive_controller/odom)')
            lines.append(f'   x: {p.x:8.3f}   y: {p.y:8.3f}   z: {p.z:8.3f}')
        else:
            lines.append('\n Wheel Odom Ground Truth:   NO DATA')

        # --- Error comparison ---
        if self.ekf_odom and self.gz_truth:
            ep = self.ekf_odom.pose.pose.position
            gp = self.gz_truth.pose.pose.position
            err_x = ep.x - gp.x
            err_y = ep.y - gp.y
            err_2d = math.sqrt(err_x ** 2 + err_y ** 2)

            lines.append(f'\n EKF vs Ground Truth ERROR')
            lines.append(f'   Δx: {err_x:+8.3f}   Δy: {err_y:+8.3f}')
            lines.append(f'   2D error: {err_2d:.3f} m')
            if err_2d < 1.0:
                lines.append('   EXCELLENT — EKF within 1m of truth')
            elif err_2d < 5.0:
                lines.append('    GOOD — EKF within 5m of truth')
            elif err_2d < 20.0:
                lines.append('    FAIR — EKF drifting, GPS fusion may be weak')
            else:
                lines.append('    BAD — EKF is way off. GPS fusion likely broken!')

        lines.append('\n' + '=' * 65)
        self.get_logger().info('\n'.join(lines))


def main():
    rclpy.init()
    node = LocalizationDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
