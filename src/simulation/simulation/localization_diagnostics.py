#!/usr/bin/env python3
"""
Localization Diagnostics Node — Real Hardware
==============================================
Monitors EKF fusion health on the real rover. There is no ground truth on
hardware, so this does NOT compute an "error vs truth" number. Instead it
shows:
  - Raw GPS fix quality (status, covariance, satellite-derived confidence)
  - navsat_transform output (/odometry/gps)
  - EKF fused output (/odometry/filtered) — what Nav2 actually uses
  - Wheel odom (/odom) as a drift-check reference — NOT ground truth,
    just useful to see how far the EKF has pulled away from raw odom
    (e.g. because GPS correction kicked in, or because odom drifted)

Subscribes:
  /gps                  — raw GPS fix (sensor_msgs/NavSatFix), from nmea_serial_driver
  /odometry/gps         — navsat_transform_node output
  /odometry/filtered    — EKF fused output
  /odom                 — wheel odometry

Datum defaults to the last computed navsat_transform_node datum
(update DATUM_LAT/DATUM_LON below to match your current launch site,
or leave auto-datum mode on to grab it from the first GPS fix instead).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# Set this to match your actual launch site datum (e.g. from navsat_transform_node
# startup log: "Datum (latitude, longitude, altitude) is (...)").
# Leave as None to auto-capture from the first GPS fix received instead.
DATUM_LAT = None
DATUM_LON = None

# GPS fix status meaning (sensor_msgs/NavSatStatus)
FIX_STATUS = {
    -1: 'NO FIX',
    0: 'FIX (no augmentation)',
    1: 'SBAS FIX',
    2: 'GBAS FIX',
}


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

        self.datum_lat = DATUM_LAT
        self.datum_lon = DATUM_LON

        self.gps_data = None
        self.odom_gps = None
        self.ekf_odom = None
        self.wheel_odom = None

        # GPS/serial drivers often publish best-effort; match QoS so we don't
        # silently drop messages against a mismatched default.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(NavSatFix, '/gps', self._cb_gps, sensor_qos)
        self.create_subscription(Odometry, '/odometry/gps', self._cb_odom_gps, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._cb_ekf, 10)
        self.create_subscription(Odometry, '/odom', self._cb_wheel, 10)

        self.timer = self.create_timer(2.0, self._print_dashboard)
        self.get_logger().info(
            'localization diagnostics (real hardware) started. dashboard prints every 2s...')
        if self.datum_lat is None:
            self.get_logger().info('DATUM not set — will auto-capture from first GPS fix.')

    def _cb_gps(self, msg):
        self.gps_data = msg
        if self.datum_lat is None and msg.status.status >= 0:
            self.datum_lat = msg.latitude
            self.datum_lon = msg.longitude
            self.get_logger().info(
                f'Auto-captured datum: ({self.datum_lat:.7f}, {self.datum_lon:.7f})')

    def _cb_odom_gps(self, msg):
        self.odom_gps = msg

    def _cb_ekf(self, msg):
        self.ekf_odom = msg

    def _cb_wheel(self, msg):
        self.wheel_odom = msg

    def _print_dashboard(self):
        lines = []
        lines.append('\n' + '=' * 65)
        lines.append('     localization diagnostics dashboard (real hardware)')
        lines.append('=' * 65)

        # --- GPS Raw ---
        if self.gps_data:
            g = self.gps_data
            status_str = FIX_STATUS.get(g.status.status, f'UNKNOWN({g.status.status})')
            lines.append(f'\nGPS raw (/gps)  frame: {g.header.frame_id}')
            lines.append(f'   status: {status_str}')
            lines.append(f'   lat: {g.latitude:.7f}   lon: {g.longitude:.7f}   alt: {g.altitude:.2f}')
            if g.status.status < 0:
                lines.append('   NO FIX — position below is stale/invalid, ignore it')
            elif self.datum_lat is not None:
                dist = haversine_m(self.datum_lat, self.datum_lon, g.latitude, g.longitude)
                lines.append(f'   distance from datum: {dist:.2f} m')
            cov_diag = [g.position_covariance[0],
                        g.position_covariance[4],
                        g.position_covariance[8]]
            lines.append(f'   covariance diag (m^2): {cov_diag}  type: {g.position_covariance_type}')
            if cov_diag[0] > 100 or cov_diag[4] > 100:
                lines.append('   ⚠ covariance is very high — GPS fix is low quality, EKF should be discounting it')
        else:
            lines.append('\nGPS Raw (/gps):   NO DATA — check nmea_serial_driver is running and connected')

        # --- Navsat Odom ---
        if self.odom_gps:
            p = self.odom_gps.pose.pose.position
            lines.append(f'\nNavsat Odom (/odometry/gps)  frame: {self.odom_gps.header.frame_id}')
            lines.append(f'   x: {p.x:8.3f}   y: {p.y:8.3f}   z: {p.z:8.3f}')
        else:
            lines.append('\nNavsat Odom (/odometry/gps):   NO DATA')
            lines.append('   → navsat_transform_node is not producing output (no valid fix yet, or not receiving /imu for heading)')

        # --- EKF ---
        if self.ekf_odom:
            p = self.ekf_odom.pose.pose.position
            lines.append(f'\nEKF Output (/odometry/filtered)  frame: {self.ekf_odom.header.frame_id}')
            lines.append(f'   x: {p.x:8.3f}   y: {p.y:8.3f}   z: {p.z:8.3f}')
        else:
            lines.append('\nEKF Output (/odometry/filtered):   NO DATA')

        # --- Wheel odom (reference only, NOT ground truth) ---
        if self.wheel_odom:
            p = self.wheel_odom.pose.pose.position
            lines.append(f'\nWheel Odom (/odom) — reference only, NOT ground truth')
            lines.append(f'   x: {p.x:8.3f}   y: {p.y:8.3f}   z: {p.z:8.3f}')
        else:
            lines.append('\nWheel Odom (/odom):   NO DATA')

        # --- Drift check: EKF vs wheel odom ---
        if self.ekf_odom and self.wheel_odom:
            ep = self.ekf_odom.pose.pose.position
            wp = self.wheel_odom.pose.pose.position
            diff_x = ep.x - wp.x
            diff_y = ep.y - wp.y
            diff_2d = math.sqrt(diff_x ** 2 + diff_y ** 2)

            lines.append(f'\nEKF vs Wheel Odom DIVERGENCE (not an error metric — no ground truth on hardware)')
            lines.append(f'   Δx: {diff_x:+8.3f}   Δy: {diff_y:+8.3f}')
            lines.append(f'   2D divergence: {diff_2d:.3f} m')
            lines.append('   Large divergence is expected once GPS correction kicks in;')
            lines.append('   a divergence that grows unbounded with no GPS fix suggests wheel odom drift.')

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