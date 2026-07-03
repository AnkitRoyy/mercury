#!/usr/bin/env python3
"""
gps_datum_injector.py
======================
ROS 2 node that runs ONCE at mission startup (real hardware only).

What it does
------------
1. Subscribes to /gps (sensor_msgs/NavSatFix).
2. Waits for `datum_avg_samples` consecutive valid fixes and averages
   them → this averaged fix is the datum (= map-frame origin, matching
   navsat_transform_node's wait_for_datum: false behaviour).
3. Reads the GPS waypoints from gps_waypoints.yaml
   (list of [NAME, lat, lon] entries).
4. Converts each waypoint to local ENU (x, y) metres using the datum.
5. Overwrites the waypoints list in mission_params.yaml so that
   mission_setup and waypoint_sender see real-world Cartesian coords.
6. Shuts itself down — mission_setup takes over next.

Timing note
-----------
navsat_transform_node has a `delay: 3.0` s before it starts publishing
/odometry/gps, but the raw /gps fix is available immediately from the
hardware driver.  This node reads /gps directly (same source), so it
can finish well before navsat is ready, giving the EKF time to warm up.
"""

import math
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


# ── Flat-earth ENU conversion (mirrors gps_utils.py) ──────────────

def gps_to_local(
    lat0: float, lon0: float,
    lat:  float, lon:  float,
) -> tuple[float, float]:
    lat0_rad = math.radians(lat0)
    m_per_deg_lat = (
        111132.92
        - 559.82 * math.cos(2 * lat0_rad)
        + 1.175  * math.cos(4 * lat0_rad)
    )
    m_per_deg_lon = (
        111412.84 * math.cos(lat0_rad)
        - 93.5    * math.cos(3 * lat0_rad)
    )
    x = (lon - lon0) * m_per_deg_lon
    y = (lat - lat0) * m_per_deg_lat
    return x, y


# ── Node ──────────────────────────────────────────────────────────

class GpsDatumInjector(Node):

    def __init__(self):
        super().__init__('gps_datum_injector')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('gps_topic',          '/gps')
        self.declare_parameter('datum_avg_samples',  5)
        self.declare_parameter('generated_yaml_path', '')

        # gps_waypoints: list of [NAME, lat, lon] triples stored flat
        # ROS 2 doesn't support nested lists as params; we pass the
        # YAML path instead and read it directly.
        self.declare_parameter('gps_waypoints_yaml', '')

        gps_topic     = self.get_parameter('gps_topic').value
        self._samples = self.get_parameter('datum_avg_samples').value
        self._out_yaml = self.get_parameter('generated_yaml_path').value
        self._wp_yaml  = self.get_parameter('gps_waypoints_yaml').value

        if not self._out_yaml:
            raise RuntimeError('generated_yaml_path not set')
        if not self._wp_yaml:
            raise RuntimeError('gps_waypoints_yaml not set')

        # ── State ─────────────────────────────────────────────────
        self._fixes: list[tuple[float, float]] = []
        self._done  = False

        # ── Subscriber ────────────────────────────────────────────
        self._sub = self.create_subscription(
            NavSatFix,
            gps_topic,
            self._gps_cb,
            10,
        )

        self.get_logger().info(
            f'Waiting for {self._samples} GPS fixes on {gps_topic} …'
        )

    # ── GPS callback ──────────────────────────────────────────────

    def _gps_cb(self, msg: NavSatFix):
        if self._done:
            return

        # Only accept a valid fix (STATUS_FIX or better)
        if msg.status.status < 0:
            self.get_logger().warn('GPS fix not valid yet — waiting …', throttle_duration_sec=5.0)
            return

        self._fixes.append((msg.latitude, msg.longitude))
        remaining = self._samples - len(self._fixes)
        if remaining > 0:
            self.get_logger().info(
                f'Collecting GPS fixes: {len(self._fixes)}/{self._samples}'
            )
            return

        # ── Enough samples — compute datum ────────────────────────
        datum_lat = sum(f[0] for f in self._fixes) / len(self._fixes)
        datum_lon = sum(f[1] for f in self._fixes) / len(self._fixes)
        self._done = True
        self.destroy_subscription(self._sub)

        self.get_logger().info(
            f'Datum established: lat={datum_lat:.7f}, lon={datum_lon:.7f}'
        )

        self._convert_and_inject(datum_lat, datum_lon)

    # ── Conversion + YAML write ───────────────────────────────────

    def _convert_and_inject(self, datum_lat: float, datum_lon: float):

        # 1. Load GPS waypoints from gps_waypoints.yaml
        with open(self._wp_yaml, 'r') as f:
            wp_config = yaml.safe_load(f)

        raw_wps = (
            wp_config
            .get('gps_datum_injector', {})
            .get('ros__parameters', {})
            .get('gps_waypoints', [])
        )

        if not raw_wps:
            self.get_logger().error(
                'No gps_waypoints found in gps_waypoints.yaml — aborting.'
            )
            rclpy.shutdown()
            return

        # 2. Convert each [NAME, lat, lon] → (x, y)
        converted_xy   = []   # flat [x0, y0, x1, y1, …]
        converted_names = []

        for entry in raw_wps:
            if len(entry) != 3:
                self.get_logger().error(f'Bad waypoint entry (expected 3 elements): {entry}')
                continue
            name, lat, lon = entry[0], float(entry[1]), float(entry[2])
            x, y = gps_to_local(datum_lat, datum_lon, lat, lon)
            converted_xy.extend([round(x, 4), round(y, 4)])
            converted_names.append(name)
            self.get_logger().info(
                f'  {name:8s} ({lat:.7f}, {lon:.7f}) → ({x:.3f}, {y:.3f}) m'
            )

        # 3. Load mission_params.yaml and patch it
        with open(self._out_yaml, 'r') as f:
            config = yaml.safe_load(f)

        wp_params = config['waypoints']['ros__parameters']

        # Keep any existing HOME entry appended by a previous mission_setup run
        old_names = wp_params.get('waypoint_names', [])
        old_flat  = wp_params.get('waypoints', [])

        # Strip the old GPS/sim waypoints (everything except HOME if present)
        n_old_wps = len(old_names)
        home_entries = []
        for i, n in enumerate(old_names):
            if n == 'HOME':
                home_entries.extend([old_flat[i * 2], old_flat[i * 2 + 1]])

        # Replace waypoints list (HOME gets re-appended by mission_setup at runtime)
        wp_params['waypoints']      = converted_xy
        wp_params['waypoint_names'] = converted_names

        # 4. Write the patched YAML
        Path(self._out_yaml).parent.mkdir(parents=True, exist_ok=True)
        with open(self._out_yaml, 'w') as f:
            yaml.safe_dump(config, f, sort_keys=False)

        self.get_logger().info(
            f'mission_params.yaml updated with {len(converted_names)} real-world waypoints.\n'
            f'Output: {self._out_yaml}'
        )

        rclpy.shutdown()


# ── Entry point ───────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GpsDatumInjector()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
