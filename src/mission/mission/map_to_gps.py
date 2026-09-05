#!/usr/bin/env python3
"""
map_to_gps.py — map-frame (x, y) → Lat/Lon
=================================================
Reverse of send_gps_goal.py. Converts map-frame coordinates into GPS
lat/lon using navsat_transform_node's /toLL service, so the datum lock
(and thus the conversion) is consistent with whatever /fromLL uses.

Usage:
    # single point
    ros2 run mission map_to_gps --x 15.57 --y -35.36

    # every waypoint in mission_params.yaml (or any yaml with the same
    # 'waypoints: {ros__parameters: {waypoints: [...], waypoint_names: [...]}}'
    # structure)
    ros2 run mission map_to_gps --yaml src/mission/config/mission_params.yaml
"""

import argparse
import sys

import rclpy
import yaml
from rclpy.node import Node
from robot_localization.srv import ToLL


class MapToGpsNode(Node):

    def __init__(self):
        super().__init__('map_to_gps')
        self._client = self.create_client(ToLL, '/toLL')

    def _wait_for_service(self) -> bool:
        self.get_logger().info('Waiting for /toLL service (navsat_transform_node)...')
        if not self._client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'navsat_transform_node not running / datum not locked yet — '
                'is real_localization.launch.py up?'
            )
            return False
        return True

    def convert(self, x: float, y: float, z: float = 0.0):
        """Convert map-frame (x, y[, z]) → (lat, lon, alt) via /toLL."""
        req = ToLL.Request()
        req.map_point.x = x
        req.map_point.y = y
        req.map_point.z = z

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('/toLL service call failed or timed out.')
            return None

        ll = future.result().ll_point
        return ll.latitude, ll.longitude, ll.altitude

    def convert_yaml(self, yaml_path: str):
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        wp_params = config['waypoints']['ros__parameters']
        flat = wp_params['waypoints']
        names = wp_params.get('waypoint_names', [])

        results = []
        for i in range(0, len(flat), 2):
            x, y = float(flat[i]), float(flat[i + 1])
            name = names[i // 2] if (i // 2) < len(names) else f'WP-{(i // 2) + 1}'
            ll = self.convert(x, y)
            if ll is None:
                self.get_logger().error(f'Failed to convert {name} ({x}, {y})')
                continue
            lat, lon, alt = ll
            results.append((name, x, y, lat, lon, alt))
            self.get_logger().info(
                f'{name}: map({x:.2f}, {y:.2f}) -> lat={lat:.7f}, lon={lon:.7f}'
            )
        return results


def main():
    parser = argparse.ArgumentParser(
        description='Convert map-frame coordinates to GPS lat/lon via /toLL'
    )
    parser.add_argument('--x', type=float, help='map-frame x')
    parser.add_argument('--y', type=float, help='map-frame y')
    parser.add_argument('--z', type=float, default=0.0, help='map-frame z (default 0.0)')
    parser.add_argument('--yaml', type=str, help='path to a waypoints yaml (mission_params.yaml format)')
    args = parser.parse_args()

    if args.yaml is None and (args.x is None or args.y is None):
        parser.error('Provide either --x and --y, or --yaml')

    rclpy.init()
    node = MapToGpsNode()

    if not node._wait_for_service():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if args.yaml:
        node.convert_yaml(args.yaml)
    else:
        result = node.convert(args.x, args.y, args.z)
        if result is None:
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        lat, lon, alt = result
        node.get_logger().info(
            f'map({args.x:.2f}, {args.y:.2f}) -> lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}'
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()