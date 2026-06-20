#!/usr/bin/env python3
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import tf2_ros
from tf2_ros import TransformException


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MissionSetup(Node):
    def __init__(self):
        super().__init__("mission_setup")

        self.declare_parameter("generated_yaml_path", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")

        self.generated_yaml = self.get_parameter("generated_yaml_path").value
        if not self.generated_yaml:
            raise RuntimeError("generated_yaml_path not provided")

        self._map_frame = self.get_parameter("map_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.home_recorded = False
        # Poll for TF instead of waiting on odom - we need map->odom AND
        # base_link's current odom-frame pose, so check on a timer.
        self.create_timer(0.2, self._try_record_home)
        self.get_logger().info("Waiting for map->odom transform and robot pose...")

    def _try_record_home(self):
        if self.home_recorded:
            return

        try:
            # map -> odom: world-frame pose of the odom origin
            t_map_odom = self.tf_buffer.lookup_transform(
                self._map_frame, self._odom_frame, Time()
            )
            # odom -> base_link: robot's current pose within odom frame
            t_odom_base = self.tf_buffer.lookup_transform(
                self._odom_frame, "base_link", Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF not ready yet: {ex}")
            return

        self.home_recorded = True

        tx = t_map_odom.transform.translation.x
        ty = t_map_odom.transform.translation.y
        yaw = yaw_from_quaternion(t_map_odom.transform.rotation)

        ox = t_odom_base.transform.translation.x
        oy = t_odom_base.transform.translation.y

        wx = ox * math.cos(yaw) - oy * math.sin(yaw)
        wy = ox * math.sin(yaw) + oy * math.cos(yaw)
        home_x = tx + wx
        home_y = ty + wy

        watchdog_share = Path(get_package_share_directory("watchdog_monitor"))
        source_yaml = watchdog_share / "config" / "watchdog_params.yaml"

        with open(source_yaml, "r") as f:
            config = yaml.safe_load(f)

        wp_params = config["waypoints"]["ros__parameters"]
        wp_params["waypoints"].extend([home_x, home_y])
        wp_params["waypoint_names"].append("HOME")

        Path(self.generated_yaml).parent.mkdir(parents=True, exist_ok=True)
        with open(self.generated_yaml, "w") as f:
            yaml.safe_dump(config, f, sort_keys=False)

        self.get_logger().info(
            f"HOME recorded at map-frame ({home_x:.3f}, {home_y:.3f})\n"
            f"Generated:\n{self.generated_yaml}"
        )
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MissionSetup()
    rclpy.spin(node)


if __name__ == "__main__":
    main()