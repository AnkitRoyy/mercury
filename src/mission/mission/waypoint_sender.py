#!/usr/bin/env python3

import json
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class WaypointSender(Node):

    def __init__(self):
        super().__init__("waypoint_sender")

        self.declare_parameter(
            "generated_yaml_path",
            ""
        )

        self.generated_yaml = self.get_parameter(
            "generated_yaml_path"
        ).value

        if not self.generated_yaml:
            raise RuntimeError(
                "generated_yaml_path parameter not provided"
            )

        self.current_index = 0
        self.waiting_for_reach = False
        self.started = False

        self.load_waypoints()

        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/final_goal",
            10
        )

        self.reached_sub = self.create_subscription(
            String,
            "/waypoint_reached",
            self.waypoint_reached_callback,
            10
        )

        self.create_timer(
            1.0,
            self.start_once
        )

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints"
        )

    def load_waypoints(self):

        with open(self.generated_yaml, "r") as f:
            config = yaml.safe_load(f)

        wp_params = config[
            "waypoints"
        ]["ros__parameters"]

        flat_waypoints = wp_params[
            "waypoints"
        ]

        waypoint_names = wp_params[
            "waypoint_names"
        ]

        self.waypoints = []

        for i in range(
            0,
            len(flat_waypoints),
            2
        ):

            idx = i // 2

            self.waypoints.append({
                "name": waypoint_names[idx],
                "x": float(flat_waypoints[i]),
                "y": float(flat_waypoints[i + 1]),
            })

    def start_once(self):

        if self.started:
            return

        self.started = True

        self.get_logger().info(
            "Starting mission"
        )

        self.send_current_waypoint()

    def send_current_waypoint(self):

        if self.current_index >= len(
            self.waypoints
        ):

            self.get_logger().info(
                "Mission complete"
            )

            return

        waypoint = self.waypoints[
            self.current_index
        ]

        goal = PoseStamped()

        goal.header.frame_id = "map"
        goal.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        goal.pose.position.x = waypoint["x"]
        goal.pose.position.y = waypoint["y"]
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)

        self.waiting_for_reach = True

        self.get_logger().info(
            f"Sent {waypoint['name']} "
            f"({waypoint['x']:.2f}, "
            f"{waypoint['y']:.2f})"
        )

    def waypoint_reached_callback(
        self,
        msg
    ):

        if not self.waiting_for_reach:
            return

        try:

            data = json.loads(
                msg.data
            )

            reached_name = (
                data
                .get(
                    "waypoint",
                    {}
                )
                .get(
                    "name",
                    ""
                )
            )

        except Exception:

            reached_name = ""

        expected_name = (
            self.waypoints[
                self.current_index
            ]["name"]
        )

        if (
            reached_name
            and
            reached_name != expected_name
        ):

            self.get_logger().warn(
                f"Ignoring waypoint "
                f"{reached_name}, "
                f"expected "
                f"{expected_name}"
            )

            return

        self.get_logger().info(
            f"Reached "
            f"{expected_name}"
        )

        self.current_index += 1
        self.waiting_for_reach = False

        self.send_current_waypoint()


def main(args=None):

    rclpy.init(args=args)

    node = WaypointSender()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()