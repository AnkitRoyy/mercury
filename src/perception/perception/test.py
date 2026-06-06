# stanley_controller_node.py  (no watchdog)

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

    

K_STANLEY  = 0.2
K_SOFT     = 3.0
MAX_STEER  = math.radians(25.0)
SPEED_MPS  = 1.0


class StanleyControllerNode(Node):

    def __init__(self):
        super().__init__("stanley_controller_node")

        self.declare_parameter("k_stanley", K_STANLEY)
        self.declare_parameter("k_soft",    K_SOFT)
        self.declare_parameter("max_steer", math.degrees(MAX_STEER))  # deg
        self.declare_parameter("speed_mps", SPEED_MPS)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        self.create_subscription(
                Float64MultiArray, "/lane_data_array", self._array_cb, 10
            )

        self._last_delta_rad = 0.0
        self._last_cte_m     = 0.0
        self._last_path_ang  = 0.0
        self._last_detected  = False
        self.create_timer(0.2, self._log_cb)

    # ── callbacks ────────────────────────────────────────────────────────────

    def _lane_cb(self, msg: "LaneData"):
        self._run_stanley(msg.cte_metres, msg.path_angle_rad, msg.lane_detected)

    def _array_cb(self, msg):
        if len(msg.data) < 4:
            self.get_logger().warn("lane_data_array: expected 4 floats")
            return
        self._run_stanley(
            cte_metres     = msg.data[0],
            path_angle_rad = msg.data[1],
            lane_detected  = bool(msg.data[3]),
        )

    # ── Stanley law ──────────────────────────────────────────────────────────

    def _run_stanley(self, cte_metres: float, path_angle_rad: float,
                     lane_detected: bool):

        if not lane_detected:
            self._last_detected = False
            self.cmd_pub.publish(Twist())   # stop
            return

        k             = self.get_parameter("k_stanley").value
        ks            = self.get_parameter("k_soft").value
        veh_speed     = self.get_parameter("speed_mps").value
        max_steer_rad = math.radians(self.get_parameter("max_steer").value)

        heading_error = -path_angle_rad
        cte_term      = math.atan2(k * cte_metres, veh_speed + ks)
        delta         = heading_error + cte_term
        delta         = max(-max_steer_rad, min(max_steer_rad, delta))

        self._last_delta_rad = delta
        self._last_cte_m     = cte_metres
        self._last_path_ang  = math.degrees(path_angle_rad)
        self._last_detected  = True

        twist           = Twist()
        twist.linear.x  = veh_speed
        twist.angular.z = -delta   # ROS: +left  Stanley: +right
        self.cmd_pub.publish(twist)

    # ── logger ───────────────────────────────────────────────────────────────

    def _log_cb(self):
        if not self._last_detected:
            self.get_logger().warn("Stanley: no lane – STOPPED")
            return

        k  = self.get_parameter("k_stanley").value
        ks = self.get_parameter("k_soft").value
        v  = self.get_parameter("speed_mps").value

        self.get_logger().info(
            f"┌─ Stanley ───────────────────────────────────────\n"
            f"│  CTE        : {self._last_cte_m:+.4f} m\n"
            f"│  Path angle : {self._last_path_ang:+.3f} deg\n"
            f"│  Steer delta: {math.degrees(self._last_delta_rad):+.3f} deg"
            f"  ({self._last_delta_rad:+.4f} rad)\n"
            f"│  Speed      : {v:.2f} m/s\n"
            f"│  Gains      : k={k}  ks={ks}\n"
            f"└─────────────────────────────────────────────────"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StanleyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()