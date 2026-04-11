"""
goal_decomposer_node.py
-----------------------
Intercepts large lateral goals and converts them into a smooth sequence
of forward-biased waypoints sent to Nav2's FollowWaypoints action.

Subscribe:  /goal_pose          (geometry_msgs/PoseStamped)  — from RViz / user
Subscribe:  /odom               (nav_msgs/Odometry)          — current pose
Action:     /follow_waypoints   (nav2_msgs/FollowWaypoints)  — Nav2 waypoint follower

Logic:
  1. When a goal arrives, compute lateral offset (perpendicular to current heading).
  2. If lateral offset < lateral_threshold  →  send goal directly to Nav2 (passthrough).
  3. If lateral offset ≥ lateral_threshold  →  decompose into N waypoints:
       - Each waypoint is `forward_step` meters ahead of the previous one.
       - Each waypoint shifts `lateral_step` meters toward the goal laterally.
       - Final waypoint is the original goal.
  This keeps the robot always moving forward, nudging sideways gradually,
  so the lane never leaves the camera FOV.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowWaypoints

import tf_transformations  # from tf_transformations package


class GoalDecomposerNode(Node):

    def __init__(self):
        super().__init__('goal_decomposer_node')

        # ── parameters ────────────────────────────────────────────────────
        # If the goal's lateral offset (metres, perpendicular to robot heading)
        # exceeds this, decompose it into waypoints.
        self.declare_parameter('lateral_threshold', 0.8)

        # Distance between consecutive intermediate waypoints (metres).
        # Smaller = smoother path, more waypoints.
        self.declare_parameter('forward_step', 0.6)

        # What fraction of remaining lateral error to close per waypoint.
        # 0.3 = close 30% each step → gradual drift. Lower = more forward-biased.
        self.declare_parameter('lateral_fraction', 0.3)

        # Max number of intermediate waypoints to generate (safety cap).
        self.declare_parameter('max_waypoints', 20)

        self.lateral_threshold = self.get_parameter('lateral_threshold').value
        self.forward_step      = self.get_parameter('forward_step').value
        self.lateral_fraction  = self.get_parameter('lateral_fraction').value
        self.max_waypoints     = self.get_parameter('max_waypoints').value

        # ── state ─────────────────────────────────────────────────────────
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0

        # ── subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.odom_cb, 10)

        # ── action client ─────────────────────────────────────────────────
        self._waypoint_client = ActionClient(
            self, FollowWaypoints, '/follow_waypoints')

        self.get_logger().info(
            f'GoalDecomposerNode ready  '
            f'lateral_threshold={self.lateral_threshold}m  '
            f'forward_step={self.forward_step}m  '
            f'lateral_fraction={self.lateral_fraction}')

    # ── odometry callback ─────────────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])

    # ── goal callback ─────────────────────────────────────────────────────
    def goal_cb(self, msg: PoseStamped):
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        # Vector from robot to goal in world frame
        dx = gx - self.current_x
        dy = gy - self.current_y

        # Project onto robot's forward and lateral axes
        forward_dist = dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)
        lateral_dist = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)

        self.get_logger().info(
            f'Goal received  forward={forward_dist:.2f}m  lateral={lateral_dist:.2f}m')

        if abs(lateral_dist) < self.lateral_threshold:
            # Goal is roughly ahead — send directly as a single waypoint
            self.get_logger().info('Goal within lateral threshold — sending directly')
            self._send_waypoints([msg])
            return

        # ── Decompose into forward-biased waypoints ───────────────────────
        waypoints = self._decompose(
            self.current_x, self.current_y, self.current_yaw,
            gx, gy, msg.pose.orientation, msg.header.frame_id)

        self.get_logger().info(
            f'Decomposed into {len(waypoints)} waypoints  '
            f'lateral={lateral_dist:.2f}m')

        self._send_waypoints(waypoints)

    # ── decomposition logic ───────────────────────────────────────────────
    def _decompose(self, rx, ry, ryaw, gx, gy, goal_orientation, frame_id):
        """
        Build a list of PoseStamped waypoints from (rx,ry) to (gx,gy).

        Strategy:
          - Start at current robot position.
          - Each step: move `forward_step` meters forward in current heading,
            then close `lateral_fraction` of remaining lateral error.
          - Stop generating intermediates once we're within `forward_step`
            of the goal, then append the exact goal as the final waypoint.
        """
        waypoints = []
        stamp     = self.get_clock().now().to_msg()

        # Running position (starts at robot)
        wx, wy  = rx, ry
        wyaw    = ryaw

        for _ in range(self.max_waypoints - 1):
            # Remaining vector to goal
            dx = gx - wx
            dy = gy - wy
            dist_to_goal = math.hypot(dx, dy)

            if dist_to_goal < self.forward_step:
                break   # Close enough — final waypoint will be the exact goal

            # Remaining lateral error (in robot-local frame at current wp)
            lateral_remaining = (
                -dx * math.sin(wyaw) + dy * math.cos(wyaw))

            # Step forward along current heading
            nx = wx + self.forward_step * math.cos(wyaw)
            ny = wy + self.forward_step * math.sin(wyaw)

            # Nudge laterally by a fraction of remaining lateral error
            lateral_nudge = lateral_remaining * self.lateral_fraction
            nx += lateral_nudge * (-math.sin(wyaw))   # lateral axis = -sin, cos
            ny += lateral_nudge * math.cos(wyaw)

            # Heading at this waypoint: point toward the next goal direction
            # (blend current yaw with direction-to-goal for smooth turning)
            angle_to_goal = math.atan2(gy - ny, gx - nx)
            # Interpolate: 70% keep current heading, 30% toward goal
            # This prevents sharp heading jumps between waypoints
            wyaw = _angle_lerp(wyaw, angle_to_goal, 0.3)

            wp = _make_pose(nx, ny, wyaw, frame_id, stamp)
            waypoints.append(wp)

            wx, wy = nx, ny

        # Always append the exact original goal as the final waypoint
        # (preserves the user's intended goal orientation)
        final = PoseStamped()
        final.header.frame_id = frame_id
        final.header.stamp    = stamp
        final.pose.position.x = gx
        final.pose.position.y = gy
        final.pose.position.z = 0.0
        final.pose.orientation = goal_orientation
        waypoints.append(final)

        return waypoints

    # ── action sender ─────────────────────────────────────────────────────
    def _send_waypoints(self, waypoints: list):
        if not self._waypoint_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                '/follow_waypoints action server not available')
            return

        goal_msg            = FollowWaypoints.Goal()
        goal_msg.poses      = waypoints

        self.get_logger().info(
            f'Sending {len(waypoints)} waypoint(s) to /follow_waypoints')

        send_future = self._waypoint_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Waypoint goal rejected by Nav2')
            return
        self.get_logger().info('Waypoint goal accepted')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        missed = result.missed_waypoints
        if missed:
            self.get_logger().warn(f'Missed waypoints: {missed}')
        else:
            self.get_logger().info('All waypoints reached successfully')


# ── helpers ───────────────────────────────────────────────────────────────

def _angle_lerp(a, b, t):
    """Interpolate between two angles, handling wrap-around."""
    diff = math.atan2(math.sin(b - a), math.cos(b - a))
    return a + t * diff


def _make_pose(x, y, yaw, frame_id, stamp):
    """Build a PoseStamped from x, y, yaw."""
    import tf_transformations
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp    = stamp
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    return ps


def main(args=None):
    rclpy.init(args=args)
    node = GoalDecomposerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()