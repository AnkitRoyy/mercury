"""
lane_detection.py
-----------------
Subscribes to /camera/image_raw, detects white lane lines using BEV
homography (reuses the same logic already in lane_costmap.py), and
publishes a Float32 to /lane_center_error representing the horizontal
pixel offset of the lane centre from the image centre (positive = robot
is left of centre, negative = right).

Also publishes /lane_debug/image (sensor_msgs/Image) so you can watch
what the node sees in RViz / rqt_image_view without any extra tooling.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge


class LaneDetectionNode(Node):

    def __init__(self):
        super().__init__('lane_detection_node')

        # ── parameters ──────────────────────────────────────────────
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('bev_width', 640)
        self.declare_parameter('bev_height', 480)
        self.declare_parameter('show_debug', True)
        # NOTE: use_sim_time is a built-in ROS 2 parameter — do NOT declare it

        self.bev_w = self.get_parameter('bev_width').value
        self.bev_h = self.get_parameter('bev_height').value
        self.show_debug = self.get_parameter('show_debug').value
        image_topic = self.get_parameter('image_topic').value

        self.bridge = CvBridge()
        self.homography = None

        # ── QoS ─────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── subscribers ──────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, sensor_qos)

        # ── publishers ───────────────────────────────────────────────
        # Primary output consumed by lane_assist_node
        self.error_pub = self.create_publisher(Float32, '/lane_center_error', 10)

        # Visibility flag — True when at least one lane line is visible
        self.visible_pub = self.create_publisher(Bool, '/lane_visible', 10)

        # Debug image (BEV with overlay)
        self.debug_pub = self.create_publisher(Image, '/lane_debug/image', sensor_qos)

        self.get_logger().info('LaneDetectionNode started')

    # ── homography ───────────────────────────────────────────────────
    def _compute_homography(self, frame):
        h, w = frame.shape[:2]
        # Source quad — tuned for the camera pose in robot_base.xacro
        # (xyz="0.15 0 2" rpy="0 0.4 0").  Adjust if you re-mount the cam.
        src = np.float32([
            [w * 0.02,  h * 0.525],   # bottom-left
            [w * 0.98,  h * 0.525],   # bottom-right
            [w * 0.225, h * 0.30],    # top-left
            [w * 0.775, h * 0.30],    # top-right
        ])
        dst = np.float32([
            [0,            self.bev_h],
            [self.bev_w,   self.bev_h],
            [0,            0],
            [self.bev_w,   0],
        ])
        self.homography = cv2.getPerspectiveTransform(src, dst)
        self.get_logger().info('Homography computed')

    # ── white-line mask (same thresholds as lane_costmap.py) ─────────
    def _detect_white(self, bev):
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        mask_hsv = cv2.inRange(hsv,
                               np.array([0, 0, 180]),
                               np.array([180, 80, 255]))

        gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        _, mask_bright = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

        mask = cv2.bitwise_and(mask_hsv, mask_bright)

        kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)

        kernel_dil = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        mask = cv2.dilate(mask, kernel_dil, iterations=1)

        # Ignore bottom strip (robot body shadow)
        mask[-20:, :] = 0

        return mask

    # ── lane-centre computation ──────────────────────────────────────
    def _compute_error(self, mask):
        """
        Split BEV into left / right halves.  Find the centroid X of white
        pixels in each half, average them to get the lane centre, then
        compute the offset from the image centre.

        Returns (error, visible) where:
          error   – pixels, positive = robot is left of lane centre
          visible – True when at least one lane line detected
        """
        half = self.bev_w // 2
        cx = self.bev_w // 2   # image centre x

        left_mask  = mask[:, :half]
        right_mask = mask[:, half:]

        left_pts  = np.argwhere(left_mask  > 0)
        right_pts = np.argwhere(right_mask > 0)

        found_left  = len(left_pts)  > 50   # noise threshold
        found_right = len(right_pts) > 50

        if not found_left and not found_right:
            return 0.0, False

        lane_cx_candidates = []
        if found_left:
            left_cx = float(np.mean(left_pts[:, 1]))          # col in left half
            lane_cx_candidates.append(left_cx)
        if found_right:
            right_cx = float(np.mean(right_pts[:, 1])) + half # col in full image
            lane_cx_candidates.append(right_cx)

        lane_cx = float(np.mean(lane_cx_candidates))
        error = cx - lane_cx   # positive → robot left of centre → turn right
        return error, True

    # ── main callback ────────────────────────────────────────────────
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        if self.homography is None:
            self._compute_homography(frame)

        bev = cv2.warpPerspective(frame, self.homography,
                                  (self.bev_w, self.bev_h))
        mask = self._detect_white(bev)
        error, visible = self._compute_error(mask)

        # Publish error
        err_msg = Float32()
        err_msg.data = float(error)
        self.error_pub.publish(err_msg)

        # Publish visibility flag
        vis_msg = Bool()
        vis_msg.data = visible
        self.visible_pub.publish(vis_msg)

        # Publish debug image
        overlay = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        if visible:
            cx = self.bev_w // 2
            lane_cx = int(cx - error)
            cv2.line(overlay, (cx, 0), (cx, self.bev_h), (0, 255, 0), 2)   # img centre
            cv2.line(overlay, (lane_cx, 0), (lane_cx, self.bev_h), (0, 0, 255), 2)  # lane centre
            cv2.putText(overlay, f'err={error:.1f}px', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

        debug_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

        if self.show_debug:
            cv2.imshow('BEV mask', overlay)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()