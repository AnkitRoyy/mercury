import cv2
import numpy as np
import json
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory
import os
# add to perceptio/lane_perception
pkg_share = get_package_share_directory("perception")

LANE_CONFIG_PATH    = os.path.join(pkg_share, "config", "lane_config.json")
BEV_CONFIG_PATH     = os.path.join(pkg_share, "config", "bev_config.json")
SLIDING_CONFIG_PATH = os.path.join(pkg_share, "config", "sliding_window_config.json")
ROAD_CONFIG_PATH    = os.path.join(pkg_share, "config", "road_config.json")

LOOKAHEAD_RATIO    = 0.60
XM_PER_PIX         = 3.7 / 700
MAX_MISSING_FRAMES = 15
ROAD_FIT_ALPHA     = 0.6


def load_json(path):
    with open(path, "r") as f:
        return json.load(f)


class LaneFollowingNode(Node):

    def __init__(self):
        super().__init__("lane_following_node")

        self.bridge      = CvBridge()
        self.lane_cfg    = load_json(LANE_CONFIG_PATH)
        self.bev_cfg     = load_json(BEV_CONFIG_PATH)
        self.sliding_cfg = load_json(SLIDING_CONFIG_PATH)

        try:
            self.road_cfg = load_json(ROAD_CONFIG_PATH)
        except FileNotFoundError:
            self.get_logger().warn("road_config.json not found – road fallback disabled.")
            self.road_cfg = None

        src = np.float32(self.bev_cfg["src_points"])
        dst = np.float32(self.bev_cfg["dst_points"])
        self.M          = cv2.getPerspectiveTransform(src, dst)
        self.bev_width  = int(np.max(dst[:, 0]))
        self.bev_height = int(np.max(dst[:, 1]))

        self.last_left_fit  = None
        self.last_right_fit = None
        self.last_road_fit  = None
        self.missing_frames = 0

        self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.lane_pub = self.create_publisher(Float64MultiArray, "/lane_data_array", 10)
        self.get_logger().info("Lane following node started.")

    def image_callback(self, msg: Image):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.process(image)

    # ── road mask ─────────────────────────────────────────────────────────────

    def _road_mask(self, bev_bgr):
        if self.road_cfg is None:
            return None
        cfg = self.road_cfg
        hsv = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
        lower = np.array([cfg["h_min"], cfg["s_min"], cfg["v_min"]])
        upper = np.array([cfg["h_max"], cfg["s_max"], cfg["v_max"]])
        m = cv2.inRange(hsv, lower, upper)
        k = np.ones((5, 5), np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  k)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
        return m

    # ── road center fit ───────────────────────────────────────────────────────

    def _road_center_fit(self, road_mask, window_height):
        h, w = road_mask.shape
        pts = []
        y = h
        while y > 0:
            y_low = max(0, y - window_height)
            strip = road_mask[y_low:y, :]
            m = cv2.moments(strip)
            if m["m00"] > 0:
                cx    = int(m["m10"] / m["m00"])
                mid_y = (y_low + y) // 2
                pts.append((cx, mid_y))
            y -= window_height

        if len(pts) < 3:
            return None

        xs  = np.array([p[0] for p in pts])
        ys  = np.array([p[1] for p in pts])
        fit = np.polyfit(ys, xs, 2)

        if self.last_road_fit is not None:
            fit = ROAD_FIT_ALPHA * fit + (1.0 - ROAD_FIT_ALPHA) * self.last_road_fit

        return fit, pts

    # ── main processing ───────────────────────────────────────────────────────

    def process(self, image):
        h, w = image.shape[:2]
        lane_cfg    = self.lane_cfg
        sliding_cfg = self.sliding_cfg

        top    = int(h * lane_cfg["roi_top"]    / 100.0)
        bottom = int(h * lane_cfg["roi_bottom"] / 100.0)
        left   = int(w * lane_cfg["roi_left"]   / 100.0)
        right  = int(w * lane_cfg["roi_right"]  / 100.0)

        roi_mask  = np.zeros((h, w), dtype=np.uint8)
        cv2.rectangle(roi_mask, (left, top), (right, bottom), 255, -1)
        roi_image = cv2.bitwise_and(image, image, mask=roi_mask)
        bev       = cv2.warpPerspective(roi_image, self.M, (self.bev_width, self.bev_height))

        hsv   = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        lower = np.array([lane_cfg["h_min"], lane_cfg["s_min"], lane_cfg["v_min"]])
        upper = np.array([lane_cfg["h_max"], lane_cfg["s_max"], lane_cfg["v_max"]])
        mask  = cv2.inRange(hsv, lower, upper)
        ks    = max(1, lane_cfg["kernel_size"])
        k     = np.ones((ks, ks), np.uint8)
        mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask  = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        bev_full      = cv2.warpPerspective(image, self.M, (self.bev_width, self.bev_height))
        road_mask_bev = self._road_mask(bev_full)

        debug = bev_full.copy()
        if road_mask_bev is not None:
            green_layer = debug.copy()
            green_layer[road_mask_bev > 0] = (0, 200, 0)
            debug = cv2.addWeighted(green_layer, 0.5, debug, 0.5, 0)

        histogram = np.sum(mask[mask.shape[0] // 2:, :], axis=0)
        midpoint  = histogram.shape[0] // 2

        HIST_MIN_PEAK = 500
        left_hist  = histogram[:midpoint]
        right_hist = histogram[midpoint:]
        left_base  = int(np.argmax(left_hist))             if int(np.max(left_hist))  >= HIST_MIN_PEAK else None
        right_base = int(np.argmax(right_hist) + midpoint) if int(np.max(right_hist)) >= HIST_MIN_PEAK else None

        margin           = sliding_cfg["margin"]
        window_height    = sliding_cfg["window_height"]
        max_shift        = sliding_cfg["max_shift"]
        min_contour_area = sliding_cfg["min_contour_area"]
        point_radius     = sliding_cfg["point_radius"]

        current_left  = left_base
        current_right = right_base
        left_points   = []
        right_points  = []

        y = mask.shape[0]
        while y > 0:
            y_low  = max(0, y - window_height)
            y_high = y

            for side in ("left", "right"):
                if side == "left"  and current_left  is None: continue
                if side == "right" and current_right is None: continue

                if side == "left":
                    x_low  = max(0, current_left - margin)
                    x_high = min(mask.shape[1], current_left + margin)
                    current = current_left
                else:
                    x_low  = max(0, current_right - margin)
                    x_high = min(mask.shape[1], current_right + margin)
                    current = current_right

                roi = mask[y_low:y_high, x_low:x_high]
                contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                best_x    = None
                best_area = 0
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area < min_contour_area:
                        continue
                    if area > best_area:
                        m = cv2.moments(cnt)
                        if m["m00"] > 0:
                            cx        = int(m["m10"] / m["m00"])
                            candidate = x_low + cx
                            if abs(candidate - current) <= max_shift:
                                best_x    = candidate
                                best_area = area

                if best_x is not None:
                    mid_y = (y_low + y_high) // 2
                    if side == "left":
                        current_left = best_x
                        left_points.append((current_left, mid_y))
                    else:
                        current_right = best_x
                        right_points.append((current_right, mid_y))

            y -= window_height

        plot_y    = np.linspace(0, mask.shape[0] - 1, mask.shape[0])
        left_fit  = None
        right_fit = None

        if len(left_points) >= 3:
            lx = np.array([p[0] for p in left_points])
            ly = np.array([p[1] for p in left_points])
            left_fit = np.polyfit(ly, lx, 2)

        if len(right_points) >= 3:
            rx = np.array([p[0] for p in right_points])
            ry = np.array([p[1] for p in right_points])
            right_fit = np.polyfit(ry, rx, 2)

        # road center fit
        road_fit = None
        road_pts = []
        if road_mask_bev is not None:
            road_result = self._road_center_fit(road_mask_bev, window_height)
            if road_result is not None:
                road_fit, road_pts = road_result
                self.last_road_fit = road_fit
                fx = road_fit[0]*plot_y**2 + road_fit[1]*plot_y + road_fit[2]
                for i in range(len(plot_y) - 1):
                    p1 = (int(np.clip(fx[i],   0, self.bev_width-1)), int(plot_y[i]))
                    p2 = (int(np.clip(fx[i+1], 0, self.bev_width-1)), int(plot_y[i+1]))
                    cv2.line(debug, p1, p2, (0, 255, 0), 2)
                for p in road_pts:
                    cv2.circle(debug, p, point_radius, (0, 220, 0), -1)
        elif self.last_road_fit is not None:
            road_fit = self.last_road_fit

        # draw lane points and fits
        for pts, color in [(left_points, (0, 255, 0)), (right_points, (0, 0, 255))]:
            for p in pts:
                cv2.circle(debug, p, point_radius, color, -1)

        for fit, colour in [(left_fit, (0, 255, 255)), (right_fit, (255, 255, 0))]:
            if fit is not None:
                fx = fit[0]*plot_y**2 + fit[1]*plot_y + fit[2]
                for i in range(len(plot_y) - 1):
                    p1 = (int(np.clip(fx[i],   0, self.bev_width-1)), int(plot_y[i]))
                    p2 = (int(np.clip(fx[i+1], 0, self.bev_width-1)), int(plot_y[i+1]))
                    cv2.line(debug, p1, p2, colour, 2)

        # update caches with fresh fits only
        if left_fit  is not None: self.last_left_fit  = left_fit
        if right_fit is not None: self.last_right_fit = right_fit

        # ── simple weighted fusion (fresh fits only — no stale lane cache) ────
        #
        # road_fit  weight 2  — moment centroid, most reliable on curves
        # both lanes weight 2 — averaged first, then added (equal to road)
        # one lane   weight 1 — half influence of road
        # fallback   cached road only if nothing fresh at all
        fits    = []
        weights = []

        if road_fit is not None:
            fits.append(road_fit)
            weights.append(2)

        if left_fit is not None and right_fit is not None:
            lane_center = (left_fit + right_fit) / 2.0
            fits.append(lane_center)
            weights.append(2)
        elif left_fit is not None:
            fits.append(left_fit)
            weights.append(1)
        elif right_fit is not None:
            fits.append(right_fit)
            weights.append(1)

        if not fits and self.last_road_fit is not None:
            fits.append(self.last_road_fit)
            weights.append(1)

        center_fit = None
        if fits:
            center_fit = np.average(np.array(fits), axis=0,
                                    weights=np.array(weights, dtype=float))

        # ── publish ───────────────────────────────────────────────────────────
        if center_fit is not None:
            self.missing_frames = 0

            center_x = center_fit[0]*plot_y**2 + center_fit[1]*plot_y + center_fit[2]

            for i in range(len(plot_y) - 1):
                p1 = (int(np.clip(center_x[i],   0, self.bev_width-1)), int(plot_y[i]))
                p2 = (int(np.clip(center_x[i+1], 0, self.bev_width-1)), int(plot_y[i+1]))
                cv2.line(debug, p1, p2, (255, 0, 255), 2)

            axle_y_pix     = float(mask.shape[0] - 1)
            axle_x_pix     = float(mask.shape[1]) / 2.0
            center_at_axle = (center_fit[0]*axle_y_pix**2
                              + center_fit[1]*axle_y_pix
                              + center_fit[2])
            cte_pixels     = center_at_axle - axle_x_pix
            cte_metres     = cte_pixels * XM_PER_PIX

            dxdy         = 2.0*center_fit[0]*axle_y_pix + center_fit[1]
            path_angle   = math.atan(dxdy)
            path_ang_deg = math.degrees(path_angle)

            if left_fit is not None and right_fit is not None:
                l_x = left_fit[0]*axle_y_pix**2  + left_fit[1]*axle_y_pix  + left_fit[2]
                r_x = right_fit[0]*axle_y_pix**2 + right_fit[1]*axle_y_pix + right_fit[2]
                lane_width_px = abs(r_x - l_x)
            else:
                lane_width_px = 0.0
            lane_width_m = lane_width_px * XM_PER_PIX

            lookahead_row = int(np.clip(
                int(mask.shape[0] * (1.0 - LOOKAHEAD_RATIO)), 0, mask.shape[0] - 1
            ))
            target_x    = int(np.clip(center_x[lookahead_row], 0, self.bev_width - 1))
            car_x       = mask.shape[1] // 2
            car_y       = mask.shape[0] - 1
            pixel_error = target_x - car_x

            cv2.circle(debug, (target_x, lookahead_row), 8, (255, 255, 255), -1)
            cv2.arrowedLine(debug, (car_x, car_y), (target_x, lookahead_row),
                            (0, 255, 255), 4, tipLength=0.25)

            direction = ("TURN RIGHT" if pixel_error > 20
                         else "TURN LEFT" if pixel_error < -20
                         else "STRAIGHT")

            src = "/".join(filter(None, [
                "road" if road_fit  is not None else None,
                "L"    if left_fit  is not None else None,
                "R"    if right_fit is not None else None,
            ])) or "cache"

            hud = [
                (f"CTE      : {cte_metres:+.3f} m  ({cte_pixels:+.0f} px)", 40),
                (f"Path ang : {path_ang_deg:+.2f} deg",                      80),
                (f"Lane wid : {lane_width_m:.2f} m  ({lane_width_px:.0f} px)", 120),
                (f"Lookahead: {pixel_error:+d} px",                          160),
                (direction,                                                   200),
                (f"Signals  : {src}",                                        240),
            ]
            for text, yp in hud:
                cv2.putText(debug, text, (20, yp),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 255), 2)

            # self.get_logger().info(
            #     f"CTE={cte_metres:+.3f}m  path_angle={path_ang_deg:+.2f}deg  "
            #     f"lane_width={lane_width_m:.2f}m  signals={src}"
            # )

            arr = Float64MultiArray()
            arr.data = [cte_metres, path_angle, lane_width_m, 1.0]
            self.lane_pub.publish(arr)

        else:
            self.missing_frames += 1
            label = ("LANE NOT DETECTED" if self.missing_frames <= MAX_MISSING_FRAMES
                     else "LOST – STOPPING")
            cv2.putText(debug, label, (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            self.get_logger().warn(f"No lane – missing_frames={self.missing_frames}")

            detected = 0.0 if self.missing_frames > MAX_MISSING_FRAMES else 1.0
            arr = Float64MultiArray()
            arr.data = [0.0, 0.0, 0.0, detected]
            self.lane_pub.publish(arr)

        cv2.imshow("Lane Following", debug)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()