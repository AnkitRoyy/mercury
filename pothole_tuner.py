#!/usr/bin/env python3
"""
lane_pothole_tuner.py — ROS2 node, live trackbar tuner for BOTH
lane_detection.py and pothole_costmap_node.py logic, using the real
Gazebo camera + TF (map/camera_link/base_link).

For every pothole candidate it prints/overlays WHY it passed or failed
(area/circularity/aspect vs thresholds), and when it's marked, projects
it to map frame via TF and plots it on a live mini top-down costmap.

Run:
  python3 lane_pothole_tuner.py [image_topic]   # default /camera/image_raw
Windows: "Lane+Pothole", "ph_mask", "Costmap (top-down)"
Keys: q=quit | s=print current params | c=clear marked potholes
"""
import sys, math
import cv2
import numpy as np
import rclpy
import rclpy.time, rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf2_ros

WIN, PHWIN, MAPWIN = "Lane+Pothole", "ph_mask", "Costmap (top-down)"

_R_OPT_TO_LINK = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)


def nothing(_):
    pass


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y+z*z),     2*(x*y-z*w),     2*(x*z+y*w)],
        [    2*(x*y+z*w), 1-2*(x*x+z*z),     2*(y*z-x*w)],
        [    2*(x*z-y*w),     2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


class Tuner(Node):
    def __init__(self, topic):
        super().__init__('lane_pothole_tuner')
        self.bridge = CvBridge()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, topic, self.cb, qos)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        self.get_logger().info(f'Subscribed to {topic}')

        self._marked = []      # list of (wx, wy, r) already on the "costmap"
        self._ph_count = 0

        # camera intrinsics (must match camera_hfov in your launch file)
        self._hfov = 1.047

        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        # lane trackbars
        cv2.createTrackbar("L_roi_top_%",  WIN, 35, 100, nothing)
        cv2.createTrackbar("L_v_min",      WIN, 170, 255, nothing)
        cv2.createTrackbar("L_s_max",      WIN, 60, 255, nothing)
        cv2.createTrackbar("L_close_kw",   WIN, 5, 40, nothing)
        cv2.createTrackbar("L_close_kh",   WIN, 25, 60, nothing)
        cv2.createTrackbar("L_canny_lo",   WIN, 30, 255, nothing)
        cv2.createTrackbar("L_canny_hi",   WIN, 100, 255, nothing)
        cv2.createTrackbar("L_hough_th",   WIN, 15, 100, nothing)
        cv2.createTrackbar("L_hough_minl", WIN, 20, 200, nothing)
        cv2.createTrackbar("L_hough_gap",  WIN, 40, 200, nothing)
        cv2.createTrackbar("L_min_sep",    WIN, 120, 400, nothing)
        # pothole trackbars
        cv2.createTrackbar("P_roi_top_%",  WIN, 35, 100, nothing)
        cv2.createTrackbar("P_v_min",      WIN, 130, 255, nothing)
        cv2.createTrackbar("P_s_max",      WIN, 80, 255, nothing)
        cv2.createTrackbar("P_min_area",   WIN, 400, 5000, nothing)
        cv2.createTrackbar("P_min_circx100", WIN, 40, 100, nothing)
        cv2.createTrackbar("P_max_aspx10",   WIN, 22, 100, nothing)

    def g(self, name, scale=1.0):
        return cv2.getTrackbarPos(name, WIN) / scale

    # ── lane helpers (same math as lane_detection.py) ──────────────────
    def white_mask(self, roi, v_min, s_max, kw, kh):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, v_min), (180, s_max, 255))
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, bright = cv2.threshold(gray, v_min, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_and(mask, bright)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (max(1, kw), max(1, kh)))
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

    def line_x(self, segs, roi_h, img_w):
        if not segs:
            return None
        m_s, b_s, w_s = [], [], []
        for x1, y1, x2, y2, length in segs:
            dx = x2 - x1
            if dx == 0:
                continue
            m = (y2 - y1) / dx
            b = y1 - m * x1
            m_s.append(m); b_s.append(b); w_s.append(length)
        if not m_s:
            return None
        tw = sum(w_s)
        m_avg = sum(m*w for m, w in zip(m_s, w_s)) / tw
        b_avg = sum(b*w for b, w in zip(b_s, w_s)) / tw
        if abs(m_avg) < 1e-6:
            return None
        return float(np.clip((roi_h - 1 - b_avg) / m_avg, 0, img_w - 1))

    # ── pothole helpers (same math as pothole_costmap_node.py) ─────────
    def ph_mask(self, roi, v_min, s_max):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, v_min), (180, s_max, 255))
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, bright = cv2.threshold(gray, v_min, 255, cv2.THRESH_BINARY)
        return cv2.bitwise_and(mask, bright)

    def pixel_to_ground(self, u, v, fx, fy, cx, cy, cam_pos, R_cam,
                         robot_pos, robot_fwd):
        ray_opt = np.array([(u-cx)/fx, (v-cy)/fy, 1.0])
        ray_map = R_cam @ (_R_OPT_TO_LINK @ ray_opt)
        if abs(ray_map[2]) < 1e-4:
            return None
        lam = -cam_pos[2] / ray_map[2]
        if lam <= 0.0:
            return None
        wx = cam_pos[0] + lam * ray_map[0]
        wy = cam_pos[1] + lam * ray_map[1]
        d = math.hypot(wx - cam_pos[0], wy - cam_pos[1])
        if d < 0.3 or d > 5.0:
            return None
        if (robot_fwd[0]*(wx-robot_pos[0]) + robot_fwd[1]*(wy-robot_pos[1])) < 0:
            return None
        return wx, wy

    def get_tf(self):
        try:
            ct = self.tf_buf.lookup_transform('map', 'camera_link', rclpy.time.Time(),
                                               timeout=rclpy.duration.Duration(seconds=0.05))
            rt = self.tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time(),
                                               timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException:
            return None
        t = ct.transform.translation
        cam_pos = np.array([t.x, t.y, t.z])
        R_cam = quat_to_rot(ct.transform.rotation)
        rt_t = rt.transform.translation
        robot_pos = np.array([rt_t.x, rt_t.y])
        R_rob = quat_to_rot(rt.transform.rotation)
        robot_fwd = (R_rob @ np.array([1.0, 0.0, 0.0]))[:2]
        return cam_pos, R_cam, robot_pos, robot_fwd

    def draw_costmap(self, robot_pos, robot_fwd):
        S = 360
        scale = 25.0  # px per metre
        canvas = np.full((S, S, 3), 30, np.uint8)
        cx_px, cy_px = S // 2, S - 20
        right = np.array([robot_fwd[1], -robot_fwd[0]])
        for r in (1, 2, 3, 4, 5):
            cv2.circle(canvas, (cx_px, cy_px), int(r*scale), (60, 60, 60), 1)
        pts = np.array([[cx_px, cy_px-10], [cx_px-7, cy_px+8], [cx_px+7, cy_px+8]])
        cv2.fillPoly(canvas, [pts], (0, 255, 0))
        for (wx, wy, r) in self._marked:
            d = np.array([wx, wy]) - robot_pos
            fwd_d, lat_d = float(np.dot(d, robot_fwd)), float(np.dot(d, right))
            px = int(cx_px + lat_d*scale)
            py = int(cy_px - fwd_d*scale)
            if 0 <= px < S and 0 <= py < S:
                cv2.circle(canvas, (px, py), max(2, int(r*scale)), (0, 0, 255), -1)
        cv2.putText(canvas, f'marked={len(self._marked)}', (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.imshow(MAPWIN, canvas)

    # ── main callback ────────────────────────────────────────────────
    def cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')
            return

        fh, fw = frame.shape[:2]
        out = frame.copy()

        # ── LANE ─────────────────────────────────────────────────────
        L_roi_top = self.g("L_roi_top_%", 100.0)
        roi_y = int(fh * L_roi_top)
        roi = frame[roi_y:fh, :]
        white = self.white_mask(roi, int(self.g("L_v_min")), int(self.g("L_s_max")),
                                 int(self.g("L_close_kw")), int(self.g("L_close_kh")))
        edges = cv2.Canny(white, self.g("L_canny_lo"), self.g("L_canny_hi"))
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, int(self.g("L_hough_th")),
                                 minLineLength=self.g("L_hough_minl"),
                                 maxLineGap=self.g("L_hough_gap"))
        img_cx = fw / 2.0
        left_segs, right_segs = [], []
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                if x2 == x1:
                    continue
                length = float(np.hypot(x2-x1, y2-y1))
                (left_segs if (x1+x2)/2 < img_cx else right_segs).append(
                    (x1, y1, x2, y2, length))
        left_x = self.line_x(left_segs, roi.shape[0], fw)
        right_x = self.line_x(right_segs, roi.shape[0], fw)
        cv2.rectangle(out, (0, roi_y), (fw, fh), (0, 80, 0), 1)
        for x1, y1, x2, y2, _ in left_segs:
            cv2.line(out, (x1, y1+roi_y), (x2, y2+roi_y), (255, 100, 0), 2)
        for x1, y1, x2, y2, _ in right_segs:
            cv2.line(out, (x1, y1+roi_y), (x2, y2+roi_y), (0, 100, 255), 2)
        cv2.line(out, (int(img_cx), roi_y), (int(img_cx), fh), (0, 255, 0), 1)

        # ── POTHOLE ──────────────────────────────────────────────────
        P_roi_top = self.g("P_roi_top_%", 100.0)
        p_roi_y = int(fh * P_roi_top)
        p_roi = frame[p_roi_y:fh, :]
        p_roi_h, p_roi_w = p_roi.shape[:2]          # FIX 1: outside loop, correct names
        pmask = self.ph_mask(p_roi, int(self.g("P_v_min")), int(self.g("P_s_max")))
        min_area = self.g("P_min_area")
        min_circ = self.g("P_min_circx100", 100.0)
        max_asp = max(1.0, self.g("P_max_aspx10", 10.0))

        tf = self.get_tf()
        fx = (fw/2.0) / math.tan(self._hfov/2.0)
        fy, cx, cy = fx, fw/2.0, fh/2.0

        contours, _ = cv2.findContours(pmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 20:
                continue
            bx, by, bw, bh = cv2.boundingRect(cnt)
            EDGE_MARGIN = 8
            edge_clip = (bx <= EDGE_MARGIN or by <= EDGE_MARGIN or
                         bx+bw >= p_roi_w-EDGE_MARGIN or by+bh >= p_roi_h-EDGE_MARGIN)# FIX 2: use p_roi_w/h
            if edge_clip:
                cnt_shift = cnt + np.array([[0, p_roi_y]])
                cv2.drawContours(out, [cnt_shift], -1, (0, 165, 255), 1)
                M = cv2.moments(cnt)
                if M['m00'] >= 1.0:
                    ex = int(M['m10']/M['m00']) + 6
                    ey = int(M['m01']/M['m00']) + p_roi_y
                    cv2.putText(out, "edge-clipped,skip", (ex, ey),   # FIX 3: label
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 165, 255), 1)
                continue
            peri = cv2.arcLength(cnt, True)
            if peri < 1.0:
                continue
            circ = (4*math.pi*area) / (peri*peri)
            rect = cv2.minAreaRect(cnt)
            w, h = rect[1]
            if w < 1 or h < 1:
                continue
            aspect = max(w, h) / min(w, h)

            ok_area, ok_circ, ok_asp = area >= min_area, circ >= min_circ, aspect < max_asp
            is_ph = ok_area and ok_circ and ok_asp
            fails = []
            if not ok_area: fails.append(f"area{area:.0f}<{min_area:.0f}")
            if not ok_circ: fails.append(f"circ{circ:.2f}<{min_circ:.2f}(stripe?)")
            if not ok_asp:  fails.append(f"asp{aspect:.2f}>={max_asp:.2f}(stripe?)")
            why = "POTHOLE" if is_ph else "reject:" + ",".join(fails)

            cnt_shift = cnt + np.array([[0, p_roi_y]])
            cv2.drawContours(out, [cnt_shift], -1, (0, 0, 255) if is_ph else (110, 110, 110), 2)
            M = cv2.moments(cnt)
            if M['m00'] < 1.0:
                continue
            ucx = M['m10']/M['m00']
            ucy = M['m01']/M['m00'] + p_roi_y
            cv2.putText(out, why, (int(ucx)+6, int(ucy)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.38, (0, 0, 255) if is_ph else (160, 160, 160), 1)

            if not is_ph:
                continue

            if tf is None:
                cv2.putText(out, "no TF->can't mark costmap", (int(ucx)+6, int(ucy)+14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 165, 255), 1)
                continue

            cam_pos, R_cam, robot_pos, robot_fwd = tf
            center_gnd = self.pixel_to_ground(ucx, ucy, fx, fy, cx, cy,
                                               cam_pos, R_cam, robot_pos, robot_fwd)
            if center_gnd is None:
                cv2.putText(out, "ray miss ground", (int(ucx)+6, int(ucy)+14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 165, 255), 1)
                continue
            wx, wy = center_gnd

            MERGE_R = 1.2  # merge radius — same pothole drifts this much as robot approaches
            nearest_i, nearest_d = None, MERGE_R
            for i, (mx, my, _) in enumerate(self._marked):
                d = math.hypot(wx - mx, wy - my)
                if d < nearest_d:
                    nearest_i, nearest_d = i, d

            r_final = float(np.clip(0.3, 0.20, 1.0))  # quick estimate; tuner-only

            if nearest_i is not None:
                self._marked[nearest_i] = (wx, wy, r_final)
                cv2.putText(out, f"updated ({wx:.1f},{wy:.1f})",
                            (int(ucx)+6, int(ucy)+14), cv2.FONT_HERSHEY_SIMPLEX,
                            0.38, (0, 255, 0), 1)
                continue

            self._marked.append((wx, wy, r_final))
            self._ph_count += 1
            log_why = (f"area={area:.0f}>={min_area:.0f}  "
                       f"circ={circ:.2f}>={min_circ:.2f}  aspect={aspect:.2f}<{max_asp:.2f}")
            self.get_logger().info(
                f'[MARK COSTMAP] #{self._ph_count} map=({wx:.2f},{wy:.2f}) r={r_final:.2f}  '
                f'WHY: {log_why}')
            cv2.putText(out, f"MARKING ({wx:.1f},{wy:.1f})", (int(ucx)+6, int(ucy)+14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        cv2.rectangle(out, (0, p_roi_y), (fw, fh), (0, 0, 120), 1)
        cv2.imshow(WIN, out)
        cv2.imshow(PHWIN, pmask)
        if tf is not None:
            self.draw_costmap(tf[2], tf[3])

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            rclpy.shutdown()
        elif k == ord('c'):
            self._marked.clear()
        elif k == ord('s'):
            print(dict(
                lane=dict(roi_top_frac=L_roi_top, white_v_min=int(self.g("L_v_min")),
                          white_s_max=int(self.g("L_s_max")), close_kw=int(self.g("L_close_kw")),
                          close_kh=int(self.g("L_close_kh")), canny_low=self.g("L_canny_lo"),
                          canny_high=self.g("L_canny_hi"), hough_threshold=int(self.g("L_hough_th")),
                          hough_min_len=self.g("L_hough_minl"), hough_max_gap=self.g("L_hough_gap"),
                          min_lane_sep_px=self.g("L_min_sep")),
                pothole=dict(roi_top_frac=P_roi_top, white_v_min=int(self.g("P_v_min")),
                             white_s_max=int(self.g("P_s_max")), blob_min_area=min_area,
                             blob_min_circularity=min_circ, blob_max_aspect=max_asp)))


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/camera/image_raw'
    rclpy.init()
    node = Tuner(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()