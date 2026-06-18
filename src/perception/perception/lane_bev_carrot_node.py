"""
lane_bev_carrot_node.py  v6
---------------------------
CHANGELOG vs v5
---------------
* Added stuck detection (two-signal: position displacement + BEV streak / LiDAR block)
* Added 3-level recovery state machine (costmap clear → backup → backup+rotate)
* Publishes recovery cmd_vel to /cmd_vel_recovery (twist_mux priority 100)
* Cancels Nav2 /navigate_to_pose action goal on recovery entry
* Publishes /system_alerts on recovery failure

Previous (v5):
* _tick() fallback chain: BEV lane fit → lateral sweep → straight-ahead crawl
* TWO independent obstacle checks per candidate: road_costmap + LaserScan
"""

import math, json, os, time as _time
import numpy as np
import cv2
import rclpy, rclpy.duration, rclpy.time, rclpy.qos
from rclpy.node import Node
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from action_msgs.srv import CancelGoal
from std_srvs.srv import Empty
from ament_index_python.packages import get_package_share_directory
import tf_transformations

_R_OPT = np.array([[0,0,1],[-1,0,0],[0,-1,0]], dtype=np.float64)

def _qrot(q):
    qx,qy,qz,qw = q.x,q.y,q.z,q.w
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),  2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),  1-2*(qx*qx+qz*qz),  2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ], dtype=np.float64)


class LaneBevCarrotNode(Node):

    def __init__(self):
        super().__init__('lane_bev_carrot')

        self.declare_parameter('carrot_dist_m',          2.0)
        self.declare_parameter('goal_tolerance',          0.8)
        self.declare_parameter('publish_rate',            5.0)
        self.declare_parameter('camera_hfov',             1.047)
        self.declare_parameter('image_width',             640)
        self.declare_parameter('image_height',            480)
        self.declare_parameter('min_proj_m',              0.2)
        self.declare_parameter('max_proj_m',              6.0)
        self.declare_parameter('n_bev_samples',           50)
        self.declare_parameter('fit_cache_sec',           1.0)
        self.declare_parameter('no_carrot_stop_streak',   3)   # kept for logging only
        self.declare_parameter('safe_cost_max',           50)
        self.declare_parameter('min_clear_m',             0.6)
        self.declare_parameter('safety_radius',           0.30)
        self.declare_parameter('max_carrot_dist_m',       4.0)
        

        # ── NEW: fallback parameters ───────────────────────────────────────────
        self.declare_parameter('fallback_dists_m',    [1.0, 0.75, 1.5])
        self.declare_parameter('fallback_laterals_m', [0.0, -0.3, 0.3, -0.6, 0.6])
        self.declare_parameter('straight_ahead_dist_m', 0.75)

        # ── Recovery parameters ────────────────────────────────────────────────
        self.declare_parameter('stuck_disp_threshold_m',   0.05)
        self.declare_parameter('stuck_confirm_secs',       3.0)
        self.declare_parameter('stuck_hard_secs',          6.0)
        self.declare_parameter('stuck_streak_threshold',   5)
        self.declare_parameter('recovery_backup_speed',    0.15)
        self.declare_parameter('recovery_rotate_speed',    0.4)
        self.declare_parameter('recovery_nav2_halt_delay', 0.5)
        self.declare_parameter('forward_lidar_arc_deg',    30.0)
        self.declare_parameter('forward_lidar_clear_m',    0.4)

        p = lambda n: self.get_parameter(n).value
        self._carrot_dist     = float(p('carrot_dist_m'))
        self._goal_tol        = float(p('goal_tolerance'))
        rate                  = float(p('publish_rate'))
        hfov                  = float(p('camera_hfov'))
        img_w                 = int(p('image_width'))
        img_h                 = int(p('image_height'))
        self._min_proj        = float(p('min_proj_m'))
        self._max_proj        = float(p('max_proj_m'))
        self._n_samples       = int(p('n_bev_samples'))
        self._fit_cache_sec   = float(p('fit_cache_sec'))
        self._stop_max        = int(p('no_carrot_stop_streak'))
        self._safe_cost_max   = int(p('safe_cost_max'))
        self._min_clear_m     = float(p('min_clear_m'))
        self._safety_r        = float(p('safety_radius'))
        self._max_carrot_dist = float(p('max_carrot_dist_m'))

        self._fallback_dists    = list(p('fallback_dists_m'))
        self._fallback_laterals = list(p('fallback_laterals_m'))
        self._straight_dist     = float(p('straight_ahead_dist_m'))

        # Recovery params
        self._stuck_disp_thr    = float(p('stuck_disp_threshold_m'))
        self._stuck_confirm_s   = float(p('stuck_confirm_secs'))
        self._stuck_hard_s      = float(p('stuck_hard_secs'))
        self._stuck_streak_thr  = int(p('stuck_streak_threshold'))
        self._rec_backup_spd    = float(p('recovery_backup_speed'))
        self._rec_rotate_spd    = float(p('recovery_rotate_speed'))
        self._rec_halt_delay    = float(p('recovery_nav2_halt_delay'))
        self._fwd_arc_deg       = float(p('forward_lidar_arc_deg'))
        self._fwd_clear_m       = float(p('forward_lidar_clear_m'))

        self._fx = (img_w/2.0)/math.tan(hfov/2.0)
        self._cx = img_w/2.0
        self._cy = img_h/2.0

        self._pothole_grid = None
        self._pothole_info = None

        pkg = get_package_share_directory('perception')
        def _load(n): return json.load(open(os.path.join(pkg,'config',n)))
        bev  = _load('bev_config.json')
        road = _load('road_config.json')
        sw   = _load('sliding_window_config.json')

        src = np.float32(bev['src_points'])
        dst = np.float32(bev['dst_points'])
        self._M     = cv2.getPerspectiveTransform(src, dst)
        self._M_inv = cv2.getPerspectiveTransform(dst, src)
        self._bev_w = int(np.max(dst[:,0]))
        self._bev_h = int(np.max(dst[:,1]))
        self._road_v_min = int(road['v_min'])
        self._road_v_max = int(road['v_max'])
        self._road_s_max = int(road.get('s_max',255))
        self._win_h = max(1, int(sw['window_height']))

        self._last_fit_robot = (0.0, 0.0)

        self._carrot_locked  = False
        self._locked_carrot  = None  # (wx, wy)

        # state
        self._final_goal      = None
        self._robot_x = self._robot_y = self._robot_yaw = 0.0
        self._last_img        = None
        self._last_fit        = None
        self._last_fit_stamp  = None
        self._streak          = 0
        self._approach_dir: tuple | None = None   # (dx, dy) unit vec

        # road costmap
        self._road_grid = None
        self._road_info = None

        # laser scan points in map frame
        self._scan_pts_map: np.ndarray | None = None
        self._last_scan_stamp: float = 0.0       # monotonic time of last scan
        self._last_scan_msg: LaserScan | None = None  # raw scan for forward arc check

        # ── Recovery state ─────────────────────────────────────────────────
        self._recovery_active      = False
        self._recovery_state       = 'NORMAL'    # NORMAL|CONFIRMING|LEVEL_1|LEVEL_2|LEVEL_3|FAILED
        self._recovery_level       = 0
        self._stuck_timer_start    = None         # monotonic time when stuck first detected
        self._pos_history: list    = []           # [(x, y, mono_time), ...]
        self._recovery_step_start  = None         # monotonic time for current recovery action
        self._recovery_sub_step    = 0            # sub-step within a level
        self._pre_recovery_pos     = None         # (x, y) at recovery entry for success check

        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)
        self._bridge = CvBridge()

        sq = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        lq = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(PoseStamped,  '/final_goal',                   self._goal_cb,    10)
        self.create_subscription(Odometry,     '/diff_drive_controller/odom',   self._odom_cb,    10)
        self.create_subscription(Image,        '/camera/image_raw',             self._img_cb,     sq)
        self.create_subscription(OccupancyGrid,'/perception/road_costmap',      self._road_cb,    lq)
        self.create_subscription(LaserScan,    '/scan',                         self._scan_cb,    sq)
        self.create_subscription(OccupancyGrid, '/perception/pothole_costmap', self._pothole_cb, lq)

        self._pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # ── Recovery publishers / clients ──────────────────────────────────
        # Recovery publishes directly to /cmd_vel. Safety ensured by
        # cancelling Nav2 action goal on entry — controller_server stops
        # its control loop on cancel, so no cmd_vel conflict.
        self._recovery_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._alert_pub = self.create_publisher(String, '/system_alerts', 10)
        self._cancel_nav2_cli = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self._clear_local_costmap_cli = self.create_client(
            Empty, '/local_costmap/clear_entirely_local_costmap')

        self.create_timer(1.0/rate, self._tick)
        self.get_logger().info(
            f'LaneBevCarrotNode v6 | safe_cost={self._safe_cost_max} '
            f'min_clear={self._min_clear_m}m | '
            f'recovery enabled (3-level, action-cancel)')

    # ── callbacks ──────────────────────────────────────────────────────

    def _goal_cb(self, msg):
        self._final_goal    = msg
        self._streak        = 0
        self._carrot_locked = False
        self._locked_carrot = None
        self._approach_dir  = None   # recomputed on first tick
        # Reset recovery on new goal
        if self._recovery_active:
            self._publish_zero_recovery()
        self._recovery_active    = False
        self._recovery_state     = 'NORMAL'
        self._recovery_level     = 0
        self._stuck_timer_start  = None
        self._pos_history.clear()
        self._recovery_step_start = None
        self._recovery_sub_step  = 0
        self._pre_recovery_pos   = None

    def _odom_cb(self, msg):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _,_,self._robot_yaw = tf_transformations.euler_from_quaternion(
            [q.x,q.y,q.z,q.w])

    def _img_cb(self, msg):
        try: self._last_img = self._bridge.imgmsg_to_cv2(msg,'bgr8')
        except: pass

    def _road_cb(self, msg: OccupancyGrid):
        self._road_info = msg.info
        self._road_grid = msg.data
    
    def _pothole_cb(self, msg: OccupancyGrid):
        self._pothole_info = msg.info
        self._pothole_grid = msg.data

    def _scan_cb(self, msg: LaserScan):
        """Convert scan to map-frame point cloud and cache it."""
        try:
            tf = self._tf_buf.lookup_transform(
                'map', msg.header.frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException:
            return

        R = _qrot(tf.transform.rotation)
        t = tf.transform.translation

        pts = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                p = R @ np.array([x, y, 0.0])
                pts.append((p[0]+t.x, p[1]+t.y))
            angle += msg.angle_increment

        self._scan_pts_map = np.array(pts, dtype=np.float64) if pts else None
        self._last_scan_stamp = _time.monotonic()
        self._last_scan_msg = msg

    # ── safety checks ──────────────────────────────────────────────────

    def _road_cost(self, wx, wy) -> int:
        if self._road_grid is None: return -1
        info = self._road_info
        col = int((wx - info.origin.position.x) / info.resolution)
        row = int((wy - info.origin.position.y) / info.resolution)
        if not (0 <= col < info.width and 0 <= row < info.height): return -1
        return int(self._road_grid[row * info.width + col])
    
    def _pothole_cost(self, wx, wy) -> int:
        if self._pothole_grid is None: return -1
        info = self._pothole_info
        col = int((wx - info.origin.position.x) / info.resolution)
        row = int((wy - info.origin.position.y) / info.resolution)
        if not (0 <= col < info.width and 0 <= row < info.height): return -1
        return int(self._pothole_grid[row * info.width + col])

    def _is_safe(self, wx, wy) -> bool:
        check_pts = [(wx, wy)]
        for deg in (0, 90, 180, 270, 45, 135, 225, 315):
            a = math.radians(deg)
            check_pts.append((wx + self._safety_r * math.cos(a),
                              wy + self._safety_r * math.sin(a)))
        for px, py in check_pts:
            c = self._road_cost(px, py)
            if c != -1 and c >= self._safe_cost_max:
                return False
            
        # --- POTHOLE CHECK (larger radius — hard avoidance) ---
        pothole_r = 0.9  # metres — carrot must stay this far from any pothole cell
        for deg in range(0, 360, 30):
            a = math.radians(deg)
            pc = self._pothole_cost(wx + pothole_r * math.cos(a),
                                    wy + pothole_r * math.sin(a))
            if pc != -1 and pc >= 50:
                return False
        if self._pothole_cost(wx, wy) >= 50:
            return False
        # ------------------------------------------------------
        
        if self._scan_pts_map is not None and len(self._scan_pts_map) > 0:
            dists = np.hypot(self._scan_pts_map[:, 0] - wx,
                             self._scan_pts_map[:, 1] - wy)
            if np.any(dists < self._min_clear_m):
                return False
        return True

    # ── BEV road fit ───────────────────────────────────────────────────

    def _road_fit(self, bev):
        hsv  = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
            np.array([0,0,self._road_v_min]),
            np.array([180,self._road_s_max,self._road_v_max]))
        k = np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        h = mask.shape[0]
        xs,ys = [],[]
        y = h
        while y > 0:
            y0 = max(0, y-self._win_h)
            cols = np.where(np.any(mask[y0:y,:]>0, axis=0))[0]
            if len(cols) >= 2:
                xs.append(int((int(cols[0])+int(cols[-1]))/2))
                ys.append((y0+y)//2)
            y -= self._win_h
        return np.polyfit(ys,xs,2) if len(xs)>=3 else None

    # ── projection ─────────────────────────────────────────────────────

    def _bev_to_ground(self, u_bev, v_bev, cam_pos, R_cam):
        pt = cv2.perspectiveTransform(
            np.array([[[u_bev,v_bev]]],dtype=np.float32), self._M_inv)[0,0]
        ray = R_cam @ (_R_OPT @ np.array(
            [(pt[0]-self._cx)/self._fx,
             (pt[1]-self._cy)/self._fx, 1.0]))
        if ray[2] >= -1e-4: return None
        lam = -cam_pos[2]/ray[2]
        if lam <= 0: return None
        wx = cam_pos[0]+lam*ray[0]; wy = cam_pos[1]+lam*ray[1]
        if not (self._min_proj <= math.hypot(wx-cam_pos[0],wy-cam_pos[1]) <= self._max_proj):
            return None
        return wx, wy

    def _lateral_clearance(self, wx, wy) -> float:
        """Returns min distance (metres) to nearest lethal cell, capped at 2.0m."""
        best = 2.0
        step = 0.1
        for r in np.arange(step, best, step):
            for deg in (0, 45, 90, 135, 180, 225, 270, 315):
                a = math.radians(deg)
                c = self._road_cost(wx + r*math.cos(a), wy + r*math.sin(a))
                if c != -1 and c >= self._safe_cost_max:
                    best = min(best, r)
                    break
        return best

    # ── fallback carrot helpers ────────────────────────────────────────

    def _fallback_carrot(self, rx_map: float, ry_map: float, fwd: np.ndarray, map_yaw: float):
        """
        Sweep a grid of MAP-FRAME points ahead with lateral offsets.
        rx_map, ry_map: robot position in MAP frame (from cam_pos projected to z=0,
                        or base_link TF — NOT self._robot_x/y which is odom-frame).
        """
        lat_x = -math.sin(map_yaw)
        lat_y =  math.cos(map_yaw)

        for dist in self._fallback_dists:
            for lateral in self._fallback_laterals:
                cx = rx_map + dist * fwd[0] + lateral * lat_x
                cy = ry_map + dist * fwd[1] + lateral * lat_y
                if self._is_safe(cx, cy):
                    self.get_logger().info(
                        f'[fallback] carrot ({cx:.2f},{cy:.2f}) '
                        f'dist={dist:.2f}m lat={lateral:+.2f}m')
                    return (cx, cy)
        return None

    def _straight_ahead_carrot(self, rx_map: float, ry_map: float, map_yaw: float) -> tuple:
        """Emergency: unconditionally place carrot straight ahead in MAP frame."""
        cx = rx_map + self._straight_dist * math.cos(map_yaw)
        cy = ry_map + self._straight_dist * math.sin(map_yaw)
        self.get_logger().warn(f'[straight-ahead] emergency carrot ({cx:.2f},{cy:.2f})')
        return (cx, cy)

    # ── main tick ──────────────────────────────────────────────────────

    def _tick(self):
        if self._final_goal is None or self._last_img is None:
            return

        self._update_stuck_detection()   # always runs

        if self._recovery_active:
            self._run_recovery_step()    # handles its own state
            return                       # skip ALL carrot logic

        gx = self._final_goal.pose.position.x
        gy = self._final_goal.pose.position.y
        if math.hypot(gx-self._robot_x, gy-self._robot_y) < self._goal_tol:
            self.get_logger().info('Goal reached!')
            self._final_goal = None; self._streak = 0; return

        try:
            cam_tf = self._tf_buf.lookup_transform(
                'map', 'camera_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException:
            return

        t       = cam_tf.transform.translation
        cam_pos = np.array([t.x, t.y, t.z])
        R_cam   = _qrot(cam_tf.transform.rotation)

        # Robot pose in MAP frame — needed for fwd direction and fallback position.
        # self._robot_yaw is odom-frame; SLAM can rotate map vs odom so we MUST
        # use map->base_link TF for both position and heading.
        try:
            base_tf = self._tf_buf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException:
            return
        bt = base_tf.transform.translation
        rx_map, ry_map = bt.x, bt.y
        bq = base_tf.transform.rotation
        _, _, map_yaw = tf_transformations.euler_from_quaternion(
            [bq.x, bq.y, bq.z, bq.w])
        fwd = np.array([math.cos(map_yaw), math.sin(map_yaw)])

        # ── Finish-line approach direction (computed once per goal) ─────
        if self._approach_dir is None:
            dx = gx - rx_map; dy = gy - ry_map
            d  = math.hypot(dx, dy)
            if d > 0.01:
                self._approach_dir = (dx / d, dy / d)

        # ── Locked carrot (near-goal persistence) ──────────────────────
        if self._carrot_locked and self._locked_carrot is not None:
            cx, cy = self._locked_carrot
            # If locked AT the goal, never invalidate — finish line was crossed
            if cx == gx and cy == gy:
                pass  # fall through to publish below
            elif np.dot(fwd, np.array([cx - cam_pos[0], cy - cam_pos[1]])) <= 0 \
                    or not self._is_safe(cx, cy):
                self.get_logger().info('Locked carrot invalidated — recomputing')
                self._carrot_locked = False
                self._locked_carrot = None
            if self._carrot_locked:  # still locked
                yaw = math.atan2(cy - ry_map, cx - rx_map)
                msg = PoseStamped()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = 'map'
                msg.pose.position.x = cx
                msg.pose.position.y = cy
                msg.pose.orientation.z = math.sin(yaw / 2)
                msg.pose.orientation.w = math.cos(yaw / 2)
                self._pub.publish(msg)
                return

        # ── BEV lane fit ───────────────────────────────────────────────
        bev   = cv2.warpPerspective(self._last_img, self._M, (self._bev_w, self._bev_h))
        fresh = self._road_fit(bev)
        if fresh is not None:
            self._last_fit       = fresh
            self._last_fit_stamp = self.get_clock().now()
            self._last_fit_robot = (self._robot_x, self._robot_y)

        fit = self._last_fit
        if fit is not None and self._last_fit_stamp is not None:
            age   = (self.get_clock().now() - self._last_fit_stamp).nanoseconds / 1e9
            drift = math.hypot(self._robot_x - self._last_fit_robot[0],
                               self._robot_y - self._last_fit_robot[1])
            if age > self._fit_cache_sec or drift > 0.3:
                fit = None

        carrot     = None
        best_score = float('inf')

        if fit is not None:
            for v in np.linspace(self._bev_h-1, 0, self._n_samples):
                u  = float(np.clip(fit[0]*v**2 + fit[1]*v + fit[2], 0, self._bev_w-1))
                pt = self._bev_to_ground(u, v, cam_pos, R_cam)
                if pt is None: continue
                dp          = np.array([pt[0]-rx_map, pt[1]-ry_map])
                dist_to_pt  = math.hypot(*dp)
                dp_from_cam = np.array([pt[0]-cam_pos[0], pt[1]-cam_pos[1]])
                if np.dot(fwd, dp_from_cam) <= 0:     continue
                if dist_to_pt < self._min_proj:        continue
                if dist_to_pt > self._max_carrot_dist: continue
                if not self._is_safe(pt[0], pt[1]):    continue
                clearance = self._lateral_clearance(pt[0], pt[1])
                score = abs(dist_to_pt - self._carrot_dist) - 0.5 * clearance
                if score < best_score:
                    best_score = score; carrot = pt

        # ── Fallback chain when BEV yields nothing ─────────────────────
        if carrot is None:
            self._streak += 1
            self.get_logger().warn(
                f'No BEV carrot (streak={self._streak}) — trying lateral sweep',
                throttle_duration_sec=1.0)

            # Fallback 1: lateral grid search in map frame
            carrot = self._fallback_carrot(rx_map, ry_map, fwd, map_yaw)

            # Fallback 2: guaranteed straight-ahead crawl (suppressed during recovery)
            if carrot is None and not self._recovery_active:
                carrot = self._straight_ahead_carrot(rx_map, ry_map, map_yaw)
            elif carrot is None:
                return  # recovery is active, don't send emergency carrot
        else:
            self._streak = 0

        # ── Finish-line gate: lock carrot at goal if carrot crossed the
        #    perpendicular plane at the goal (dot-product gate, same logic
        #    as goal_decomposer gate planes).
        #    Also triggers on the old distance-based goal_tol check.
        if self._approach_dir is not None:
            adx, ady = self._approach_dir
            # dot( carrot - goal , approach_dir ) >= 0 → carrot is past goal
            past_gate = ((carrot[0] - gx) * adx + (carrot[1] - gy) * ady) >= 0.0
        else:
            past_gate = False

        if past_gate or math.hypot(carrot[0]-gx, carrot[1]-gy) <= self._goal_tol:
            # Snap carrot to exact goal and lock permanently
            carrot = (gx, gy)
            self._carrot_locked  = True
            self._locked_carrot  = carrot
            self.get_logger().info(
                f'Finish line crossed — carrot locked at goal ({gx:.2f},{gy:.2f})')

        # ── Publish carrot ─────────────────────────────────────────────
        dx  = carrot[0] - rx_map
        dy  = carrot[1] - ry_map
        yaw = math.atan2(dy, dx)
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = carrot[0]
        msg.pose.position.y = carrot[1]
        msg.pose.orientation.z = math.sin(yaw/2)
        msg.pose.orientation.w = math.cos(yaw/2)
        self._pub.publish(msg)

    # ── Recovery: stuck detection ──────────────────────────────────────

    def _update_stuck_detection(self):
        """Track position history and detect stuck condition (two-signal)."""
        now = _time.monotonic()
        rx, ry = self._robot_x, self._robot_y

        # Record position every tick, prune old entries
        self._pos_history.append((rx, ry, now))
        self._pos_history = [(x, y, t) for x, y, t in self._pos_history
                             if now - t <= 10.0]

        if self._recovery_active or self._final_goal is None:
            return

        # ── If already in CONFIRMING, just wait for timer ──────────────
        if self._stuck_timer_start is not None:
            # Cancel confirming only if robot moved significantly
            confirm_x, confirm_y = self._confirming_pos
            if math.hypot(rx - confirm_x, ry - confirm_y) > 0.1:
                self.get_logger().info(
                    '[recovery] Robot moved during confirming — cancelling')
                self._stuck_timer_start = None
                self._confirming_pos = None
                self._recovery_state = 'NORMAL'
                return

            if now - self._stuck_timer_start < 1.0:
                return  # still in confirming window

            # ── Confirmed stuck → enter recovery ──────────────────────
            self.get_logger().error(
                f'[recovery] STUCK CONFIRMED at ({rx:.2f},{ry:.2f}) — entering LEVEL_1')
            self._recovery_active = True
            self._recovery_level = 1
            self._recovery_state = 'LEVEL_1'
            self._recovery_step_start = now
            self._recovery_sub_step = 0
            self._pre_recovery_pos = (rx, ry)
            self._confirming_pos = None
            self._cancel_nav2_goal()
            return

        # ── Signal A: position displacement ────────────────────────────
        signal_a = False
        signal_a_hard = False

        # Max displacement across all points in the 3s window
        hist_3s = [(x, y) for x, y, t in self._pos_history
                   if now - t >= self._stuck_confirm_s - 0.2]
        if hist_3s:
            max_disp = max(math.hypot(rx - x, ry - y) for x, y in hist_3s)
            if max_disp < self._stuck_disp_thr:
                signal_a = True

        # Hard stuck: max displacement < 0.05m over 6s → A alone is enough
        hist_6s = [(x, y) for x, y, t in self._pos_history
                   if now - t >= self._stuck_hard_s - 0.2]
        if hist_6s:
            max_disp_hard = max(math.hypot(rx - x, ry - y) for x, y in hist_6s)
            if max_disp_hard < self._stuck_disp_thr:
                signal_a_hard = True

        if not signal_a and not signal_a_hard:
            return

        # ── Signal B: behavioral confirmation ──────────────────────────
        signal_b = False
        if self._streak >= self._stuck_streak_thr:
            signal_b = True
        if not signal_b and self._check_forward_blocked():
            signal_b = True

        # ── Decision ───────────────────────────────────────────────────
        confirmed = (signal_a and signal_b) or signal_a_hard

        if not confirmed:
            return

        # Transition to CONFIRMING (1s extra observation)
        self._stuck_timer_start = now
        self._confirming_pos = (rx, ry)
        self._recovery_state = 'CONFIRMING'
        self.get_logger().warn(
            f'[recovery] STUCK DETECTED — confirming for 1s '
            f'(streak={self._streak}, sigA={signal_a}, sigA_hard={signal_a_hard}, sigB={signal_b})')

    def _check_forward_blocked(self) -> bool:
        """Check if forward LiDAR arc (±arc_deg, < clear_m) is blocked."""
        if self._last_scan_msg is None:
            return False
        # Scan freshness check — stale scan (>1s) is unreliable
        if _time.monotonic() - self._last_scan_stamp > 1.0:
            return False
        msg = self._last_scan_msg
        arc_rad = math.radians(self._fwd_arc_deg)
        angle = msg.angle_min
        blocked_count = 0
        total_in_arc = 0
        for r in msg.ranges:
            # Forward is angle ~0 in base_link frame
            if -arc_rad <= angle <= arc_rad:
                total_in_arc += 1
                if msg.range_min <= r <= msg.range_max and r < self._fwd_clear_m:
                    blocked_count += 1
            angle += msg.angle_increment
        if total_in_arc == 0:
            return False
        # >50% of rays in the arc are blocked
        return (blocked_count / total_in_arc) > 0.5

    # ── Recovery: state machine execution ──────────────────────────────

    def _run_recovery_step(self):
        """Execute the current recovery level. Called from _tick() when active."""
        now = _time.monotonic()
        elapsed = now - self._recovery_step_start if self._recovery_step_start else 0.0

        # ── Check for recovery success at any point ────────────────────
        if self._pre_recovery_pos is not None:
            rx, ry = self._robot_x, self._robot_y
            ox, oy = self._pre_recovery_pos
            if math.hypot(rx - ox, ry - oy) > 0.1:
                self.get_logger().info(
                    f'[recovery] SUCCESS — displaced {math.hypot(rx-ox, ry-oy):.2f}m, '
                    f'returning to NORMAL')
                self._exit_recovery_success()
                return

        if self._recovery_state == 'LEVEL_1':
            self._run_level_1(elapsed)
        elif self._recovery_state == 'LEVEL_2':
            self._run_level_2(elapsed)
        elif self._recovery_state == 'LEVEL_3':
            self._run_level_3(elapsed)
        elif self._recovery_state == 'FAILED':
            self._run_failed()

    def _run_level_1(self, elapsed):
        """LEVEL 1: Clear local costmap + wait 2s (zero physical risk)."""
        now = _time.monotonic()
        if self._recovery_sub_step == 0:
            # Freeze carrot at robot position → Nav2 decelerates
            self._publish_robot_pos_as_carrot()
            self._recovery_sub_step = 1
            self._recovery_step_start = now
            self.get_logger().info('[recovery] L1: freezing carrot at robot pos')
            return

        if self._recovery_sub_step == 1 and elapsed >= self._rec_halt_delay:
            # Clear local costmap
            if self._clear_local_costmap_cli.service_is_ready():
                self._clear_local_costmap_cli.call_async(Empty.Request())
                self.get_logger().info('[recovery] L1: cleared local costmap')
            else:
                self.get_logger().warn('[recovery] L1: local costmap clear service not ready')
            self._recovery_sub_step = 2
            self._recovery_step_start = now
            return

        if self._recovery_sub_step == 2 and elapsed >= 2.0:
            # Check if streak dropped (costmap obstacle was false)
            if self._streak < self._stuck_streak_thr:
                self.get_logger().info('[recovery] L1: streak dropped, costmap clear worked!')
                self._exit_recovery_success()
                return
            # Escalate to LEVEL_2
            self.get_logger().warn('[recovery] L1 failed — escalating to LEVEL_2')
            self._recovery_level = 2
            self._recovery_state = 'LEVEL_2'
            self._recovery_step_start = now
            self._recovery_sub_step = 0

    def _run_level_2(self, elapsed):
        """LEVEL 2: Backup 0.5m (~3.3s at 0.15m/s)."""
        now = _time.monotonic()
        backup_duration = 0.5 / self._rec_backup_spd  # time = dist / speed

        if self._recovery_sub_step == 0:
            # Freeze carrot
            self._publish_robot_pos_as_carrot()
            self._recovery_sub_step = 1
            self._recovery_step_start = now
            self.get_logger().info('[recovery] L2: freezing carrot, waiting for Nav2 halt')
            return

        if self._recovery_sub_step == 1 and elapsed >= self._rec_halt_delay:
            # Start backup via twist_mux /cmd_vel_recovery
            self._recovery_sub_step = 2
            self._recovery_step_start = now
            self.get_logger().info(
                f'[recovery] L2: backing up at {self._rec_backup_spd}m/s '
                f'for {backup_duration:.1f}s')
            return

        if self._recovery_sub_step == 2:
            if elapsed < backup_duration:
                # Publish backup velocity
                cmd = Twist()
                cmd.linear.x = -self._rec_backup_spd
                self._recovery_cmd_pub.publish(cmd)
            else:
                # Stop
                self._publish_zero_recovery()
                self._recovery_sub_step = 3
                self._recovery_step_start = now
                return

        if self._recovery_sub_step == 3 and elapsed >= 0.3:
            # Check displacement
            rx, ry = self._robot_x, self._robot_y
            ox, oy = self._pre_recovery_pos
            if math.hypot(rx - ox, ry - oy) > 0.1:
                self._exit_recovery_success()
                return
            # Escalate to LEVEL_3
            self.get_logger().warn('[recovery] L2 failed — escalating to LEVEL_3')
            self._recovery_level = 3
            self._recovery_state = 'LEVEL_3'
            self._recovery_step_start = now
            self._recovery_sub_step = 0

    def _run_level_3(self, elapsed):
        """LEVEL 3: Backup 0.3m + Rotate 60°."""
        now = _time.monotonic()
        backup_duration = 0.3 / self._rec_backup_spd
        # 60° ≈ 1.047 rad, time = angle / speed
        rotate_duration = 1.047 / self._rec_rotate_spd

        if self._recovery_sub_step == 0:
            # Freeze carrot
            self._publish_robot_pos_as_carrot()
            self._recovery_sub_step = 1
            self._recovery_step_start = now
            self.get_logger().info('[recovery] L3: freezing carrot for halt')
            return

        if self._recovery_sub_step == 1 and elapsed >= self._rec_halt_delay:
            # Start backup
            self._recovery_sub_step = 2
            self._recovery_step_start = now
            self.get_logger().info(f'[recovery] L3: backing up {backup_duration:.1f}s')
            return

        if self._recovery_sub_step == 2:
            if elapsed < backup_duration:
                cmd = Twist()
                cmd.linear.x = -self._rec_backup_spd
                self._recovery_cmd_pub.publish(cmd)
            else:
                self._publish_zero_recovery()
                self._recovery_sub_step = 3
                self._recovery_step_start = now
                return

        if self._recovery_sub_step == 3 and elapsed >= 0.2:
            # Start rotation
            self._recovery_sub_step = 4
            self._recovery_step_start = now
            self.get_logger().info(f'[recovery] L3: rotating {rotate_duration:.1f}s')
            return

        if self._recovery_sub_step == 4:
            if elapsed < rotate_duration:
                cmd = Twist()
                cmd.angular.z = self._rec_rotate_spd
                self._recovery_cmd_pub.publish(cmd)
            else:
                self._publish_zero_recovery()
                self._recovery_sub_step = 5
                self._recovery_step_start = now
                return

        if self._recovery_sub_step == 5 and elapsed >= 0.3:
            # Check displacement
            rx, ry = self._robot_x, self._robot_y
            ox, oy = self._pre_recovery_pos
            if math.hypot(rx - ox, ry - oy) > 0.1:
                self._exit_recovery_success()
                return
            # Escalate to FAILED
            self.get_logger().error('[recovery] L3 failed — RECOVERY EXHAUSTED')
            self._recovery_state = 'FAILED'
            self._recovery_step_start = now
            self._recovery_sub_step = 0

    def _run_failed(self):
        """FAILED: Alert, stop, and require operator intervention."""
        rx, ry = self._robot_x, self._robot_y
        self._publish_zero_recovery()

        alert = json.dumps({
            'event': 'RECOVERY_EXHAUSTED',
            'position': {'x': round(rx, 2), 'y': round(ry, 2)},
            'level_reached': self._recovery_level,
            'streak': self._streak,
        })
        msg = String()
        msg.data = alert
        self._alert_pub.publish(msg)

        self.get_logger().error(
            f'[recovery] RECOVERY EXHAUSTED at ({rx:.2f},{ry:.2f}). '
            f'Republish /final_goal to resume.')

        # Hard stop — don't let carrot keep trying
        self._final_goal = None
        self._recovery_active = False
        self._recovery_state = 'NORMAL'

    # ── Recovery: helpers ──────────────────────────────────────────────

    def _publish_zero_recovery(self):
        """Publish zero velocity on /cmd_vel_recovery."""
        self._recovery_cmd_pub.publish(Twist())

    def _publish_robot_pos_as_carrot(self):
        """Publish current robot position as carrot → Nav2 decelerates to stop."""
        try:
            base_tf = self._tf_buf.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException:
            return
        bt = base_tf.transform.translation
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = bt.x
        msg.pose.position.y = bt.y
        bq = base_tf.transform.rotation
        msg.pose.orientation = bq
        self._pub.publish(msg)

    def _cancel_nav2_goal(self):
        """Cancel all active Nav2 NavigateToPose action goals."""
        if not self._cancel_nav2_cli.service_is_ready():
            self.get_logger().warn('[recovery] Nav2 cancel service not ready')
            return
        req = CancelGoal.Request()
        # Zero goal_id + zero stamp = cancel ALL goals
        self._cancel_nav2_cli.call_async(req)
        self.get_logger().info('[recovery] Sent cancel_all_goals to Nav2')

    def _exit_recovery_success(self):
        """Clean exit from recovery — reset all state."""
        self._publish_zero_recovery()
        # Full state reset
        self._streak            = 0
        self._last_fit          = None
        self._last_fit_stamp    = None
        self._locked_carrot     = None
        self._carrot_locked     = False
        self._approach_dir      = None
        self._recovery_active   = False
        self._recovery_state    = 'NORMAL'
        self._recovery_level    = 0
        self._stuck_timer_start = None
        self._pos_history.clear()
        self._recovery_step_start = None
        self._recovery_sub_step = 0
        self._pre_recovery_pos  = None
        self.get_logger().info('[recovery] State fully reset, resuming normal carrot')


def main(args=None):
    rclpy.init(args=args)
    node = LaneBevCarrotNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()