#include "control/stanley_controller.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <memory>
#include <limits>

namespace control
{

// ── configure ────────────────────────────────────────────────────────────────

void StanleyController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_        = parent;
  plugin_name_ = name;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) throw std::runtime_error("StanleyController: parent node is null");
  logger_ = node->get_logger();
  clock_  = node->get_clock();

  node->declare_parameter(plugin_name_ + ".k_stanley",                k_stanley_);
  node->declare_parameter(plugin_name_ + ".k_soft",                   k_soft_);
  node->declare_parameter(plugin_name_ + ".speed_mps",                speed_mps_);
  node->declare_parameter(plugin_name_ + ".max_steer_deg",            30.0);
  node->declare_parameter(plugin_name_ + ".stop_dist",                stop_dist_);
  node->declare_parameter(plugin_name_ + ".slowdown_dist",            slowdown_dist_);
  node->declare_parameter(plugin_name_ + ".min_speed",                min_speed_);
  node->declare_parameter(plugin_name_ + ".nudge_weight",             nudge_weight_);
  node->declare_parameter(plugin_name_ + ".lateral_offset",           lateral_offset_);
  node->declare_parameter(plugin_name_ + ".sample_dist_near",         sample_dist_near_);
  node->declare_parameter(plugin_name_ + ".sample_dist_far",          sample_dist_far_);
  node->declare_parameter(plugin_name_ + ".sample_slices",            sample_slices_);
  node->declare_parameter(plugin_name_ + ".obstacle_blend_threshold", obstacle_blend_threshold_);
  node->declare_parameter(plugin_name_ + ".door_scan_dist",           door_scan_dist_);
  node->declare_parameter(plugin_name_ + ".door_scan_half_width",     door_scan_half_width_);
  node->declare_parameter(plugin_name_ + ".door_scan_slices",         door_scan_slices_);
  node->declare_parameter(plugin_name_ + ".rotate_in_place_speed",    rotate_in_place_speed_);
  node->declare_parameter(plugin_name_ + ".costmap_warmup_ms",        costmap_warmup_ms_);
  node->declare_parameter(plugin_name_ + ".synthetic_wp_count",       synthetic_wp_count_);
  node->declare_parameter(plugin_name_ + ".synthetic_wp_spacing",     synthetic_wp_spacing_);

  // ── lane-bounded gap/bias params ────────────────────────────────────────
  node->declare_parameter(plugin_name_ + ".lane_margin_m",                 lane_margin_m_);
  node->declare_parameter(plugin_name_ + ".min_gap_width_m",               min_gap_width_m_);
  node->declare_parameter(plugin_name_ + ".door_scan_fallback_half_width", door_scan_fallback_half_width_);
  node->declare_parameter(plugin_name_ + ".obstacle_weight_smoothing",     obstacle_weight_smoothing_);
  node->declare_parameter(plugin_name_ + ".gap_bypass_cte_sign",           gap_bypass_cte_sign_);

  // ── lane width cache params ─────────────────────────────────────────────
  node->declare_parameter(plugin_name_ + ".lane_width_alpha",       lane_width_alpha_);
  node->declare_parameter(plugin_name_ + ".lane_width_min_m",       lane_width_min_m_);
  node->declare_parameter(plugin_name_ + ".lane_width_max_m",       lane_width_max_m_);
  node->declare_parameter(plugin_name_ + ".lane_width_stale_limit", lane_width_stale_limit_);

  // ── curve tracking threshold ────────────────────────────────────────────
  node->declare_parameter(plugin_name_ + ".curve_heading_thresh_deg", 10.0);

  // ── NEW: gap bypass heading gate ────────────────────────────────────────
  // Suppresses gap bypass when the gap centroid direction contradicts the
  // lane heading sign. Prevents wrong-side bypass on curves with obstacles.
  node->declare_parameter(plugin_name_ + ".gap_heading_gate_thresh_deg", 5.0);
  node->declare_parameter(plugin_name_ + ".gap_heading_gate_tol",        gap_heading_gate_tol_);

  // ── NEW: rotate-in-place heading bias ───────────────────────────────────
  // Uses lane_heading sign to choose spin direction when the road is curving,
  // instead of relying on lateral costmap bias which can point the wrong way.
  node->declare_parameter(plugin_name_ + ".rotate_heading_thresh_deg", 5.0);

  k_stanley_              = node->get_parameter(plugin_name_ + ".k_stanley").as_double();
  k_soft_                 = node->get_parameter(plugin_name_ + ".k_soft").as_double();
  speed_mps_              = node->get_parameter(plugin_name_ + ".speed_mps").as_double();
  max_steer_rad_          = node->get_parameter(plugin_name_ + ".max_steer_deg").as_double()
                            * M_PI / 180.0;
  stop_dist_              = node->get_parameter(plugin_name_ + ".stop_dist").as_double();
  slowdown_dist_          = node->get_parameter(plugin_name_ + ".slowdown_dist").as_double();
  min_speed_              = node->get_parameter(plugin_name_ + ".min_speed").as_double();
  nudge_weight_           = node->get_parameter(plugin_name_ + ".nudge_weight").as_double();
  lateral_offset_         = node->get_parameter(plugin_name_ + ".lateral_offset").as_double();
  sample_dist_near_       = node->get_parameter(plugin_name_ + ".sample_dist_near").as_double();
  sample_dist_far_        = node->get_parameter(plugin_name_ + ".sample_dist_far").as_double();
  sample_slices_          = node->get_parameter(plugin_name_ + ".sample_slices").as_int();
  obstacle_blend_threshold_ = node->get_parameter(plugin_name_ + ".obstacle_blend_threshold").as_double();
  door_scan_dist_         = node->get_parameter(plugin_name_ + ".door_scan_dist").as_double();
  door_scan_half_width_   = node->get_parameter(plugin_name_ + ".door_scan_half_width").as_double();
  door_scan_slices_       = node->get_parameter(plugin_name_ + ".door_scan_slices").as_int();
  rotate_in_place_speed_  = node->get_parameter(plugin_name_ + ".rotate_in_place_speed").as_double();
  costmap_warmup_ms_      = node->get_parameter(plugin_name_ + ".costmap_warmup_ms").as_int();
  synthetic_wp_count_     = node->get_parameter(plugin_name_ + ".synthetic_wp_count").as_int();
  synthetic_wp_spacing_   = node->get_parameter(plugin_name_ + ".synthetic_wp_spacing").as_double();

  lane_margin_m_                 = node->get_parameter(plugin_name_ + ".lane_margin_m").as_double();
  min_gap_width_m_               = node->get_parameter(plugin_name_ + ".min_gap_width_m").as_double();
  door_scan_fallback_half_width_ = node->get_parameter(plugin_name_ + ".door_scan_fallback_half_width").as_double();
  obstacle_weight_smoothing_     = node->get_parameter(plugin_name_ + ".obstacle_weight_smoothing").as_double();
  gap_bypass_cte_sign_           = node->get_parameter(plugin_name_ + ".gap_bypass_cte_sign").as_double();

  lane_width_alpha_       = node->get_parameter(plugin_name_ + ".lane_width_alpha").as_double();
  lane_width_min_m_       = node->get_parameter(plugin_name_ + ".lane_width_min_m").as_double();
  lane_width_max_m_       = node->get_parameter(plugin_name_ + ".lane_width_max_m").as_double();
  lane_width_stale_limit_ = node->get_parameter(plugin_name_ + ".lane_width_stale_limit").as_int();

  curve_heading_thresh_rad_ = node->get_parameter(
    plugin_name_ + ".curve_heading_thresh_deg").as_double() * M_PI / 180.0;

  // NEW params
  gap_heading_gate_thresh_rad_ = node->get_parameter(
    plugin_name_ + ".gap_heading_gate_thresh_deg").as_double() * M_PI / 180.0;
  gap_heading_gate_tol_        = node->get_parameter(
    plugin_name_ + ".gap_heading_gate_tol").as_double();
  rotate_heading_thresh_rad_   = node->get_parameter(
    plugin_name_ + ".rotate_heading_thresh_deg").as_double() * M_PI / 180.0;

  lane_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/lane_data_array", 10,
    std::bind(&StanleyController::laneDataCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_,
    "StanleyController configured — "
    "k=%.2f ks=%.2f speed=%.2f "
    "stop=%.2fm slowdown=%.2fm "
    "lat_off=%.2fm near=%.2fm far=%.2fm slices=%d "
    "blend_thresh=%.2f door_scan_dist=%.1fm door_half_w=%.1fm door_slices=%d "
    "rotate_speed=%.2f rad/s warmup=%dms "
    "synthetic_wp_count=%d synthetic_wp_spacing=%.1fm "
    "lane_margin=%.2fm min_gap_width=%.2fm door_fallback_half_w=%.2fm "
    "obs_weight_smoothing=%.2f gap_cte_sign=%.1f "
    "lw_alpha=%.2f lw_min=%.2fm lw_max=%.2fm lw_stale_limit=%d "
    "curve_thresh=%.1f deg "
    "gap_gate_thresh=%.1f deg gap_gate_tol=%.3f rad "
    "rotate_heading_thresh=%.1f deg",
    k_stanley_, k_soft_, speed_mps_,
    stop_dist_, slowdown_dist_,
    lateral_offset_, sample_dist_near_, sample_dist_far_, sample_slices_,
    obstacle_blend_threshold_, door_scan_dist_, door_scan_half_width_, door_scan_slices_,
    rotate_in_place_speed_, costmap_warmup_ms_,
    synthetic_wp_count_, synthetic_wp_spacing_,
    lane_margin_m_, min_gap_width_m_, door_scan_fallback_half_width_,
    obstacle_weight_smoothing_, gap_bypass_cte_sign_,
    lane_width_alpha_, lane_width_min_m_, lane_width_max_m_, lane_width_stale_limit_,
    curve_heading_thresh_rad_ * 180.0 / M_PI,
    gap_heading_gate_thresh_rad_ * 180.0 / M_PI, gap_heading_gate_tol_,
    rotate_heading_thresh_rad_ * 180.0 / M_PI);
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

void StanleyController::cleanup()
{
  lane_sub_.reset();
  costmap_ready_ = false;
  cached_lane_width_m_    = 0.0;
  lane_width_stale_count_ = 0;
  {
    std::lock_guard<std::mutex> lk(synthetic_path_mutex_);
    synthetic_path_.poses.clear();
    waypoint_idx_ = 0;
  }
  RCLCPP_INFO(logger_, "StanleyController cleaned up");
}

void StanleyController::activate()
{
  costmap_ready_ = false;

  if (costmap_warmup_ms_ > 0) {
    RCLCPP_INFO(logger_,
      "StanleyController: waiting %dms for costmap warmup…", costmap_warmup_ms_);
    rclcpp::sleep_for(std::chrono::milliseconds(costmap_warmup_ms_));
  }

  RCLCPP_INFO(logger_, "StanleyController activated");
}

void StanleyController::deactivate()
{
  costmap_ready_ = false;
  RCLCPP_INFO(logger_, "StanleyController deactivated");
}

// ── lane callback ─────────────────────────────────────────────────────────────

void StanleyController::laneDataCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) return;
  std::lock_guard<std::mutex> lock(lane_mutex_);
  cte_metres_     = msg->data[0];
  path_angle_rad_ = msg->data[1];
  lane_width_m_   = msg->data[2];
  lane_detected_  = msg->data[3] > 0.5;
}

// ── setPlan ───────────────────────────────────────────────────────────────────

void StanleyController::setPlan(const nav_msgs::msg::Path & /*path*/)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose)) {
    RCLCPP_WARN(logger_, "setPlan: could not get robot pose — synthetic path NOT rebuilt");
    return;
  }

  rebuildSyntheticPath(robot_pose);
}

// ── rebuildSyntheticPath ──────────────────────────────────────────────────────

void StanleyController::rebuildSyntheticPath(
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  double yaw     = tf2::getYaw(robot_pose.pose.orientation);
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  nav_msgs::msg::Path new_path;
  new_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  new_path.header.stamp    = clock_->now();

  for (int i = 1; i <= synthetic_wp_count_; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.header = new_path.header;
    p.pose.position.x  = robot_pose.pose.position.x + i * synthetic_wp_spacing_ * cos_yaw;
    p.pose.position.y  = robot_pose.pose.position.y + i * synthetic_wp_spacing_ * sin_yaw;
    p.pose.orientation = robot_pose.pose.orientation;
    new_path.poses.push_back(p);
  }

  std::lock_guard<std::mutex> lk(synthetic_path_mutex_);
  synthetic_path_ = std::move(new_path);
  waypoint_idx_   = 0;

  RCLCPP_INFO(logger_,
    "rebuildSyntheticPath: %d waypoints built (%.1fm spacing) ahead of robot yaw=%.1f°",
    synthetic_wp_count_, synthetic_wp_spacing_, yaw * 180.0 / M_PI);
}

// ── advanceToNextClearWaypoint ────────────────────────────────────────────────

geometry_msgs::msg::PoseStamped StanleyController::advanceToNextClearWaypoint(
  const geometry_msgs::msg::PoseStamped & pose,
  double yaw)
{
  std::lock_guard<std::mutex> lk(synthetic_path_mutex_);

  if (synthetic_path_.poses.empty() || waypoint_idx_ >= synthetic_path_.poses.size()) {
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
      "advanceWaypoint: synthetic path exhausted — rebuilding");
    double robot_yaw     = yaw;
    double cos_yaw       = std::cos(robot_yaw);
    double sin_yaw       = std::sin(robot_yaw);
    nav_msgs::msg::Path new_path;
    new_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    new_path.header.stamp    = clock_->now();
    for (int i = 1; i <= synthetic_wp_count_; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = new_path.header;
      p.pose.position.x  = pose.pose.position.x + i * synthetic_wp_spacing_ * cos_yaw;
      p.pose.position.y  = pose.pose.position.y + i * synthetic_wp_spacing_ * sin_yaw;
      p.pose.orientation = pose.pose.orientation;
      new_path.poses.push_back(p);
    }
    synthetic_path_ = std::move(new_path);
    waypoint_idx_   = 0;
  }

  auto * costmap = isCostmapReady() ? costmap_ros_->getCostmap() : nullptr;
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  size_t best_idx = waypoint_idx_;

  for (size_t i = waypoint_idx_; i < synthetic_path_.poses.size(); ++i) {
    const auto & wp = synthetic_path_.poses[i].pose.position;

    double dx  = wp.x - pose.pose.position.x;
    double dy  = wp.y - pose.pose.position.y;
    double dot = dx * cos_yaw + dy * sin_yaw;
    if (dot <= 0.0) {
      waypoint_idx_ = i + 1;
      continue;
    }

    if (costmap) {
      unsigned int mx, my;
      if (costmap->worldToMap(wp.x, wp.y, mx, my)) {
        if (costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          RCLCPP_WARN_THROTTLE(logger_, *clock_, 300,
            "advanceWaypoint: waypoint[%zu] at (%.2f,%.2f) is BLOCKED — skipping",
            i, wp.x, wp.y);
          continue;
        }
      }
    }

    waypoint_idx_ = i;
    best_idx      = i;
    break;
  }

  return synthetic_path_.poses[best_idx];
}

// ── setSpeedLimit ─────────────────────────────────────────────────────────────

void StanleyController::setSpeedLimit(const double & limit, const bool & percentage)
{
  speed_mps_ = percentage ? speed_mps_ * limit / 100.0 : limit;
  RCLCPP_INFO(logger_, "Speed limit set to %.2f m/s", speed_mps_);
}

// ── costmap ready check ───────────────────────────────────────────────────────

bool StanleyController::isCostmapReady()
{
  if (costmap_ready_) return true;
  if (!costmap_ros_)  return false;

  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) return false;

  if (!costmap_ros_->isCurrent()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "Costmap not yet current — obstacle checks skipped, lane-only driving");
    return false;
  }

  costmap_ready_ = true;
  RCLCPP_INFO(logger_, "Costmap is now current — obstacle avoidance active");
  return true;
}

// ── forward obstacle distance ─────────────────────────────────────────────────

double StanleyController::computeForwardObstacleDist(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (!isCostmapReady()) return slowdown_dist_;

  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap)  return slowdown_dist_;

  double res     = costmap->getResolution();
  int    cells   = static_cast<int>(slowdown_dist_ / res);
  double yaw     = tf2::getYaw(pose.pose.orientation);
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  for (int i = 1; i <= cells; ++i) {
    double wx = pose.pose.position.x + i * res * cos_yaw;
    double wy = pose.pose.position.y + i * res * sin_yaw;
    unsigned int cx, cy;
    if (!costmap->worldToMap(wx, wy, cx, cy)) continue;
    if (costmap->getCost(cx, cy) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      return static_cast<double>(i) * res;
  }

  return slowdown_dist_;
}

// ── lateral obstacle bias ─────────────────────────────────────────────────────
//
// effective_lane_width_m is the already-resolved width (cached + smoothed,
// never raw lane_width_m_). It clamps how far we sample to either side of
// center. Without this the bias can pick up cost from well outside the
// actual road and nudge steering toward off-road space.

double StanleyController::computeLateralCostBias(
  const geometry_msgs::msg::PoseStamped & pose,
  double yaw,
  double lane_width_m,           // effective width — already cached/resolved
  double * out_left_cost,
  double * out_right_cost)
{
  if (out_left_cost)  *out_left_cost  = 0.0;
  if (out_right_cost) *out_right_cost = 0.0;

  if (!isCostmapReady()) return 0.0;

  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap)  return 0.0;

  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  double effective_lateral_offset = lateral_offset_;
  if (lane_width_m > 0.1) {
    effective_lateral_offset = std::min(
      lateral_offset_,
      std::max(0.1, lane_width_m / 2.0 - lane_margin_m_));
  }
  // If lane_width_m is still 0 here (cache not yet populated), we keep
  // the raw lateral_offset_ but clamp bias interpretation in step 6 of
  // computeVelocityCommands (bypass suppressed until cache is warm).

  const int    n     = std::max(2, sample_slices_);
  const double d_min = sample_dist_near_;
  const double d_max = sample_dist_far_;
  const double step  = (d_max - d_min) / static_cast<double>(n - 1);

  double weighted_bias = 0.0;
  double total_weight  = 0.0;

  double near_left_cost  = 0.0;
  double near_right_cost = 0.0;

  auto sample_cost = [&](double dist, double lat_sign) -> double {
    double wx = pose.pose.position.x
                + dist     * cos_yaw
                - lat_sign * effective_lateral_offset * sin_yaw;
    double wy = pose.pose.position.y
                + dist     * sin_yaw
                + lat_sign * effective_lateral_offset * cos_yaw;
    unsigned int mx, my;
    if (!costmap->worldToMap(wx, wy, mx, my)) return 0.0;
    return static_cast<double>(costmap->getCost(mx, my));
  };

  for (int i = 0; i < n; ++i) {
    double dist = d_min + i * step;
    double w    = dist / d_max;

    double lc = sample_cost(dist, +1.0);
    double rc = sample_cost(dist, -1.0);

    if (i == 0) {
      near_left_cost  = lc;
      near_right_cost = rc;
    }

    weighted_bias += w * (rc - lc) / 255.0;
    total_weight  += w;
  }

  if (out_left_cost)  *out_left_cost  = near_left_cost;
  if (out_right_cost) *out_right_cost = near_right_cost;

  double bias = (total_weight > 1e-6) ? (weighted_bias / total_weight) : 0.0;

  if (std::abs(bias) > 0.05)
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 300,
      "LateralBias: bias=%+.3f  near_L=%.0f near_R=%.0f  slices=%d  near=%.1fm far=%.1fm  "
      "lat_off=%.2fm  eff_lane_w=%.2fm",
      bias, near_left_cost, near_right_cost, n, d_min, d_max,
      effective_lateral_offset, lane_width_m);

  return bias;
}

// ── gap / doorway detection ───────────────────────────────────────────────────
//
// effective_lane_width_m is the already-resolved width (cached + smoothed).
// It bounds the scan arc to roughly the lane (+ lane_margin_m_). Among free
// runs wide enough to drive through (>= min_gap_width_m_), the one whose
// centroid is closest to straight-ahead is chosen — not simply the widest
// run.

double StanleyController::findGapCentroidOffset(
  const geometry_msgs::msg::PoseStamped & pose,
  double yaw,
  double lane_width_m,           // effective width — already cached/resolved
  bool * out_found)
{
  if (out_found) *out_found = false;

  if (!isCostmapReady()) return 0.0;

  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) return 0.0;

  double effective_half_w;
  if (lane_width_m > 0.1) {
    effective_half_w = std::min(
      door_scan_half_width_,
      std::max(min_gap_width_m_ / 2.0, lane_width_m / 2.0 - lane_margin_m_));
  } else {
    // Cache not yet warm or completely stale.  Use the conservative fallback
    // so we never scan into off-road space on the very first curve.
    effective_half_w = std::min(door_scan_half_width_, door_scan_fallback_half_width_);
  }

  const int n = std::max(3, door_scan_slices_);
  const double half_w = std::max(0.01, effective_half_w);
  const double step = (2.0 * half_w) / static_cast<double>(n - 1);

  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  std::vector<bool> free(n, false);

  for (int i = 0; i < n; ++i) {
    double lat = half_w - i * step;

    double wx = pose.pose.position.x
                + door_scan_dist_ * cos_yaw
                - lat              * sin_yaw;
    double wy = pose.pose.position.y
                + door_scan_dist_ * sin_yaw
                + lat              * cos_yaw;

    unsigned int mx, my;
    if (!costmap->worldToMap(wx, wy, mx, my)) {
      free[i] = false;
      continue;
    }
    unsigned char cost = costmap->getCost(mx, my);
    free[i] = (cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }

  struct Run { int start; int len; };
  std::vector<Run> runs;
  int cur_start = -1, cur_len = 0;
  for (int i = 0; i < n; ++i) {
    if (free[i]) {
      if (cur_len == 0) cur_start = i;
      cur_len++;
    } else {
      if (cur_len > 0) runs.push_back({cur_start, cur_len});
      cur_len = 0;
    }
  }
  if (cur_len > 0) runs.push_back({cur_start, cur_len});

  if (runs.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 500,
      "GapScan: NO free lateral sample found at %.1fm ahead (full %.1fm arc blocked, "
      "half_w=%.2fm)", door_scan_dist_, 2.0 * half_w, half_w);
    return 0.0;
  }

  int    best_start        = -1;
  int    best_len          = 0;
  double best_centroid     = 0.0;
  double best_centroid_abs = std::numeric_limits<double>::infinity();

  for (const auto & r : runs) {
    double width = r.len * step;
    if (width < min_gap_width_m_) continue;

    int    mid_idx      = r.start + (r.len - 1) / 2;
    double centroid_lat = half_w - mid_idx * step;

    if (std::abs(centroid_lat) < best_centroid_abs) {
      best_centroid_abs = std::abs(centroid_lat);
      best_centroid      = centroid_lat;
      best_start         = r.start;
      best_len           = r.len;
    }
  }

  if (best_start < 0) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 500,
      "GapScan: free space found but no run >= min_gap_width_m=%.2fm "
      "within half_w=%.2fm — treating as blocked",
      min_gap_width_m_, half_w);
    return 0.0;
  }

  if (out_found) *out_found = true;

  RCLCPP_INFO_THROTTLE(logger_, *clock_, 300,
    "GapScan: free_run=[idx %d..%d] width=%.2fm centroid=%+.2fm "
    "(scan_dist=%.1fm half_w=%.2fm slices=%d eff_lane_w=%.2fm)",
    best_start, best_start + best_len - 1,
    best_len * step, best_centroid,
    door_scan_dist_, half_w, n, lane_width_m);

  return best_centroid;
}

// ── local bypass: CTE + heading to gap-centroid waypoint ─────────────────────

std::pair<double, double> StanleyController::getGapBypassCteAndHeading(
  const geometry_msgs::msg::PoseStamped & pose,
  double yaw,
  double lane_heading_rad,
  double lane_width_m,
  bool * out_found)
{
  double centroid_lat = findGapCentroidOffset(pose, yaw, lane_width_m, out_found);

  if (!out_found || !*out_found) {
    return {0.0, 0.0};
  }

  const double yaw_weight = 0.7;
  double proj_angle = yaw_weight * yaw + (1.0 - yaw_weight) * lane_heading_rad;

  double cos_p = std::cos(proj_angle);
  double sin_p = std::sin(proj_angle);

  double tx = pose.pose.position.x
              + door_scan_dist_ * cos_p
              - centroid_lat    * std::sin(yaw);
  double ty = pose.pose.position.y
              + door_scan_dist_ * sin_p
              + centroid_lat    * std::cos(yaw);

  double dx = tx - pose.pose.position.x;
  double dy = ty - pose.pose.position.y;

  double cte = gap_bypass_cte_sign_ * (-dx * std::sin(yaw) + dy * std::cos(yaw));

  double target_yaw     = std::atan2(dy, dx);
  double heading_error  = target_yaw - yaw;
  while (heading_error >  M_PI) heading_error -= 2.0 * M_PI;
  while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

  return {cte, heading_error};
}

// ── computeVelocityCommands ───────────────────────────────────────────────────

geometry_msgs::msg::TwistStamped StanleyController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist       & /*velocity*/,
  nav2_core::GoalChecker                * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();

  // ── 1. lane state ─────────────────────────────────────────────────────
  double lane_cte, lane_heading, raw_lane_width;
  bool   detected;
  {
    std::lock_guard<std::mutex> lock(lane_mutex_);
    lane_cte       = cte_metres_;
    lane_heading   = path_angle_rad_;
    raw_lane_width = lane_width_m_;
    detected       = lane_detected_;
  }

  if (!detected) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "Lane not detected — stopping");
    return cmd;
  }

  // ── 1b. lane width cache — smooth, bound, and persist across frames ───
  //
  // raw_lane_width is 0.0 whenever the perception node only has road_fit
  // (no bilateral lane lines) — exactly on curves. We cache the last valid
  // bilateral reading with a low-pass filter and sanity bounds so that
  // findGapCentroidOffset / computeLateralCostBias always have a usable
  // estimate to bound their scan arcs, preventing them from looking into
  // off-road space during the frames where bilateral detections are absent.
  //
  // Stale guard: if no valid bilateral reading has arrived for
  // lane_width_stale_limit_ consecutive frames, the cache is cleared so
  // the fallback path in gap/bias functions activates rather than letting
  // a very old width estimate persist indefinitely.
  //
  // The resulting effective_lane_width_m is what gap/bias functions see.
  // It is NEVER the raw per-frame value.

  if (raw_lane_width >= lane_width_min_m_ && raw_lane_width <= lane_width_max_m_) {
    // Valid bilateral reading this frame.
    lane_width_stale_count_ = 0;
    if (cached_lane_width_m_ < lane_width_min_m_) {
      // Cold start: accept the first in-bounds reading directly so the cache
      // isn't sluggish to initialise (alpha=0.3 would need ~10 frames to get
      // even halfway to the true value from 0).
      cached_lane_width_m_ = raw_lane_width;
      RCLCPP_INFO(logger_,
        "LaneWidthCache: cold-start init %.2fm", cached_lane_width_m_);
    } else {
      // Steady-state: low-pass filter.
      cached_lane_width_m_ = lane_width_alpha_ * raw_lane_width
                           + (1.0 - lane_width_alpha_) * cached_lane_width_m_;
    }
  } else {
    // No valid bilateral reading this frame (curve, or out-of-bounds spike).
    lane_width_stale_count_++;
    if (lane_width_stale_count_ >= lane_width_stale_limit_) {
      if (cached_lane_width_m_ > 0.0) {
        RCLCPP_WARN(logger_,
          "LaneWidthCache: stale for %d frames — clearing (was %.2fm)",
          lane_width_stale_count_, cached_lane_width_m_);
        cached_lane_width_m_ = 0.0;
      }
    }
  }

  // effective_lane_width_m is what every downstream function uses.
  // 0.0 means "not yet warm" — gap/bias functions fall back to their
  // conservative fallback paths when they see this.
  const double effective_lane_width_m = cached_lane_width_m_;

  RCLCPP_DEBUG(logger_,
    "LaneWidthCache: raw=%.2fm cached=%.2fm stale=%d",
    raw_lane_width, effective_lane_width_m, lane_width_stale_count_);

  // ── 1c. advance synthetic waypoint ───────────────────────────────────
  double yaw = tf2::getYaw(pose.pose.orientation);
  auto   active_wp = advanceToNextClearWaypoint(pose, yaw);

  RCLCPP_DEBUG(logger_,
    "ActiveWP: [%.2f, %.2f]  idx=%zu/%d",
    active_wp.pose.position.x, active_wp.pose.position.y,
    waypoint_idx_, synthetic_wp_count_);

  // ── 2. forward obstacle distance ──────────────────────────────────────
  double fwd_dist = computeForwardObstacleDist(pose);

  // ── 3. lateral lookahead bias ─────────────────────────────────────────
  double near_left_cost = 0.0, near_right_cost = 0.0;
  double bias = computeLateralCostBias(
    pose, yaw, effective_lane_width_m, &near_left_cost, &near_right_cost);

  // ── 4. ROTATE-IN-PLACE when path is blocked ───────────────────────────
  if (fwd_dist <= stop_dist_) {
    double spin_dir = 0.0;

    // ── NEW: heading-biased spin direction ────────────────────────────
    // When the road is curving (|lane_heading| >= rotate_heading_thresh_rad_),
    // use the lane heading sign to determine spin direction. This ensures the
    // robot rotates toward the direction the road is actually bending rather
    // than toward whichever side of the road happens to have lower costmap cost
    // (which can be the wrong side when an obstacle blocks the correct curve
    // direction).
    //
    // Example: right curve with obstacle on right → costmap bias says "spin left"
    // (less cost on left) → robot spins off-road.
    // With this fix: lane_heading > 0 → spin right → robot follows the curve.
    if (std::abs(lane_heading) >= rotate_heading_thresh_rad_) {
      spin_dir = (lane_heading > 0.0) ? +1.0 : -1.0;
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 300,
        "ROTATE-IN-PLACE (heading-biased): fwd=%.2fm <= stop=%.2fm  "
        "lane_heading=%+.1f deg → spin=%s  omega=%.2f rad/s",
        fwd_dist, stop_dist_,
        lane_heading * 180.0 / M_PI,
        spin_dir > 0 ? "LEFT" : "RIGHT",
        spin_dir * rotate_in_place_speed_);
    } else if (std::abs(bias) > 1e-3) {
      // Road is nearly straight: use lateral costmap bias as before.
      spin_dir = (bias > 0.0) ? +1.0 : -1.0;
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 300,
        "ROTATE-IN-PLACE (bias): fwd=%.2fm <= stop=%.2fm  "
        "bias=%+.3f  spin=%s  omega=%.2f rad/s",
        fwd_dist, stop_dist_,
        bias,
        spin_dir > 0 ? "LEFT" : "RIGHT",
        spin_dir * rotate_in_place_speed_);
    } else {
      // Fallback: compare near costmap costs.
      spin_dir = (near_right_cost >= near_left_cost) ? +1.0 : -1.0;
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 300,
        "ROTATE-IN-PLACE (cost): fwd=%.2fm <= stop=%.2fm  "
        "near_L=%.0f near_R=%.0f  spin=%s  omega=%.2f rad/s",
        fwd_dist, stop_dist_,
        near_left_cost, near_right_cost,
        spin_dir > 0 ? "LEFT" : "RIGHT",
        spin_dir * rotate_in_place_speed_);
    }

    cmd.twist.linear.x  = 0.0;
    cmd.twist.angular.z = spin_dir * rotate_in_place_speed_;
    return cmd;
  }

  // ── 5. forward speed scale ────────────────────────────────────────────
  double scale = 1.0;
  if (fwd_dist < slowdown_dist_) {
    double t = (fwd_dist - stop_dist_) / (slowdown_dist_ - stop_dist_);
    scale = min_speed_ + std::clamp(t, 0.0, 1.0) * (1.0 - min_speed_);
  }

  // ── 6. CURVE vs STRAIGHT mode selection ──────────────────────────────
  //
  // On a curve (|lane_heading| >= curve_heading_thresh_rad_):
  //   delta = lane_heading only — pure curvature tracking.
  //   No CTE correction, no gap bypass, no lateral bias.
  //   The robot rotates at the angle the road curves, staying in place
  //   on the road surface rather than being pulled toward a lane marking
  //   or steered toward off-road space by a spurious obstacle signal.
  //
  // On a straight (|lane_heading| < curve_heading_thresh_rad_):
  //   Full Stanley law (CTE + heading) + gap-bypass if an obstacle is
  //   actually blocking forward travel.  This is unchanged from before.
  //
  // curve_heading_thresh_rad_ is tunable via param "curve_heading_thresh_deg"
  // (default 10°).  Raise it if gentle bends still trigger gap bypass;
  // lower it if the robot is slow to correct lateral drift on near-straights.

  const bool on_curve = (std::abs(lane_heading) >= curve_heading_thresh_rad_);

  double delta     = 0.0;
  double final_cte = lane_cte;
  const char * mode_str = "LANE_ONLY";

  if (on_curve) {
    // ── CURVE: track curvature angle, nothing else ─────────────────────
    // Reset obstacle smoothing so it doesn't bleed into the next straight.
    prev_obstacle_weight_ = 0.0;

    delta = std::clamp(lane_heading, -max_steer_rad_, max_steer_rad_);

    RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
      "CURVE_TRACK: heading=%+.2f deg  delta=%+.2f deg  "
      "fwd=%.2fm  scale=%.2f  eff_lane_w=%.2fm",
      lane_heading * 180.0 / M_PI,
      delta        * 180.0 / M_PI,
      fwd_dist, scale, effective_lane_width_m);

  } else {
    // ── STRAIGHT: full Stanley + optional gap bypass ───────────────────
    //
    // Gap bypass is also suppressed when the lane-width cache has not yet
    // been populated — without a valid width, the scan arcs are unguarded
    // and can see off-road space.
    const bool width_cache_warm = (effective_lane_width_m > 0.1);

    double raw_obs_weight = 0.0;
    if (width_cache_warm) {
      raw_obs_weight = std::min(std::abs(bias) / obstacle_blend_threshold_, 1.0);
    } else {
      RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
        "BypassMode suppressed: lane-width cache not yet warm "
        "(raw=%.2fm, stale=%d) — lane-only until a bilateral width is seen",
        raw_lane_width, lane_width_stale_count_);
    }

    double path_cte     = lane_cte;
    double path_heading = lane_heading;

    if (raw_obs_weight > 0.05) {
      bool gap_found = false;
      auto [gc, gh] = getGapBypassCteAndHeading(
        pose, yaw, lane_heading, effective_lane_width_m, &gap_found);

      if (gap_found) {
        // ── NEW: heading gate — reject bypass if gap direction contradicts lane ──
        //
        // Problem: on a curve with an obstacle, the gap scanner can find free
        // space on the WRONG side of the road (e.g. off-road grass to the left
        // when the robot needs to turn right). Without this gate, bypass steers
        // toward the wrong gap.
        //
        // Fix: when lane_heading is strong enough to indicate a curve direction,
        // only accept a gap whose CTE sign agrees with the lane heading sign.
        //   lane_heading > 0 → road curves right → gap must be to the right (gc > 0)
        //   lane_heading < 0 → road curves left  → gap must be to the left  (gc < 0)
        // Near-zero lane_heading (straight road) → accept either side.
        //
        // gap_heading_gate_thresh_deg (default 5°): minimum lane_heading magnitude
        //   to activate the gate. Tune: raise if bypass is over-rejected on mild
        //   bends; lower if wrong-side bypass still fires.
        // gap_heading_gate_tol (default 0.05 rad): gaps within this of zero are
        //   considered "centred" and always accepted regardless of heading sign.

        bool heading_gate_pass = true;
        if (std::abs(lane_heading) >= gap_heading_gate_thresh_rad_) {
          // Gate is active: gap centroid must agree with lane heading direction.
          const bool same_side = (lane_heading * gc > 0.0);
          const bool gap_centred = (std::abs(gc) < gap_heading_gate_tol_);
          heading_gate_pass = same_side || gap_centred;

          if (!heading_gate_pass) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 500,
              "GapBypass HEADING-GATE REJECTED: "
              "lane_heading=%+.1f deg  gap_cte=%+.3f (wrong side) — "
              "holding lane-only this cycle",
              lane_heading * 180.0 / M_PI, gc);
            raw_obs_weight = 0.0;
          }
        }

        if (heading_gate_pass) {
          path_cte     = gc;
          path_heading = gh;
          mode_str     = "GAP_BYPASS";

          RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
            "BypassMode: gap-centroid target %.1fm ahead  obs_w=%.2f  bias=%+.3f  "
            "gap_cte=%+.3f  lane_heading=%+.1f deg  eff_lane_w=%.2fm",
            door_scan_dist_, raw_obs_weight, bias,
            gc, lane_heading * 180.0 / M_PI, effective_lane_width_m);
        }
      } else {
        raw_obs_weight = 0.0;
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 500,
          "BypassMode requested (bias=%+.3f) but no free gap found — "
          "holding lane-following this cycle", bias);
      }
    }

    // Blend lane ↔ gap-bypass with exponential smoothing.
    double obstacle_weight = obstacle_weight_smoothing_ * raw_obs_weight
                            + (1.0 - obstacle_weight_smoothing_) * prev_obstacle_weight_;
    prev_obstacle_weight_ = obstacle_weight;

    final_cte       = obstacle_weight * path_cte
                    + (1.0 - obstacle_weight) * lane_cte;
    double path_ang = obstacle_weight * path_heading
                    + (1.0 - obstacle_weight) * lane_heading;

    double cte_term = std::atan2(k_stanley_ * final_cte, speed_mps_ + k_soft_);
    delta = std::clamp(path_ang + cte_term, -max_steer_rad_, max_steer_rad_);

    RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
      "STRAIGHT: obs_w=%.2f (raw=%.2f)  lane_cte=%+.3f path_cte=%+.3f  "
      "final_cte=%+.3f  bias=%+.3f  fwd=%.2fm  scale=%.2f  costmap=%s  mode=%s  "
      "eff_lane_w=%.2fm (raw=%.2fm stale=%d)",
      obstacle_weight, raw_obs_weight, lane_cte, path_cte, final_cte, bias,
      fwd_dist, scale,
      costmap_ready_ ? "ready" : "warming",
      mode_str,
      effective_lane_width_m, raw_lane_width, lane_width_stale_count_);
  }

  // ── 7. emit command ───────────────────────────────────────────────────
  cmd.twist.linear.x  = speed_mps_ * scale;
  cmd.twist.angular.z = delta;

  RCLCPP_DEBUG(logger_,
    "CMD: mode=%s  cte=%+.3f  delta=%+.1fdeg  scale=%.2f",
    on_curve ? "CURVE" : mode_str,
    final_cte,
    delta * 180.0 / M_PI,
    scale);

  return cmd;
}

}  // namespace control

PLUGINLIB_EXPORT_CLASS(control::StanleyController, nav2_core::Controller)