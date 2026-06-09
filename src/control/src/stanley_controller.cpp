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
  node->declare_parameter(plugin_name_ + ".sample_dist",              sample_dist_);
  node->declare_parameter(plugin_name_ + ".obstacle_blend_threshold", obstacle_blend_threshold_);
  node->declare_parameter(plugin_name_ + ".costmap_warmup_ms",        costmap_warmup_ms_);

  k_stanley_               = node->get_parameter(plugin_name_ + ".k_stanley").as_double();
  k_soft_                  = node->get_parameter(plugin_name_ + ".k_soft").as_double();
  speed_mps_               = node->get_parameter(plugin_name_ + ".speed_mps").as_double();
  max_steer_rad_           = node->get_parameter(plugin_name_ + ".max_steer_deg").as_double()
                             * M_PI / 180.0;
  stop_dist_               = node->get_parameter(plugin_name_ + ".stop_dist").as_double();
  slowdown_dist_           = node->get_parameter(plugin_name_ + ".slowdown_dist").as_double();
  min_speed_               = node->get_parameter(plugin_name_ + ".min_speed").as_double();
  nudge_weight_            = node->get_parameter(plugin_name_ + ".nudge_weight").as_double();
  lateral_offset_          = node->get_parameter(plugin_name_ + ".lateral_offset").as_double();
  sample_dist_             = node->get_parameter(plugin_name_ + ".sample_dist").as_double();
  obstacle_blend_threshold_= node->get_parameter(plugin_name_ + ".obstacle_blend_threshold").as_double();
  costmap_warmup_ms_       = node->get_parameter(plugin_name_ + ".costmap_warmup_ms").as_int();

  lane_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/lane_data_array", 10,
    std::bind(&StanleyController::laneDataCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_,
    "StanleyController configured — "
    "k=%.2f ks=%.2f speed=%.2f "
    "stop=%.2fm slowdown=%.2fm "
    "nudge=%.2f lat_off=%.2fm sample=%.2fm "
    "blend_thresh=%.2f warmup=%dms",
    k_stanley_, k_soft_, speed_mps_,
    stop_dist_, slowdown_dist_,
    nudge_weight_, lateral_offset_, sample_dist_,
    obstacle_blend_threshold_, costmap_warmup_ms_);
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

void StanleyController::cleanup()
{
  lane_sub_.reset();
  costmap_ready_ = false;
  RCLCPP_INFO(logger_, "StanleyController cleaned up");
}

void StanleyController::activate()
{
  costmap_ready_ = false;

  // Sleep briefly so the costmap has time to receive its first sensor update
  // before we start commanding. This avoids the race condition where
  // computeVelocityCommands fires before any scan has been processed.
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
  lane_detected_  = msg->data[3] > 0.5;
}

// ── setPlan ───────────────────────────────────────────────────────────────────
//
// Nav2 calls this every time A* produces a new global path.
// We store it so computeVelocityCommands can blend it in when
// the lane would steer the robot toward an obstacle.

void StanleyController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  current_path_ = path;
  RCLCPP_INFO(logger_, "setPlan: stored %zu waypoints from A* planner",
    path.poses.size());
}

// ── setSpeedLimit ─────────────────────────────────────────────────────────────

void StanleyController::setSpeedLimit(const double & limit, const bool & percentage)
{
  speed_mps_ = percentage ? speed_mps_ * limit / 100.0 : limit;
  RCLCPP_INFO(logger_, "Speed limit set to %.2f m/s", speed_mps_);
}

// ── costmap ready check ───────────────────────────────────────────────────────
//
// Returns true once the costmap has received at least one sensor update.
// Logs once when it first becomes ready so timing is visible in logs.

bool StanleyController::isCostmapReady()
{
  if (costmap_ready_) return true;  // fast path once warmed up

  if (!costmap_ros_) return false;

  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) return false;

  if (!costmap_ros_->isCurrent()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
      "Costmap not yet current — obstacle checks skipped, lane-only driving");
    return false;
  }

  // First time we see it as current — log and latch
  costmap_ready_ = true;
  RCLCPP_INFO(logger_, "Costmap is now current — obstacle avoidance active");
  return true;
}

// ── A* path CTE + heading ─────────────────────────────────────────────────────
//
// Finds the nearest waypoint on the stored A* path, then looks ahead
// a few steps to compute:
//   cte           – signed lateral error (m), + = robot left of path → steer right
//   heading_error – angle between robot heading and path tangent (rad)

std::pair<double, double> StanleyController::getPathCteAndHeading(
  const geometry_msgs::msg::PoseStamped & pose)
{
  std::lock_guard<std::mutex> lock(path_mutex_);
  if (current_path_.poses.empty()) return {0.0, 0.0};

  const double robot_x   = pose.pose.position.x;
  const double robot_y   = pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(pose.pose.orientation);

  // ── 1. find nearest waypoint ───────────────────────────────────────────
  size_t nearest  = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < current_path_.poses.size(); ++i)
  {
    double dx = current_path_.poses[i].pose.position.x - robot_x;
    double dy = current_path_.poses[i].pose.position.y - robot_y;
    double d  = std::hypot(dx, dy);
    if (d < min_dist) { min_dist = d; nearest = i; }
  }

  // ── 2. lookahead point (Stanley front-axle reference) ─────────────────
  // +3 steps gives a small lookahead; increase if path is dense
  const size_t lookahead_steps = 3;
  size_t target = std::min(nearest + lookahead_steps,
                           current_path_.poses.size() - 1);

  const double tx = current_path_.poses[target].pose.position.x;
  const double ty = current_path_.poses[target].pose.position.y;

  // ── 3. CTE: signed lateral distance robot → lookahead point ───────────
  //   positive → path is to the LEFT  of robot → Stanley steers right
  //   negative → path is to the RIGHT of robot → Stanley steers left
  double dx  = tx - robot_x;
  double dy  = ty - robot_y;
  double cte = -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);

  // ── 4. heading error: path tangent minus robot yaw ────────────────────
  const double px_near = current_path_.poses[nearest].pose.position.x;
  const double py_near = current_path_.poses[nearest].pose.position.y;
  double path_yaw      = std::atan2(ty - py_near, tx - px_near);

  double heading_error = path_yaw - robot_yaw;
  // normalise to [-π, π]
  while (heading_error >  M_PI) heading_error -= 2.0 * M_PI;
  while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

  return {cte, heading_error};
}

// ── forward obstacle: speed scale ────────────────────────────────────────────
//
// Scans costmap cells along the robot heading out to slowdown_dist_.
// Returns 0.0 (hard stop) → 1.0 (full speed).
// Returns 1.0 immediately if costmap is not yet ready.

double StanleyController::computeObstacleSpeedScale(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (!isCostmapReady()) return 1.0;  // costmap not ready → full speed, no false stops

  auto * costmap  = costmap_ros_->getCostmap();
  if (!costmap)   return 1.0;

  double res      = costmap->getResolution();
  int    cells    = static_cast<int>(slowdown_dist_ / res);
  double min_dist = slowdown_dist_;

  double yaw     = tf2::getYaw(pose.pose.orientation);
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  for (int i = 1; i <= cells; ++i)
  {
    double wx = pose.pose.position.x + i * res * cos_yaw;
    double wy = pose.pose.position.y + i * res * sin_yaw;
    unsigned int cx, cy;
    if (!costmap->worldToMap(wx, wy, cx, cy)) continue;
    if (costmap->getCost(cx, cy) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      min_dist = std::min(min_dist, static_cast<double>(i) * res);
      break;
    }
  }

  if (min_dist <= stop_dist_)     return 0.0;
  if (min_dist >= slowdown_dist_) return 1.0;

  double t = (min_dist - stop_dist_) / (slowdown_dist_ - stop_dist_);
  return min_speed_ + t * (1.0 - min_speed_);
}

// ── lateral obstacle: cost bias ───────────────────────────────────────────────
//
// Samples two arcs (near + far) at ±lateral_offset_ left/right of heading.
// Returns normalised bias in [-1, +1]:
//   +1 → heavy cost on right → A* path weight increases (steer away from right)
//   -1 → heavy cost on left  → A* path weight increases (steer away from left)
// Returns 0.0 immediately if costmap is not yet ready.

double StanleyController::computeLateralCostBias(
  const geometry_msgs::msg::PoseStamped & pose, double yaw)
{
  if (!isCostmapReady()) return 0.0;  // costmap not ready → no bias, pure lane

  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap)  return 0.0;

  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  const double dist_near = sample_dist_ * 0.5;
  const double dist_far  = sample_dist_;
  const double w_near    = 0.4;
  const double w_far     = 0.6;

  auto sample_cost = [&](double dist, double lat_sign) -> double {
    double wx = pose.pose.position.x
                + dist     * cos_yaw
                - lat_sign * lateral_offset_ * sin_yaw;
    double wy = pose.pose.position.y
                + dist     * sin_yaw
                + lat_sign * lateral_offset_ * cos_yaw;
    unsigned int mx, my;
    if (!costmap->worldToMap(wx, wy, mx, my)) return 0.0;
    return static_cast<double>(costmap->getCost(mx, my));
  };

  // left = lat_sign +1,  right = lat_sign -1
  double left_cost  = w_near * sample_cost(dist_near, +1.0)
                    + w_far  * sample_cost(dist_far,  +1.0);
  double right_cost = w_near * sample_cost(dist_near, -1.0)
                    + w_far  * sample_cost(dist_far,  -1.0);

  // normalise to [-1, +1]  (max weighted cost ≈ 255)
  double bias = (right_cost - left_cost) / 255.0;

  if (std::abs(bias) > 0.05)
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 300,
      "LateralBias: left=%.0f right=%.0f bias=%+.3f  A*_weight=%.2f",
      left_cost, right_cost, bias,
      std::min(std::abs(bias) / obstacle_blend_threshold_, 1.0));

  return bias;
}

// ── computeVelocityCommands ───────────────────────────────────────────────────
//
// Blending strategy
// -----------------
// obstacle_weight = clamp(|bias| / obstacle_blend_threshold, 0, 1)
//
//   obstacle_weight = 0  → pure lane following   (no obstacle / costmap not ready)
//   obstacle_weight = 1  → pure A* path following (obstacle fully detected)
//
// While costmap is warming up the robot drives purely on lane data at full
// speed — it will not false-stop and will not make erratic avoidance moves.
// Once isCurrent() latches true, obstacle avoidance activates automatically.

geometry_msgs::msg::TwistStamped StanleyController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist       & /*velocity*/,
  nav2_core::GoalChecker                * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = clock_->now();

  // ── 1. lane state ─────────────────────────────────────────────────────
  double lane_cte, lane_heading;
  bool   detected;
  {
    std::lock_guard<std::mutex> lock(lane_mutex_);
    lane_cte     = cte_metres_;
    lane_heading = path_angle_rad_;
    detected     = lane_detected_;
  }

  if (!detected)
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "Lane not detected — stopping");
    return cmd;
  }

  // ── 2. forward obstacle check ─────────────────────────────────────────
  // Returns 1.0 (no slowdown) when costmap is not yet ready
  double scale = computeObstacleSpeedScale(pose);
  if (scale < 0.01)
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 500,
      "Obstacle within stop distance — hard stop");
    return cmd;
  }

  // ── 3. lateral obstacle bias ──────────────────────────────────────────
  // Returns 0.0 (no bias) when costmap is not yet ready
  double yaw  = tf2::getYaw(pose.pose.orientation);
  double bias = computeLateralCostBias(pose, yaw);

  // ── 4. A* path CTE + heading ──────────────────────────────────────────
  auto [path_cte, path_heading] = getPathCteAndHeading(pose);

  // ── 5. blend lane ↔ A* based on obstacle severity ────────────────────
  //
  // obstacle_weight ramps 0→1 as |bias| grows from 0 → obstacle_blend_threshold.
  // At weight=1 the robot ignores the lane entirely and follows A*.
  // When costmap is not ready, bias=0 so obstacle_weight=0 → pure lane.
  double obstacle_weight = std::min(
    std::abs(bias) / obstacle_blend_threshold_, 1.0);

  double cte      = obstacle_weight * path_cte
                  + (1.0 - obstacle_weight) * lane_cte;
  double path_ang = obstacle_weight * path_heading
                  + (1.0 - obstacle_weight) * lane_heading;

  RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
    "Blend: obs_w=%.2f  lane_cte=%+.3f path_cte=%+.3f  "
    "final_cte=%+.3f  bias=%+.3f  costmap=%s",
    obstacle_weight, lane_cte, path_cte, cte, bias,
    costmap_ready_ ? "ready" : "warming");

  // ── 6. Stanley law ────────────────────────────────────────────────────
  //
  // Sign convention (ROS REP-103):
  //   cte > 0      → robot left of path   → steer right → angular.z < 0
  //   path_ang > 0 → path curves left     → steer left  → angular.z > 0
  //   angular.z > 0 = turn left
  //
  double cte_term = std::atan2(k_stanley_ * cte, speed_mps_ + k_soft_);
  double delta    = std::clamp(
    path_ang + cte_term, -max_steer_rad_, max_steer_rad_);

  cmd.twist.linear.x  = speed_mps_ * scale;
  cmd.twist.angular.z = delta;

  RCLCPP_DEBUG(logger_,
    "Stanley: cte=%+.3f path_ang=%+.1fdeg delta=%+.1fdeg scale=%.2f",
    cte,
    path_ang * 180.0 / M_PI,
    delta    * 180.0 / M_PI,
    scale);

  return cmd;
}

}  // namespace control

PLUGINLIB_EXPORT_CLASS(control::StanleyController, nav2_core::Controller)