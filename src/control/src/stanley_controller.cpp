#include "control/stanley_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <memory>

namespace control
{

// ── lifecycle ────────────────────────────────────────────────────────────────

void StanleyController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  node_        = parent;
  plugin_name_ = name;
  max_steer_rad_ = M_PI / 6.0;   // 30 degrees default

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("StanleyController: parent node is null");
  }

  logger_ = node->get_logger();

  // declare parameters (namespace = plugin_name so multiple instances work)
  node->declare_parameter(plugin_name_ + ".k_stanley",  k_stanley_);
  node->declare_parameter(plugin_name_ + ".k_soft",     k_soft_);
  node->declare_parameter(plugin_name_ + ".speed_mps",  speed_mps_);
  node->declare_parameter(plugin_name_ + ".max_steer_deg", 30.0);

  k_stanley_     = node->get_parameter(plugin_name_ + ".k_stanley").as_double();
  k_soft_        = node->get_parameter(plugin_name_ + ".k_soft").as_double();
  speed_mps_     = node->get_parameter(plugin_name_ + ".speed_mps").as_double();
  max_steer_rad_ = node->get_parameter(plugin_name_ + ".max_steer_deg").as_double()
                   * M_PI / 180.0;

  // subscribe to lane perception output
  lane_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/lane_data_array", 10,
    std::bind(&StanleyController::laneDataCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_,
    "StanleyController configured — k=%.2f  ks=%.2f  speed=%.2f m/s  max_steer=%.1f deg",
    k_stanley_, k_soft_, speed_mps_,
    node->get_parameter(plugin_name_ + ".max_steer_deg").as_double());
}

void StanleyController::cleanup()
{
  lane_sub_.reset();
  RCLCPP_INFO(logger_, "StanleyController cleaned up");
}

void StanleyController::activate()
{
  RCLCPP_INFO(logger_, "StanleyController activated");
}

void StanleyController::deactivate()
{
  RCLCPP_INFO(logger_, "StanleyController deactivated");
}

// ── lane data callback ───────────────────────────────────────────────────────

void StanleyController::laneDataCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4) {
    RCLCPP_WARN(logger_, "lane_data_array: expected 4 floats, got %zu", msg->data.size());
    return;
  }

  std::lock_guard<std::mutex> lock(lane_mutex_);
  cte_metres_     = msg->data[0];
  path_angle_rad_ = msg->data[1];
  // data[2] = lane_width_m  (not used by Stanley directly)
  lane_detected_  = msg->data[3] > 0.5;
}

// ── core: Stanley law ────────────────────────────────────────────────────────

geometry_msgs::msg::TwistStamped StanleyController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & /*pose*/,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = rclcpp::Clock().now();

  double cte, path_ang;
  bool   detected;

  {
    std::lock_guard<std::mutex> lock(lane_mutex_);
    cte      = cte_metres_;
    path_ang = path_angle_rad_;
    detected = lane_detected_;
  }

  if (!detected) {
    // publish zero twist — robot stops, Nav2 will trigger recovery
    RCLCPP_WARN_THROTTLE(logger_, *rclcpp::Clock().make_shared(), 1000,
      "StanleyController: lane not detected — stopping");
    return cmd;
  }

  // Stanley law (identical to your Python _run_stanley)
  double heading_error = -path_ang;
  double cte_term      = std::atan2(k_stanley_ * cte, speed_mps_ + k_soft_);
  double delta         = heading_error + cte_term;
  delta = std::clamp(delta, -max_steer_rad_, max_steer_rad_);

  cmd.twist.linear.x  = speed_mps_;
  cmd.twist.angular.z = -delta;   // ROS: +left  Stanley: +right

  RCLCPP_DEBUG(logger_,
    "Stanley: CTE=%.3f m  path_ang=%.2f deg  delta=%.2f deg",
    cte, path_ang * 180.0 / M_PI, delta * 180.0 / M_PI);

  return cmd;
}

// ── setPlan / setSpeedLimit ──────────────────────────────────────────────────

void StanleyController::setPlan(const nav_msgs::msg::Path & /*path*/)
{
  // Nav2 sends us a global path — we intentionally ignore it.
  // The road itself is our path; lane perception handles everything.
  RCLCPP_DEBUG(logger_, "StanleyController: setPlan called (road-following mode — plan ignored)");
}

void StanleyController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    speed_mps_ = speed_mps_ * speed_limit / 100.0;
  } else {
    speed_mps_ = speed_limit;
  }
  RCLCPP_INFO(logger_, "StanleyController: speed limit set to %.2f m/s", speed_mps_);
}

}  // namespace control

// pluginlib registration
PLUGINLIB_EXPORT_CLASS(control::StanleyController, nav2_core::Controller)