#pragma once

#include <string>
#include <mutex>
#include <memory>
#include <cmath>
#include <limits>
#include <utility>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2/utils.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace control
{

class StanleyController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup()    override;
  void activate()   override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist       & velocity,
    nav2_core::GoalChecker                * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // ── costmap readiness ─────────────────────────────────────────────────────
  // Returns true once costmap has received at least one sensor update.
  // Safe to call from computeVelocityCommands on every tick — fast after latch.
  bool isCostmapReady();

  // ── obstacle helpers ──────────────────────────────────────────────────────

  // Scale forward speed 0→1 based on nearest obstacle ahead.
  // Returns 1.0 (no slowdown) if costmap is not yet ready.
  double computeObstacleSpeedScale(const geometry_msgs::msg::PoseStamped & pose);

  // Signed lateral cost bias in [-1, +1].
  //   +1 = heavy cost on right → blend toward A* path (avoid right obstacle)
  //   -1 = heavy cost on left  → blend toward A* path (avoid left obstacle)
  // Returns 0.0 (no bias) if costmap is not yet ready.
  double computeLateralCostBias(
    const geometry_msgs::msg::PoseStamped & pose, double yaw);

  // ── A* path helpers ───────────────────────────────────────────────────────

  // Returns {cte, heading_error} from the stored Nav2 A* path.
  // cte > 0  → robot is left  of path → steer right (angular.z < 0)
  // cte < 0  → robot is right of path → steer left  (angular.z > 0)
  std::pair<double, double> getPathCteAndHeading(
    const geometry_msgs::msg::PoseStamped & pose);

  // ── lane callback ─────────────────────────────────────────────────────────
  void laneDataCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // ── ROS handles ───────────────────────────────────────────────────────────
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger              logger_{rclcpp::get_logger("StanleyController")};
  rclcpp::Clock::SharedPtr    clock_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr lane_sub_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // ── costmap warmup state ──────────────────────────────────────────────────
  // Latches true the first time isCurrent() returns true.
  // Declared atomic so it is safe to read from the control thread without
  // a mutex (only ever transitions false → true, never back).
  std::atomic<bool> costmap_ready_{false};

  // ── A* path state ─────────────────────────────────────────────────────────
  nav_msgs::msg::Path current_path_;
  std::mutex          path_mutex_;

  // ── lane state ────────────────────────────────────────────────────────────
  std::mutex lane_mutex_;
  double cte_metres_     = 0.0;
  double path_angle_rad_ = 0.0;
  bool   lane_detected_  = false;

  // ── Stanley params ────────────────────────────────────────────────────────
  std::string plugin_name_;
  double k_stanley_     = 0.5;
  double k_soft_        = 3.0;
  double speed_mps_     = 1.0;
  double max_steer_rad_ = M_PI / 6.0;

  // ── obstacle params ───────────────────────────────────────────────────────
  double stop_dist_      = 0.8;   // hard-stop distance (m)
  double slowdown_dist_  = 3.0;   // begin slowing at this distance (m)
  double min_speed_      = 0.20;  // minimum speed while slowing
  double nudge_weight_   = 0.9;   // kept for compat (not used in blend logic)
  double lateral_offset_ = 0.8;   // lateral sample distance (m)
  double sample_dist_    = 2.5;   // forward sample distance (m)

  // ── blend param ───────────────────────────────────────────────────────────
  // Bias magnitude at which A* path weight reaches 1.0 (full takeover).
  // Lower value = A* kicks in sooner / more aggressively.
  double obstacle_blend_threshold_ = 0.20;

  // ── startup param ─────────────────────────────────────────────────────────
  // Extra sleep in activate() to let costmap receive its first scan (ms).
  // Set to 0 to rely purely on isCurrent() check with no blocking sleep.
  int costmap_warmup_ms_ = 500;
};

}  // namespace control