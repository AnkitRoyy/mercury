#pragma once

#include <string>
#include <mutex>
#include <memory>
#include <cmath>
#include <limits>
#include <utility>
#include <atomic>
#include <vector>

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
  bool isCostmapReady();

  // ── obstacle helpers ──────────────────────────────────────────────────────

  // Scale forward speed 0→1 based on nearest obstacle ahead.
  double computeObstacleSpeedScale(const geometry_msgs::msg::PoseStamped & pose);

  // Signed lateral cost bias computed across a LOOKAHEAD arc, not just
  // at the current position. Returns bias in [-1, +1].
  // Multiple slices are sampled from sample_dist_near_ → sample_dist_far_
  // and distance-weighted so far threats count more (pre-emptive steering).
  double computeLateralCostBias(
    const geometry_msgs::msg::PoseStamped & pose, double yaw);

  // ── A* path helpers ───────────────────────────────────────────────────────
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
  std::atomic<bool> costmap_ready_{false};

  // ── A* path state ─────────────────────────────────────────────────────────
  nav_msgs::msg::Path current_path_;
  std::mutex          path_mutex_;

  // ── lane state ────────────────────────────────────────────────────────────
  std::mutex lane_mutex_;
  double cte_metres_     = 0.0;
  double path_angle_rad_ = 0.0;
  double lane_width_m_   = 0.0;   // [2] from lane_data_array
  bool   lane_detected_  = false;

  // ── Stanley params ────────────────────────────────────────────────────────
  std::string plugin_name_;
  double k_stanley_     = 0.5;
  double k_soft_        = 3.0;
  double speed_mps_     = 1.0;
  double max_steer_rad_ = M_PI / 6.0;

  // ── obstacle params ───────────────────────────────────────────────────────
  double stop_dist_      = 0.5;
  double slowdown_dist_  = 3.0;
  double min_speed_      = 0.25;
  double nudge_weight_   = 0.5;   // kept for compat

  // ── lookahead lateral sampling ────────────────────────────────────────────
  // The robot samples costmap at N slices between near and far distances.
  // Each slice is weighted by its distance (farther = higher weight) so
  // the controller starts steering BEFORE it reaches the obstacle.
  double lateral_offset_   = 0.8;   // how wide left/right to probe (m)
  double sample_dist_near_ = 1.0;   // closest slice distance (m)
  double sample_dist_far_  = 4.0;   // furthest slice distance (m)  ← key param
  int    sample_slices_    = 6;     // number of slices between near and far

  // ── blend param ───────────────────────────────────────────────────────────
  double obstacle_blend_threshold_ = 0.15;

  // ── startup param ─────────────────────────────────────────────────────────
  int costmap_warmup_ms_ = 3000;
};

}  // namespace control