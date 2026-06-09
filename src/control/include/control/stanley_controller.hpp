#ifndef CONTROL__STANLEY_CONTROLLER_HPP_
#define CONTROL__STANLEY_CONTROLLER_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace control
{

class StanleyController : public nav2_core::Controller
{
public:
  StanleyController() = default;
  ~StanleyController() override = default;

  // nav2_core::Controller interface
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup()  override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  void laneDataCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("StanleyController")};
  std::string plugin_name_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr lane_sub_;

  // latest lane data (protected by mutex)
  std::mutex lane_mutex_;
  double cte_metres_    {0.0};
  double path_angle_rad_{0.0};
  bool   lane_detected_ {false};

  // parameters
  double k_stanley_  {0.5};
  double k_soft_     {3.0};
  double speed_mps_  {1.0};
  double max_steer_rad_;
};

}  // namespace control

#endif  // CONTROL__STANLEY_CONTROLLER_HPP_