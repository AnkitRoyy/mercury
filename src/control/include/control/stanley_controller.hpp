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

  // setPlan() no longer uses A* path for steering.
  // Instead it builds a short synthetic straight-ahead path (3 poses, 1-3m)
  // so nav2's BT / progress_checker see a nearby reachable "goal" and stop
  // pulling the robot toward a distant A* destination through obstacles.
  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  // ── costmap readiness ─────────────────────────────────────────────────────
  bool isCostmapReady();

  // ── obstacle helpers ──────────────────────────────────────────────────────
  double computeForwardObstacleDist(const geometry_msgs::msg::PoseStamped & pose);

  // lane_width_m clamps how far laterally we sample, so the cost-bias never
  // looks past the actual road edge into off-road space.
  double computeLateralCostBias(
    const geometry_msgs::msg::PoseStamped & pose,
    double yaw,
    double lane_width_m,
    double * out_left_cost  = nullptr,
    double * out_right_cost = nullptr);

  // ── gap / doorway detection ───────────────────────────────────────────────
  // lane_width_m bounds the scan arc to roughly the lane (- lane_margin_m_),
  // so a wide-open shoulder/off-road area can no longer out-compete a
  // narrower in-lane gap. Among candidate gaps wide enough to actually drive
  // through (>= min_gap_width_m_), the one closest to straight-ahead is
  // chosen — not simply the widest one.
  double findGapCentroidOffset(
    const geometry_msgs::msg::PoseStamped & pose,
    double yaw,
    double lane_width_m,
    bool * out_found);

  // ── local obstacle bypass (gap-centered) ──────────────────────────────────
  std::pair<double, double> getGapBypassCteAndHeading(
    const geometry_msgs::msg::PoseStamped & pose,
    double yaw,
    double lane_heading_rad,
    double lane_width_m,
    bool * out_found);

  // ── synthetic path helpers ────────────────────────────────────────────────

  // Rebuilds synthetic_path_ as N poses spaced 1m apart directly ahead of
  // robot_pose along its current yaw. Called from setPlan() and also from
  // computeVelocityCommands() whenever the robot has moved far enough that
  // all remaining waypoints are behind waypoint_idx_.
  void rebuildSyntheticPath(const geometry_msgs::msg::PoseStamped & robot_pose);

  // Advances waypoint_idx_ to the first waypoint that is:
  //   (a) still ahead of the robot (positive dot-product with heading), AND
  //   (b) not in a lethal/inscribed costmap cell.
  // If every remaining waypoint is blocked, index stays at the last one
  // (rotate-in-place in step 4 of computeVelocityCommands handles that).
  // Returns the selected waypoint pose.
  geometry_msgs::msg::PoseStamped advanceToNextClearWaypoint(
    const geometry_msgs::msg::PoseStamped & pose,
    double yaw);

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

  // ── lane state ────────────────────────────────────────────────────────────
  std::mutex lane_mutex_;
  double cte_metres_     = 0.0;
  double path_angle_rad_ = 0.0;
  double lane_width_m_   = 0.0;
  bool   lane_detected_  = false;

  // ── cached lane width (persists across single-source / road-only frames) ──
  //
  // lane_width_m_ from the callback is 0.0 whenever the perception node only
  // has road_fit (no bilateral lane lines) — exactly what happens on curves.
  // cached_lane_width_px_ holds the last valid bilateral reading, low-pass
  // filtered and sanity-bounded, so gap / bias functions always have a usable
  // estimate even when fresh bilateral detections are absent.
  //
  // Pixel-domain cache (mirrors lane_following_node.py):
  //   raw bilateral reading → sanity bounds → low-pass → convert to metres
  // The metre value is what the rest of the controller sees as
  // effective_lane_width_m in computeVelocityCommands.
  double cached_lane_width_m_    = 0.0;   // 0.0 = not yet populated (cold start)
  double lane_width_alpha_       = 0.3;   // low-pass weight for new readings
  double lane_width_min_m_       = 0.5;   // sanity floor  (~2× min_gap_width_m_)
  double lane_width_max_m_       = 5.0;   // sanity ceiling (well beyond any real road)
  int    lane_width_stale_count_ = 0;     // frames since last valid bilateral reading
  int    lane_width_stale_limit_ = 30;    // after this many stale frames, clear the cache

  // ── synthetic path state ──────────────────────────────────────────────────
  // synthetic_path_: 3 poses 1m apart straight ahead, rebuilt every setPlan()
  //   call and also on-the-fly in computeVelocityCommands when exhausted.
  // waypoint_idx_:   index of the current target waypoint inside synthetic_path_.
  //   Advances past waypoints that are behind the robot or in lethal cells.
  // synthetic_path_mutex_: guards both fields (setPlan runs on a different
  //   thread from computeVelocityCommands in some nav2 configurations).
  nav_msgs::msg::Path synthetic_path_;
  size_t              waypoint_idx_ = 0;
  std::mutex          synthetic_path_mutex_;

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
  double nudge_weight_   = 0.5;

  // ── lookahead lateral sampling ────────────────────────────────────────────
  double lateral_offset_   = 0.8;
  double sample_dist_near_ = 1.0;
  double sample_dist_far_  = 4.0;
  int    sample_slices_    = 6;

  // ── blend param ───────────────────────────────────────────────────────────
  double obstacle_blend_threshold_ = 0.17;

  // Exponential smoothing applied to obstacle_weight each cycle:
  //   smoothed = alpha * raw + (1 - alpha) * previous
  // Lower alpha = slower to commit to / exit a bypass. Without this the
  // controller could snap straight to obstacle_weight = 1.0 the instant
  // obstacle_blend_threshold is crossed, and back to 0.0 just as abruptly.
  double obstacle_weight_smoothing_ = 0.3;
  double prev_obstacle_weight_      = 0.0;

  // ── gap / doorway scan params ─────────────────────────────────────────────
  double door_scan_dist_       = 5.0;
  double door_scan_half_width_ = 2.0;
  int    door_scan_slices_     = 21;

  // Safety margin (metres) SUBTRACTED from half the detected lane width
  // when bounding the gap scan / lateral sampling — pulls the corridor IN
  // from the lane edge (accounts for robot footprint / wheel clearance).
  // Must be subtracted, not added: adding it pushes the boundary past the
  // actual lane edge, defeating the purpose.
  double lane_margin_m_ = 0.3;

  // Minimum width (metres) a free run must have to be treated as a usable
  // gap. Prevents targeting a sliver of clearance the robot can't actually
  // fit through.
  double min_gap_width_m_ = 0.6;

  // Half-width used for the gap scan when no usable lane-width estimate is
  // available (cached_lane_width_m_ == 0.0 i.e. cache not yet populated, or
  // gone stale past lane_width_stale_limit_ frames). Intentionally narrower
  // than door_scan_half_width_ so the scan never looks into off-road space
  // when the lane detector hasn't established a bilateral baseline yet.
  double door_scan_fallback_half_width_ = 2.0;

  // Sign correction applied to the gap-bypass CTE before it's blended with
  // lane_cte. lane_cte's sign convention comes from the external lane
  // detector and isn't guaranteed to match the convention used here
  // (positive = right of heading). If the robot steers the wrong way
  // specifically during GAP_BYPASS mode, flip this to -1.0.
  double gap_bypass_cte_sign_ = 1.0;

  // ── gap bypass heading gate (NEW) ─────────────────────────────────────────
  // Gap bypass is suppressed when the gap centroid direction contradicts the
  // lane heading sign. This prevents the bypass from steering the robot off-road
  // when an obstacle blocks a curve: the "free gap" on the wrong side of the
  // road looks attractive to the costmap scanner but is actually off-road.
  //
  // When |lane_heading| >= gap_heading_gate_thresh_rad_, the gap centroid CTE
  // must share the same sign as lane_heading (or be within gap_heading_gate_tol_
  // of zero) or the bypass is rejected and lane-only control is used instead.
  //
  // Tune:
  //   gap_heading_gate_thresh_deg: raise if bypass is over-rejected on mild bends;
  //                                lower if wrong-side bypass still fires on curves.
  //   gap_heading_gate_tol:        dead-band around zero for "nearly straight" roads
  //                                where either side of a gap is acceptable.
  double gap_heading_gate_thresh_rad_ = 5.0 * M_PI / 180.0;  // param: gap_heading_gate_thresh_deg
  double gap_heading_gate_tol_        = 0.05;                  // param: gap_heading_gate_tol (rad)

  // ── rotate-in-place heading bias (NEW) ────────────────────────────────────
  // When fwd_dist <= stop_dist_ AND |lane_heading| >= this threshold, the
  // rotate-in-place spin direction is taken from the lane_heading sign rather
  // than from the lateral costmap bias. This ensures the robot spins toward
  // the direction the road is actually curving, not toward whichever side of
  // the road happens to have lower cost (which can be the wrong side when an
  // obstacle sits on the correct curve side).
  //
  // Tune:
  //   Raise if the heading-biased spin fires too early on near-straight roads.
  //   Lower if the robot still spins the wrong way on tighter curves.
  double rotate_heading_thresh_rad_ = 5.0 * M_PI / 180.0;  // param: rotate_heading_thresh_deg

  // ── synthetic path params ─────────────────────────────────────────────────
  // Number of waypoints to place ahead (spaced 1m apart).
  // 3 is enough to keep nav2's progress_checker happy while staying close
  // enough that A* never routes the "goal" far away through obstacles.
  int    synthetic_wp_count_  = 3;   // declared via param "synthetic_wp_count"
  double synthetic_wp_spacing_ = 1.0; // metres between waypoints

  // ── rotate-in-place param ─────────────────────────────────────────────────
  double rotate_in_place_speed_ = 0.4;

  // ── curve tracking threshold ──────────────────────────────────────────────
  // When |lane_heading| >= this value the robot is considered to be on a curve
  // and switches to pure curvature-tracking mode (delta = lane_heading only).
  // No CTE correction, no gap bypass. Tunable via param "curve_heading_thresh_deg".
  // Default 10°. Raise if gentle bends still trigger gap bypass; lower if lateral
  // drift on near-straights is too slow to correct.
  double curve_heading_thresh_rad_ = 10.0 * M_PI / 180.0;

  // ── startup param ─────────────────────────────────────────────────────────
  int costmap_warmup_ms_ = 3000;
};

}  // namespace control