#ifndef TCPA_DCPA_CRITIC__TCPA_DCPA_CRITIC_HPP_
#define TCPA_DCPA_CRITIC__TCPA_DCPA_CRITIC_HPP_

#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include "dwb_core/trajectory_critic.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "predictive_navigation_msgs/msg/tracked_obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tcpa_dcpa_critic
{

class TCPADCPACritic : public dwb_core::TrajectoryCritic
{
public:
  TCPADCPACritic() = default;
  ~TCPADCPACritic() override = default;

  void onInit() override;
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
  void reset() override;

private:
  void trackedObstaclesCallback(
    const predictive_navigation_msgs::msg::TrackedObstacleArray::SharedPtr msg);
  std::vector<predictive_navigation_msgs::msg::TrackedObstacle> getObstaclesSnapshot() const;
  bool obstaclesAreFresh() const;

  mutable std::mutex obstacles_mutex_;
  std::vector<predictive_navigation_msgs::msg::TrackedObstacle> latest_obstacles_;
  rclcpp::Time latest_obstacles_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<predictive_navigation_msgs::msg::TrackedObstacleArray>::SharedPtr tracked_obstacles_sub_;

  double tau_safe_{2.0};
  double sigma_safe_{0.5};
  double max_obstacle_age_{0.5};
  double min_obstacle_speed_{0.05};
  double hesitation_speed_threshold_{0.75};
  double hesitation_penalty_scale_{1.5};
  double urgency_tcpa_threshold_{0.8};
  double urgency_dcpa_threshold_{0.9};
  double lateral_escape_penalty_scale_{2.0};
  double lateral_escape_speed_threshold_{0.8};
  double lateral_escape_ratio_{1.2};
  double goal_progress_penalty_scale_{1.2};
  double goal_progress_speed_threshold_{0.8};
  double escape_alignment_penalty_scale_{2.0};
  double escape_alignment_speed_threshold_{1.2};
  double escape_lateral_weight_{1.5};
  double rear_passing_penalty_scale_{0.0};
  double crossing_front_min_forward_distance_{0.3};
  double crossing_front_max_lateral_offset_{1.2};
  double crossing_obstacle_lateral_speed_threshold_{0.3};
  double crossing_obstacle_lateral_dominance_ratio_{1.2};
  double co_motion_relief_scale_{0.0};
  double co_motion_relief_min_alignment_{0.75};
  double co_motion_relief_max_relative_speed_{0.8};
  double co_motion_relief_min_multiplier_{0.35};
  double swept_corridor_penalty_scale_{0.0};
  double swept_corridor_half_width_{0.8};
  double rear_tail_margin_{0.2};
  double direction_flip_penalty_scale_{0.8};
  double direction_flip_speed_threshold_{0.2};
  double axis_commitment_penalty_scale_{0.0};
  double axis_commitment_speed_threshold_{0.2};
  bool latency_stats_enabled_{false};
  int latency_report_interval_{500};
  std::size_t latency_sample_count_{0};
  double latency_accumulator_ms_{0.0};
  double latency_max_ms_{0.0};
  double current_velocity_x_{0.0};
  double current_velocity_y_{0.0};
  double goal_direction_x_{0.0};
  double goal_direction_y_{0.0};
  bool has_goal_direction_{false};
  std::string tracked_topic_{"/tracked_obstacles"};
};

}  // namespace tcpa_dcpa_critic

#endif  // TCPA_DCPA_CRITIC__TCPA_DCPA_CRITIC_HPP_
