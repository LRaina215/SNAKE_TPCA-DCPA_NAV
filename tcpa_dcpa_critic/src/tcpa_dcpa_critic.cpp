#include "tcpa_dcpa_critic/tcpa_dcpa_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace tcpa_dcpa_critic
{

void TCPADCPACritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock lifecycle node in TCPADCPACritic::onInit");
  }

  const std::string critic_ns = dwb_plugin_name_ + "." + name_;
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".tau_safe", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".sigma_safe", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".tracked_topic", rclcpp::ParameterValue(std::string("/tracked_obstacles")));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".max_obstacle_age", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".min_obstacle_speed", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".hesitation_speed_threshold", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".hesitation_penalty_scale", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".urgency_tcpa_threshold", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".urgency_dcpa_threshold", rclcpp::ParameterValue(0.9));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".lateral_escape_penalty_scale", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".lateral_escape_speed_threshold", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".lateral_escape_ratio", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".goal_progress_penalty_scale", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".goal_progress_speed_threshold", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".direction_flip_penalty_scale", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".direction_flip_speed_threshold", rclcpp::ParameterValue(0.2));

  node->get_parameter(critic_ns + ".tau_safe", tau_safe_);
  node->get_parameter(critic_ns + ".sigma_safe", sigma_safe_);
  node->get_parameter(critic_ns + ".tracked_topic", tracked_topic_);
  node->get_parameter(critic_ns + ".max_obstacle_age", max_obstacle_age_);
  node->get_parameter(critic_ns + ".min_obstacle_speed", min_obstacle_speed_);
  node->get_parameter(critic_ns + ".hesitation_speed_threshold", hesitation_speed_threshold_);
  node->get_parameter(critic_ns + ".hesitation_penalty_scale", hesitation_penalty_scale_);
  node->get_parameter(critic_ns + ".urgency_tcpa_threshold", urgency_tcpa_threshold_);
  node->get_parameter(critic_ns + ".urgency_dcpa_threshold", urgency_dcpa_threshold_);
  node->get_parameter(critic_ns + ".lateral_escape_penalty_scale", lateral_escape_penalty_scale_);
  node->get_parameter(critic_ns + ".lateral_escape_speed_threshold", lateral_escape_speed_threshold_);
  node->get_parameter(critic_ns + ".lateral_escape_ratio", lateral_escape_ratio_);
  node->get_parameter(critic_ns + ".goal_progress_penalty_scale", goal_progress_penalty_scale_);
  node->get_parameter(critic_ns + ".goal_progress_speed_threshold", goal_progress_speed_threshold_);
  node->get_parameter(critic_ns + ".direction_flip_penalty_scale", direction_flip_penalty_scale_);
  node->get_parameter(critic_ns + ".direction_flip_speed_threshold", direction_flip_speed_threshold_);

  if (tau_safe_ <= 0.0) {
    RCLCPP_WARN(
      node->get_logger(), "%s.tau_safe must be positive. Clamping to 1e-3.", critic_ns.c_str());
    tau_safe_ = 1e-3;
  }
  if (sigma_safe_ <= 0.0) {
    RCLCPP_WARN(
      node->get_logger(), "%s.sigma_safe must be positive. Clamping to 1e-3.", critic_ns.c_str());
    sigma_safe_ = 1e-3;
  }
  if (max_obstacle_age_ <= 0.0) {
    RCLCPP_WARN(
      node->get_logger(), "%s.max_obstacle_age must be positive. Clamping to 1e-3.", critic_ns.c_str());
    max_obstacle_age_ = 1e-3;
  }
  if (min_obstacle_speed_ < 0.0) {
    RCLCPP_WARN(
      node->get_logger(), "%s.min_obstacle_speed must be non-negative. Clamping to 0.0.", critic_ns.c_str());
    min_obstacle_speed_ = 0.0;
  }
  hesitation_speed_threshold_ = std::max(0.0, hesitation_speed_threshold_);
  hesitation_penalty_scale_ = std::max(0.0, hesitation_penalty_scale_);
  urgency_tcpa_threshold_ = std::max(0.0, urgency_tcpa_threshold_);
  urgency_dcpa_threshold_ = std::max(0.0, urgency_dcpa_threshold_);
  lateral_escape_penalty_scale_ = std::max(0.0, lateral_escape_penalty_scale_);
  lateral_escape_speed_threshold_ = std::max(0.0, lateral_escape_speed_threshold_);
  lateral_escape_ratio_ = std::max(1.0, lateral_escape_ratio_);
  goal_progress_penalty_scale_ = std::max(0.0, goal_progress_penalty_scale_);
  goal_progress_speed_threshold_ = std::max(0.0, goal_progress_speed_threshold_);
  direction_flip_penalty_scale_ = std::max(0.0, direction_flip_penalty_scale_);
  direction_flip_speed_threshold_ = std::max(0.0, direction_flip_speed_threshold_);

  tracked_obstacles_sub_ = node->create_subscription<predictive_navigation_msgs::msg::TrackedObstacleArray>(
    tracked_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&TCPADCPACritic::trackedObstaclesCallback, this, std::placeholders::_1));
}

bool TCPADCPACritic::prepare(
  const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Twist2D & vel,
  const geometry_msgs::msg::Pose2D & goal,
  const nav_2d_msgs::msg::Path2D &)
{
  current_velocity_x_ = vel.x;
  current_velocity_y_ = vel.y;

  const double goal_dx = goal.x - pose.x;
  const double goal_dy = goal.y - pose.y;
  const double goal_distance = std::hypot(goal_dx, goal_dy);
  if (goal_distance > 1e-6) {
    goal_direction_x_ = goal_dx / goal_distance;
    goal_direction_y_ = goal_dy / goal_distance;
    has_goal_direction_ = true;
  } else {
    goal_direction_x_ = 0.0;
    goal_direction_y_ = 0.0;
    has_goal_direction_ = false;
  }
  return true;
}

double TCPADCPACritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  const auto obstacles = getObstaclesSnapshot();
  if (obstacles.empty() || traj.poses.empty() || !obstaclesAreFresh()) {
    return 0.0;
  }

  const auto & robot_pose = traj.poses.front();
  const double robot_vx = traj.velocity.x;
  const double robot_vy = traj.velocity.y;
  const double sigma_safe_sq = sigma_safe_ * sigma_safe_;
  double total_cost = 0.0;

  for (const auto & obstacle : obstacles) {
    const double obstacle_speed = std::hypot(obstacle.velocity.x, obstacle.velocity.y);
    if (obstacle_speed < min_obstacle_speed_) {
      continue;
    }

    const double rel_px = obstacle.position.x - robot_pose.x;
    const double rel_py = obstacle.position.y - robot_pose.y;
    const double rel_vx = robot_vx - obstacle.velocity.x;
    const double rel_vy = robot_vy - obstacle.velocity.y;
    const double rel_v_sq = (rel_vx * rel_vx) + (rel_vy * rel_vy);
    if (rel_v_sq <= 1e-9) {
      continue;
    }

    const double approach_trend = (rel_px * rel_vx) + (rel_py * rel_vy);
    if (approach_trend <= 0.0) {
      continue;
    }

    const double tcpa = approach_trend / rel_v_sq;
    if (tcpa <= 0.0 || tcpa > tau_safe_) {
      continue;
    }

    const double dcpa_x = rel_px - (tcpa * rel_vx);
    const double dcpa_y = rel_py - (tcpa * rel_vy);
    const double dcpa_sq = (dcpa_x * dcpa_x) + (dcpa_y * dcpa_y);

    const double dcpa = std::sqrt(dcpa_sq);
    const double temporal_term = std::exp(-tcpa / tau_safe_);
    const double spatial_term = std::exp(-dcpa_sq / (2.0 * sigma_safe_sq));
    const double risk_term = temporal_term * spatial_term;
    total_cost += risk_term;

    const bool urgent_interaction =
      (tcpa <= urgency_tcpa_threshold_) && (dcpa <= urgency_dcpa_threshold_);
    if (!urgent_interaction) {
      continue;
    }

    const double traj_speed = std::hypot(robot_vx, robot_vy);
    if (traj_speed < hesitation_speed_threshold_ && hesitation_speed_threshold_ > 1e-9) {
      const double speed_deficit = 1.0 - (traj_speed / hesitation_speed_threshold_);
      total_cost += hesitation_penalty_scale_ * risk_term * speed_deficit;
    }

    const bool laterally_dominant = std::abs(rel_py) > (lateral_escape_ratio_ * std::abs(rel_px));
    if (laterally_dominant && lateral_escape_speed_threshold_ > 1e-9) {
      const double side_sign = (rel_py >= 0.0) ? 1.0 : -1.0;
      const double away_lateral_speed = -side_sign * robot_vy;
      if (away_lateral_speed < lateral_escape_speed_threshold_) {
        const double lateral_speed_deficit =
          (lateral_escape_speed_threshold_ - away_lateral_speed) / lateral_escape_speed_threshold_;
        total_cost += lateral_escape_penalty_scale_ * risk_term * lateral_speed_deficit;
      }

      if (has_goal_direction_ && goal_progress_speed_threshold_ > 1e-9) {
        const double goal_progress_speed =
          (robot_vx * goal_direction_x_) + (robot_vy * goal_direction_y_);
        if (goal_progress_speed < goal_progress_speed_threshold_) {
          const double goal_progress_deficit =
            (goal_progress_speed_threshold_ - goal_progress_speed) / goal_progress_speed_threshold_;
          total_cost += goal_progress_penalty_scale_ * risk_term * goal_progress_deficit;
        }
      }
    }

    const double current_speed = std::hypot(current_velocity_x_, current_velocity_y_);
    if (traj_speed > 1e-9 && current_speed > direction_flip_speed_threshold_) {
      const double normalized_dot =
        ((robot_vx * current_velocity_x_) + (robot_vy * current_velocity_y_)) /
        (traj_speed * current_speed);
      if (normalized_dot < 0.0) {
        total_cost += direction_flip_penalty_scale_ * risk_term * (-normalized_dot);
      }
    }
  }

  return total_cost * scale_;
}

void TCPADCPACritic::reset()
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  latest_obstacles_.clear();
  latest_obstacles_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  current_velocity_x_ = 0.0;
  current_velocity_y_ = 0.0;
  goal_direction_x_ = 0.0;
  goal_direction_y_ = 0.0;
  has_goal_direction_ = false;
}

void TCPADCPACritic::trackedObstaclesCallback(
  const predictive_navigation_msgs::msg::TrackedObstacleArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  latest_obstacles_ = msg->obstacles;
  latest_obstacles_stamp_ = msg->header.stamp;
}

std::vector<predictive_navigation_msgs::msg::TrackedObstacle>
TCPADCPACritic::getObstaclesSnapshot() const
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  return latest_obstacles_;
}

bool TCPADCPACritic::obstaclesAreFresh() const
{
  auto node = node_.lock();
  if (!node) {
    return false;
  }

  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  if (latest_obstacles_stamp_.nanoseconds() == 0) {
    return false;
  }

  const double obstacle_age = (node->now() - latest_obstacles_stamp_).seconds();
  return obstacle_age <= max_obstacle_age_;
}

}  // namespace tcpa_dcpa_critic

PLUGINLIB_EXPORT_CLASS(tcpa_dcpa_critic::TCPADCPACritic, dwb_core::TrajectoryCritic)
