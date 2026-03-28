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

  node->get_parameter(critic_ns + ".tau_safe", tau_safe_);
  node->get_parameter(critic_ns + ".sigma_safe", sigma_safe_);
  node->get_parameter(critic_ns + ".tracked_topic", tracked_topic_);
  node->get_parameter(critic_ns + ".max_obstacle_age", max_obstacle_age_);
  node->get_parameter(critic_ns + ".min_obstacle_speed", min_obstacle_speed_);

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

  tracked_obstacles_sub_ = node->create_subscription<predictive_navigation_msgs::msg::TrackedObstacleArray>(
    tracked_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&TCPADCPACritic::trackedObstaclesCallback, this, std::placeholders::_1));
}

bool TCPADCPACritic::prepare(
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D &)
{
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

    const double temporal_term = std::exp(-tcpa / tau_safe_);
    const double spatial_term = std::exp(-dcpa_sq / (2.0 * sigma_safe_sq));
    total_cost += temporal_term * spatial_term;
  }

  return total_cost * scale_;
}

void TCPADCPACritic::reset()
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  latest_obstacles_.clear();
  latest_obstacles_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
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
