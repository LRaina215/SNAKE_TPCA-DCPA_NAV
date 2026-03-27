#include "tcpa_dcpa_critic/tcpa_dcpa_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace tcpa_dcpa_critic
{

namespace
{

double durationToSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + (static_cast<double>(duration.nanosec) * 1e-9);
}

}  // namespace

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

  node->get_parameter(critic_ns + ".tau_safe", tau_safe_);
  node->get_parameter(critic_ns + ".sigma_safe", sigma_safe_);
  node->get_parameter(critic_ns + ".tracked_topic", tracked_topic_);

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
  if (obstacles.empty() || traj.poses.empty()) {
    return 0.0;
  }

  const double robot_vx = traj.velocity.x;
  const double robot_vy = traj.velocity.y;
  const double sigma_safe_sq = sigma_safe_ * sigma_safe_;
  double total_cost = 0.0;

  for (const auto & obstacle : obstacles) {
    const double rel_vx = robot_vx - obstacle.velocity.x;
    const double rel_vy = robot_vy - obstacle.velocity.y;
    const double rel_v_sq = (rel_vx * rel_vx) + (rel_vy * rel_vy);
    if (rel_v_sq <= 1e-9) {
      continue;
    }

    for (std::size_t i = 0; i < traj.poses.size(); ++i) {
      const double sample_time =
        (i < traj.time_offsets.size()) ? durationToSeconds(traj.time_offsets[i]) : 0.0;

      const double robot_px = traj.poses[i].x;
      const double robot_py = traj.poses[i].y;
      const double obstacle_px = obstacle.position.x + (obstacle.velocity.x * sample_time);
      const double obstacle_py = obstacle.position.y + (obstacle.velocity.y * sample_time);

      const double rel_px = obstacle_px - robot_px;
      const double rel_py = obstacle_py - robot_py;
      const double approach_trend = (rel_px * rel_vx) + (rel_py * rel_vy);
      if (approach_trend <= 0.0) {
        continue;
      }

      const double tcpa = approach_trend / rel_v_sq;
      if (tcpa > tau_safe_) {
        continue;
      }

      const double dcpa_x = rel_px - (tcpa * rel_vx);
      const double dcpa_y = rel_py - (tcpa * rel_vy);
      const double dcpa_sq = (dcpa_x * dcpa_x) + (dcpa_y * dcpa_y);

      const double temporal_term = std::exp(-tcpa / tau_safe_);
      const double spatial_term = std::exp(-dcpa_sq / (2.0 * sigma_safe_sq));
      total_cost += temporal_term * spatial_term;
    }
  }

  return total_cost * scale_;
}

void TCPADCPACritic::reset()
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  latest_obstacles_.clear();
}

void TCPADCPACritic::trackedObstaclesCallback(
  const predictive_navigation_msgs::msg::TrackedObstacleArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  latest_obstacles_ = msg->obstacles;
}

std::vector<predictive_navigation_msgs::msg::TrackedObstacle>
TCPADCPACritic::getObstaclesSnapshot() const
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  return latest_obstacles_;
}

}  // namespace tcpa_dcpa_critic

PLUGINLIB_EXPORT_CLASS(tcpa_dcpa_critic::TCPADCPACritic, dwb_core::TrajectoryCritic)
