#ifndef TCPA_DCPA_CRITIC__TCPA_DCPA_CRITIC_HPP_
#define TCPA_DCPA_CRITIC__TCPA_DCPA_CRITIC_HPP_

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

  mutable std::mutex obstacles_mutex_;
  std::vector<predictive_navigation_msgs::msg::TrackedObstacle> latest_obstacles_;
  rclcpp::Subscription<predictive_navigation_msgs::msg::TrackedObstacleArray>::SharedPtr tracked_obstacles_sub_;

  double tau_safe_{2.0};
  double sigma_safe_{0.5};
  std::string tracked_topic_{"/tracked_obstacles"};
};

}  // namespace tcpa_dcpa_critic

#endif  // TCPA_DCPA_CRITIC__TCPA_DCPA_CRITIC_HPP_
