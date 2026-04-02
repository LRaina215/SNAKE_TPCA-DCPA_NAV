#include "tcpa_dcpa_critic/tcpa_dcpa_critic.hpp"

#include <algorithm>
#include <chrono>
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
    node, critic_ns + ".escape_alignment_penalty_scale", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".escape_alignment_speed_threshold", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".escape_lateral_weight", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".rear_passing_penalty_scale", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".crossing_front_min_forward_distance", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".crossing_front_max_lateral_offset", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".crossing_obstacle_lateral_speed_threshold", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".crossing_obstacle_lateral_dominance_ratio", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".swept_corridor_penalty_scale", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".swept_corridor_half_width", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".rear_tail_margin", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".latency_stats_enabled", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, critic_ns + ".latency_report_interval", rclcpp::ParameterValue(500));
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
  node->get_parameter(critic_ns + ".escape_alignment_penalty_scale", escape_alignment_penalty_scale_);
  node->get_parameter(critic_ns + ".escape_alignment_speed_threshold", escape_alignment_speed_threshold_);
  node->get_parameter(critic_ns + ".escape_lateral_weight", escape_lateral_weight_);
  node->get_parameter(critic_ns + ".rear_passing_penalty_scale", rear_passing_penalty_scale_);
  node->get_parameter(
    critic_ns + ".crossing_front_min_forward_distance", crossing_front_min_forward_distance_);
  node->get_parameter(
    critic_ns + ".crossing_front_max_lateral_offset", crossing_front_max_lateral_offset_);
  node->get_parameter(
    critic_ns + ".crossing_obstacle_lateral_speed_threshold",
    crossing_obstacle_lateral_speed_threshold_);
  node->get_parameter(
    critic_ns + ".crossing_obstacle_lateral_dominance_ratio",
    crossing_obstacle_lateral_dominance_ratio_);
  node->get_parameter(
    critic_ns + ".swept_corridor_penalty_scale", swept_corridor_penalty_scale_);
  node->get_parameter(
    critic_ns + ".swept_corridor_half_width", swept_corridor_half_width_);
  node->get_parameter(
    critic_ns + ".rear_tail_margin", rear_tail_margin_);
  node->get_parameter(
    critic_ns + ".latency_stats_enabled", latency_stats_enabled_);
  node->get_parameter(
    critic_ns + ".latency_report_interval", latency_report_interval_);
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
  escape_alignment_penalty_scale_ = std::max(0.0, escape_alignment_penalty_scale_);
  escape_alignment_speed_threshold_ = std::max(0.0, escape_alignment_speed_threshold_);
  escape_lateral_weight_ = std::max(0.0, escape_lateral_weight_);
  rear_passing_penalty_scale_ = std::max(0.0, rear_passing_penalty_scale_);
  crossing_front_min_forward_distance_ = std::max(0.0, crossing_front_min_forward_distance_);
  crossing_front_max_lateral_offset_ = std::max(0.0, crossing_front_max_lateral_offset_);
  crossing_obstacle_lateral_speed_threshold_ =
    std::max(0.0, crossing_obstacle_lateral_speed_threshold_);
  crossing_obstacle_lateral_dominance_ratio_ =
    std::max(1.0, crossing_obstacle_lateral_dominance_ratio_);
  swept_corridor_penalty_scale_ = std::max(0.0, swept_corridor_penalty_scale_);
  swept_corridor_half_width_ = std::max(0.0, swept_corridor_half_width_);
  rear_tail_margin_ = std::max(0.0, rear_tail_margin_);
  latency_report_interval_ = std::max(1, latency_report_interval_);
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
  const auto start_time = std::chrono::steady_clock::now();
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

    bool use_motion_based_escape = false;
    double desired_escape_sign = 0.0;
    double desired_lateral_speed = 0.0;
    double goal_lateral_x = 0.0;
    double goal_lateral_y = 0.0;
    double robot_lateral_speed = robot_vy;
    if (has_goal_direction_) {
      goal_lateral_x = -goal_direction_y_;
      goal_lateral_y = goal_direction_x_;

      const double rel_forward =
        (rel_px * goal_direction_x_) + (rel_py * goal_direction_y_);
      const double rel_lateral =
        (rel_px * goal_lateral_x) + (rel_py * goal_lateral_y);
      const double obstacle_forward_speed =
        (obstacle.velocity.x * goal_direction_x_) + (obstacle.velocity.y * goal_direction_y_);
      const double obstacle_lateral_speed =
        (obstacle.velocity.x * goal_lateral_x) + (obstacle.velocity.y * goal_lateral_y);
      const double obstacle_lateral_abs = std::abs(obstacle_lateral_speed);
      const double obstacle_forward_abs = std::abs(obstacle_forward_speed);

      robot_lateral_speed = (robot_vx * goal_lateral_x) + (robot_vy * goal_lateral_y);

      const bool front_crossing =
        rel_forward >= crossing_front_min_forward_distance_ &&
        std::abs(rel_lateral) <= crossing_front_max_lateral_offset_ &&
        obstacle_lateral_abs >= crossing_obstacle_lateral_speed_threshold_ &&
        obstacle_lateral_abs >=
        (crossing_obstacle_lateral_dominance_ratio_ * obstacle_forward_abs);

      if (front_crossing) {
        use_motion_based_escape = true;
        desired_escape_sign = (obstacle_lateral_speed >= 0.0) ? -1.0 : 1.0;
        desired_lateral_speed = desired_escape_sign * robot_lateral_speed;

        if (rear_passing_penalty_scale_ > 1e-9 && desired_lateral_speed < 0.0) {
          const double follow_speed_ratio =
            std::min(1.0, (-desired_lateral_speed) /
            std::max(lateral_escape_speed_threshold_, 1e-3));
          total_cost += rear_passing_penalty_scale_ * risk_term * (1.0 + follow_speed_ratio);
        }

        if (swept_corridor_penalty_scale_ > 1e-9 && swept_corridor_half_width_ > 1e-9) {
          const double eval_time = std::min(tcpa, urgency_tcpa_threshold_);
          const double robot_pred_x = robot_pose.x + (robot_vx * eval_time);
          const double robot_pred_y = robot_pose.y + (robot_vy * eval_time);
          const double obstacle_pred_x = obstacle.position.x + (obstacle.velocity.x * eval_time);
          const double obstacle_pred_y = obstacle.position.y + (obstacle.velocity.y * eval_time);

          const double obs_dir_x = obstacle.velocity.x / obstacle_speed;
          const double obs_dir_y = obstacle.velocity.y / obstacle_speed;
          const double obs_normal_x = -obs_dir_y;
          const double obs_normal_y = obs_dir_x;

          const double from_obstacle_x = robot_pred_x - obstacle_pred_x;
          const double from_obstacle_y = robot_pred_y - obstacle_pred_y;
          const double corridor_longitudinal =
            (from_obstacle_x * obs_dir_x) + (from_obstacle_y * obs_dir_y);
          const double corridor_lateral =
            std::abs((from_obstacle_x * obs_normal_x) + (from_obstacle_y * obs_normal_y));

          if (corridor_lateral < swept_corridor_half_width_ &&
            corridor_longitudinal > -rear_tail_margin_)
          {
            const double lateral_exposure =
              1.0 - (corridor_lateral / swept_corridor_half_width_);
            const double longitudinal_exposure =
              (corridor_longitudinal + rear_tail_margin_) /
              std::max(rear_tail_margin_ + swept_corridor_half_width_, 1e-3);
            total_cost += swept_corridor_penalty_scale_ * risk_term *
              (lateral_exposure + longitudinal_exposure);
          }
        }
      }
    }

    const bool laterally_dominant = std::abs(rel_py) > (lateral_escape_ratio_ * std::abs(rel_px));
    if ((use_motion_based_escape || laterally_dominant) && lateral_escape_speed_threshold_ > 1e-9) {
      if (!use_motion_based_escape) {
        const double side_sign = (rel_py >= 0.0) ? 1.0 : -1.0;
        desired_escape_sign = -side_sign;
        desired_lateral_speed = desired_escape_sign * robot_vy;
      }

      if (desired_lateral_speed < lateral_escape_speed_threshold_) {
        const double lateral_speed_deficit =
          (lateral_escape_speed_threshold_ - desired_lateral_speed) / lateral_escape_speed_threshold_;
        total_cost += lateral_escape_penalty_scale_ * risk_term * lateral_speed_deficit;
      }

      double goal_progress_speed = 0.0;
      if (has_goal_direction_) {
        goal_progress_speed =
          (robot_vx * goal_direction_x_) + (robot_vy * goal_direction_y_);
        if (goal_progress_speed_threshold_ > 1e-9 &&
          goal_progress_speed < goal_progress_speed_threshold_)
        {
          const double goal_progress_deficit =
            (goal_progress_speed_threshold_ - goal_progress_speed) / goal_progress_speed_threshold_;
          total_cost += goal_progress_penalty_scale_ * risk_term * goal_progress_deficit;
        }
      }

      if (has_goal_direction_ && escape_alignment_speed_threshold_ > 1e-9) {
        const double preferred_escape_x =
          goal_direction_x_ + (desired_escape_sign * escape_lateral_weight_ * goal_lateral_x);
        const double preferred_escape_y =
          goal_direction_y_ + (desired_escape_sign * escape_lateral_weight_ * goal_lateral_y);
        const double preferred_escape_norm =
          std::hypot(preferred_escape_x, preferred_escape_y);
        if (preferred_escape_norm > 1e-9) {
          const double preferred_escape_dir_x = preferred_escape_x / preferred_escape_norm;
          const double preferred_escape_dir_y = preferred_escape_y / preferred_escape_norm;
          const double escape_alignment_speed =
            (robot_vx * preferred_escape_dir_x) + (robot_vy * preferred_escape_dir_y);
          if (escape_alignment_speed < escape_alignment_speed_threshold_) {
            const double escape_alignment_deficit =
              (escape_alignment_speed_threshold_ - escape_alignment_speed) /
              escape_alignment_speed_threshold_;
            total_cost +=
              escape_alignment_penalty_scale_ * risk_term * escape_alignment_deficit;
          }
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

  const double final_cost = total_cost * scale_;
  if (latency_stats_enabled_) {
    auto node = node_.lock();
    if (node) {
      const auto end_time = std::chrono::steady_clock::now();
      const double elapsed_ms =
        std::chrono::duration<double, std::milli>(end_time - start_time).count();
      latency_sample_count_ += 1;
      latency_accumulator_ms_ += elapsed_ms;
      latency_max_ms_ = std::max(latency_max_ms_, elapsed_ms);
      if (latency_sample_count_ % static_cast<std::size_t>(latency_report_interval_) == 0) {
        const double average_ms = latency_accumulator_ms_ /
          static_cast<double>(latency_sample_count_);
        RCLCPP_INFO(
          node->get_logger(),
          "TCPADCPA score latency over %zu calls: avg=%.4f ms max=%.4f ms",
          latency_sample_count_, average_ms, latency_max_ms_);
        latency_sample_count_ = 0;
        latency_accumulator_ms_ = 0.0;
        latency_max_ms_ = 0.0;
      }
    }
  }

  return final_cost;
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
