#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <predictive_navigation_msgs/msg/tracked_obstacle.hpp>
#include <predictive_navigation_msgs/msg/tracked_obstacle_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
struct Detection
{
  Eigen::Vector2d position_xy;
  double z{0.0};
};

struct Track
{
  int id{0};
  Eigen::Vector4d state{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d covariance{Eigen::Matrix4d::Identity()};
  double z{0.0};
  rclcpp::Time last_update_stamp{0, 0, RCL_ROS_TIME};
  int hit_frames{0};
  int dynamic_hit_frames{0};
  int missed_frames{0};
};

struct Assignment
{
  std::size_t detection_index{0};
  int track_id{0};
  double distance{0.0};
};
}  // namespace

class DynamicTrackerNode : public rclcpp::Node
{
public:
  DynamicTrackerNode()
  : Node("dynamic_tracker_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/terrain_map");
    tracked_topic_ = declare_parameter<std::string>("tracked_topic", "/tracked_obstacles");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/tracked_obstacle_markers");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    cluster_tolerance_ = declare_parameter<double>("cluster_tolerance", 0.35);
    min_cluster_size_ = declare_parameter<int>("min_cluster_size", 8);
    max_cluster_size_ = declare_parameter<int>("max_cluster_size", 5000);
    association_distance_threshold_ =
      declare_parameter<double>("association_distance_threshold", 0.8);
    max_missed_frames_ = declare_parameter<int>("max_missed_frames", 5);
    process_noise_accel_ = declare_parameter<double>("process_noise_accel", 2.0);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.1);
    measurement_noise_position_ = declare_parameter<double>("measurement_noise_position", 0.08);
    min_confirmed_hits_ = declare_parameter<int>("min_confirmed_hits", 3);
    min_dynamic_hits_ = declare_parameter<int>("min_dynamic_hits", 2);
    dynamic_speed_threshold_ = declare_parameter<double>("dynamic_speed_threshold", 0.20);
    velocity_arrow_scale_ = declare_parameter<double>("velocity_arrow_scale", 0.5);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 1.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DynamicTrackerNode::pointCloudCallback, this, std::placeholders::_1));

    tracked_pub_ = create_publisher<predictive_navigation_msgs::msg::TrackedObstacleArray>(
      tracked_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DynamicTrackerNode::watchdogCallback, this));

    RCLCPP_INFO(get_logger(), "Dynamic tracker listening on %s", input_topic_.c_str());
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    last_input_receive_time_ = get_clock()->now();
    last_output_header_ = msg->header;
    input_received_once_ = true;
    stale_timeout_handled_ = false;

    sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
    try {
      const geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
      tf2::doTransform(*msg, transformed_cloud_msg, transform_stamped);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to transform point cloud from %s to %s: %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(transformed_cloud_msg, *raw_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(raw_cloud);
    voxel_filter.setLeafSize(
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_));
    voxel_filter.filter(*cloud_xyz);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_flat(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_flat->reserve(cloud_xyz->size());

    for (const auto & point : cloud_xyz->points) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      pcl::PointXYZ flat_point = point;
      flat_point.z = 0.0f;
      cloud_flat->push_back(flat_point);
    }

    std::vector<Detection> detections = extractDetections(cloud_xyz, cloud_flat);
    updateTracks(detections, rclcpp::Time(transformed_cloud_msg.header.stamp));
    publishTrackedObstacles(transformed_cloud_msg.header);
  }

  void watchdogCallback()
  {
    if (!input_received_once_ || stale_timeout_handled_) {
      return;
    }

    const double silence_seconds = (get_clock()->now() - last_input_receive_time_).seconds();
    if (silence_seconds <= input_timeout_sec_) {
      return;
    }

    if (!tracks_.empty() || !previous_marker_ids_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "No obstacle cloud for %.2f s (> %.2f s). Clearing stale tracked obstacles.",
        silence_seconds, input_timeout_sec_);
    }

    tracks_.clear();

    std_msgs::msg::Header header = last_output_header_;
    header.stamp = get_clock()->now();
    if (header.frame_id.empty()) {
      header.frame_id = target_frame_;
    }
    publishTrackedObstacles(header);
    stale_timeout_handled_ = true;
  }

  std::vector<Detection> extractDetections(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_xyz,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_flat) const
  {
    std::vector<Detection> detections;
    if (cloud_flat->empty()) {
      return detections;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud_flat);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
    cluster_extraction.setClusterTolerance(cluster_tolerance_);
    cluster_extraction.setMinClusterSize(min_cluster_size_);
    cluster_extraction.setMaxClusterSize(max_cluster_size_);
    cluster_extraction.setSearchMethod(tree);
    cluster_extraction.setInputCloud(cloud_flat);
    cluster_extraction.extract(cluster_indices);

    detections.reserve(cluster_indices.size());
    for (const auto & cluster : cluster_indices) {
      if (cluster.indices.empty()) {
        continue;
      }

      Eigen::Vector2d centroid_xy = Eigen::Vector2d::Zero();
      double mean_z = 0.0;
      for (const int index : cluster.indices) {
        const auto & original = cloud_xyz->points[static_cast<std::size_t>(index)];
        centroid_xy.x() += original.x;
        centroid_xy.y() += original.y;
        mean_z += original.z;
      }

      const double cluster_size = static_cast<double>(cluster.indices.size());
      centroid_xy /= cluster_size;
      mean_z /= cluster_size;
      detections.push_back(Detection{centroid_xy, mean_z});
    }

    return detections;
  }

  void updateTracks(const std::vector<Detection> & detections, const rclcpp::Time & current_stamp)
  {

    for (auto & item : tracks_) {
      predictTrack(item.second, current_stamp);
    }

    std::vector<Assignment> assignments = associateDetections(detections);
    std::unordered_set<std::size_t> matched_detections;
    std::unordered_set<int> matched_tracks;

    for (const auto & assignment : assignments) {
      auto track_it = tracks_.find(assignment.track_id);
      if (track_it == tracks_.end()) {
        continue;
      }
      matched_detections.insert(assignment.detection_index);
      matched_tracks.insert(assignment.track_id);
      updateTrack(track_it->second, detections[assignment.detection_index], current_stamp);
    }

    for (auto & item : tracks_) {
      if (matched_tracks.find(item.first) == matched_tracks.end()) {
        item.second.missed_frames += 1;
        item.second.dynamic_hit_frames = 0;
      }
    }

    for (std::size_t i = 0; i < detections.size(); ++i) {
      if (matched_detections.find(i) == matched_detections.end()) {
        createTrack(detections[i], current_stamp);
      }
    }

    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if (it->second.missed_frames > max_missed_frames_) {
        it = tracks_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::vector<Assignment> associateDetections(const std::vector<Detection> & detections) const
  {
    std::vector<Assignment> candidate_assignments;
    for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
      for (const auto & item : tracks_) {
        const auto & track = item.second;
        const Eigen::Vector2d predicted_position(track.state(0), track.state(1));
        const double distance = (detections[detection_index].position_xy - predicted_position).norm();
        if (distance <= association_distance_threshold_) {
          candidate_assignments.push_back(Assignment{detection_index, item.first, distance});
        }
      }
    }

    std::sort(
      candidate_assignments.begin(), candidate_assignments.end(),
      [](const Assignment & lhs, const Assignment & rhs) {
        return lhs.distance < rhs.distance;
      });

    std::unordered_set<std::size_t> used_detections;
    std::unordered_set<int> used_tracks;
    std::vector<Assignment> final_assignments;

    for (const auto & candidate : candidate_assignments) {
      if (used_detections.count(candidate.detection_index) > 0) {
        continue;
      }
      if (used_tracks.count(candidate.track_id) > 0) {
        continue;
      }
      used_detections.insert(candidate.detection_index);
      used_tracks.insert(candidate.track_id);
      final_assignments.push_back(candidate);
    }

    return final_assignments;
  }

  void createTrack(const Detection & detection, const rclcpp::Time & current_stamp)
  {
    Track track;
    track.id = next_track_id_++;
    track.state << detection.position_xy.x(), detection.position_xy.y(), 0.0, 0.0;
    track.covariance = Eigen::Matrix4d::Identity();
    track.covariance(0, 0) = 0.5;
    track.covariance(1, 1) = 0.5;
    track.covariance(2, 2) = 5.0;
    track.covariance(3, 3) = 5.0;
    track.z = detection.z;
    track.last_update_stamp = current_stamp;
    track.hit_frames = 1;
    track.dynamic_hit_frames = 0;
    track.missed_frames = 0;
    tracks_.emplace(track.id, track);
  }

  void predictTrack(Track & track, const rclcpp::Time & current_stamp) const
  {
    double dt = (current_stamp - track.last_update_stamp).seconds();
    if (dt <= 0.0) {
      dt = 1e-3;
    }

    Eigen::Matrix4d transition = Eigen::Matrix4d::Identity();
    transition(0, 2) = dt;
    transition(1, 3) = dt;

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    const double q = process_noise_accel_;

    Eigen::Matrix4d process_noise = Eigen::Matrix4d::Zero();
    process_noise(0, 0) = 0.25 * dt4 * q;
    process_noise(0, 2) = 0.5 * dt3 * q;
    process_noise(1, 1) = 0.25 * dt4 * q;
    process_noise(1, 3) = 0.5 * dt3 * q;
    process_noise(2, 0) = 0.5 * dt3 * q;
    process_noise(2, 2) = dt2 * q;
    process_noise(3, 1) = 0.5 * dt3 * q;
    process_noise(3, 3) = dt2 * q;

    track.state = transition * track.state;
    track.covariance = transition * track.covariance * transition.transpose() + process_noise;
    track.last_update_stamp = current_stamp;
  }

  void updateTrack(Track & track, const Detection & detection, const rclcpp::Time & current_stamp) const
  {
    Eigen::Matrix<double, 2, 4> measurement_matrix;
    measurement_matrix <<
      1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0;

    Eigen::Vector2d measurement = detection.position_xy;
    Eigen::Matrix2d measurement_noise = Eigen::Matrix2d::Identity() * measurement_noise_position_;

    const Eigen::Vector2d innovation = measurement - measurement_matrix * track.state;
    const Eigen::Matrix2d innovation_covariance =
      measurement_matrix * track.covariance * measurement_matrix.transpose() + measurement_noise;
    const Eigen::Matrix<double, 4, 2> kalman_gain =
      track.covariance * measurement_matrix.transpose() * innovation_covariance.inverse();

    track.state = track.state + kalman_gain * innovation;
    track.covariance =
      (Eigen::Matrix4d::Identity() - kalman_gain * measurement_matrix) * track.covariance;
    track.z = detection.z;
    track.last_update_stamp = current_stamp;
    track.hit_frames += 1;
    track.missed_frames = 0;

    const double speed = std::hypot(track.state(2), track.state(3));
    if (speed >= dynamic_speed_threshold_) {
      track.dynamic_hit_frames += 1;
    } else {
      track.dynamic_hit_frames = 0;
    }
  }

  void publishTrackedObstacles(const std_msgs::msg::Header & header)
  {
    predictive_navigation_msgs::msg::TrackedObstacleArray tracked_array;
    tracked_array.header = header;

    std::vector<int> ordered_ids;
    ordered_ids.reserve(tracks_.size());
    for (const auto & item : tracks_) {
      ordered_ids.push_back(item.first);
    }
    std::sort(ordered_ids.begin(), ordered_ids.end());

    tracked_array.obstacles.reserve(ordered_ids.size());
    for (const int track_id : ordered_ids) {
      const auto & track = tracks_.at(track_id);
      if (!shouldPublishTrack(track)) {
        continue;
      }

      predictive_navigation_msgs::msg::TrackedObstacle obstacle;
      obstacle.id = track.id;
      obstacle.position.x = track.state(0);
      obstacle.position.y = track.state(1);
      obstacle.position.z = track.z;
      obstacle.velocity.x = track.state(2);
      obstacle.velocity.y = track.state(3);
      obstacle.velocity.z = 0.0;
      tracked_array.obstacles.push_back(obstacle);
    }

    tracked_pub_->publish(tracked_array);
    publishMarkers(tracked_array);
  }

  void publishMarkers(const predictive_navigation_msgs::msg::TrackedObstacleArray & tracked_array)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    std::unordered_set<int> active_ids;

    for (const auto & obstacle : tracked_array.obstacles) {
      active_ids.insert(obstacle.id);
      marker_array.markers.push_back(makeSphereMarker(tracked_array.header, obstacle));
      marker_array.markers.push_back(makeVelocityMarker(tracked_array.header, obstacle));
      marker_array.markers.push_back(makeTextMarker(tracked_array.header, obstacle));
    }

    for (const int stale_id : previous_marker_ids_) {
      if (active_ids.count(stale_id) > 0) {
        continue;
      }
      marker_array.markers.push_back(makeDeleteMarker("obstacle", stale_id, tracked_array.header));
      marker_array.markers.push_back(makeDeleteMarker("velocity", stale_id, tracked_array.header));
      marker_array.markers.push_back(makeDeleteMarker("label", stale_id, tracked_array.header));
    }

    previous_marker_ids_ = active_ids;
    marker_pub_->publish(marker_array);
  }

  bool shouldPublishTrack(const Track & track) const
  {
    if (track.missed_frames != 0) {
      return false;
    }

    if (track.hit_frames < min_confirmed_hits_) {
      return false;
    }

    if (track.dynamic_hit_frames < min_dynamic_hits_) {
      return false;
    }

    const double speed = std::hypot(track.state(2), track.state(3));
    return speed >= dynamic_speed_threshold_;
  }

  visualization_msgs::msg::Marker makeSphereMarker(
    const std_msgs::msg::Header & header,
    const predictive_navigation_msgs::msg::TrackedObstacle & obstacle) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "obstacle";
    marker.id = obstacle.id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.pose.position = obstacle.position;
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;
    marker.color.a = 0.85;
    marker.color.r = 1.0;
    marker.color.g = 0.2;
    marker.color.b = 0.2;
    return marker;
  }

  visualization_msgs::msg::Marker makeVelocityMarker(
    const std_msgs::msg::Header & header,
    const predictive_navigation_msgs::msg::TrackedObstacle & obstacle) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "velocity";
    marker.id = obstacle.id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.10;
    marker.scale.z = 0.12;
    marker.color.a = 0.95;
    marker.color.r = 0.1;
    marker.color.g = 0.8;
    marker.color.b = 0.2;

    geometry_msgs::msg::Point start = obstacle.position;
    geometry_msgs::msg::Point end = obstacle.position;
    end.x += obstacle.velocity.x * velocity_arrow_scale_;
    end.y += obstacle.velocity.y * velocity_arrow_scale_;
    marker.points.push_back(start);
    marker.points.push_back(end);
    return marker;
  }

  visualization_msgs::msg::Marker makeTextMarker(
    const std_msgs::msg::Header & header,
    const predictive_navigation_msgs::msg::TrackedObstacle & obstacle) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "label";
    marker.id = obstacle.id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.pose.position = obstacle.position;
    marker.pose.position.z += 0.45;
    marker.scale.z = 0.20;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    std::ostringstream text;
    text << "id=" << obstacle.id
         << " v=(" << obstacle.velocity.x << ", " << obstacle.velocity.y << ")";
    marker.text = text.str();
    return marker;
  }

  visualization_msgs::msg::Marker makeDeleteMarker(
    const std::string & ns, int id, const std_msgs::msg::Header & header) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
  }

  std::string input_topic_;
  std::string tracked_topic_;
  std::string marker_topic_;
  std::string target_frame_;
  double cluster_tolerance_{0.35};
  int min_cluster_size_{8};
  int max_cluster_size_{5000};
  double association_distance_threshold_{0.8};
  int max_missed_frames_{5};
  double process_noise_accel_{2.0};
  double voxel_leaf_size_{0.1};
  double measurement_noise_position_{0.08};
  int min_confirmed_hits_{3};
  int min_dynamic_hits_{2};
  double dynamic_speed_threshold_{0.20};
  double velocity_arrow_scale_{0.5};
  double input_timeout_sec_{1.0};
  int next_track_id_{0};
  bool input_received_once_{false};
  bool stale_timeout_handled_{false};
  rclcpp::Time last_input_receive_time_{0, 0, RCL_ROS_TIME};
  std_msgs::msg::Header last_output_header_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<predictive_navigation_msgs::msg::TrackedObstacleArray>::SharedPtr tracked_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::unordered_map<int, Track> tracks_;
  std::unordered_set<int> previous_marker_ids_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
