#include "icp_registration/icp_registration.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/create_timer_ros.h>

namespace icp {

namespace {

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromRotation(const Eigen::Matrix3d &rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

Eigen::Quaterniond quaternionFromYaw(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

Eigen::Matrix4d transformToMatrix(const geometry_msgs::msg::Transform &transform) {
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q(transform.rotation.w, transform.rotation.x,
                       transform.rotation.y, transform.rotation.z);
  out.block<3, 3>(0, 0) = q.toRotationMatrix();
  out(0, 3) = transform.translation.x;
  out(1, 3) = transform.translation.y;
  out(2, 3) = transform.translation.z;
  return out;
}

}  // namespace

IcpNode::IcpNode(const rclcpp::NodeOptions &options)
    : Node("icp_registration", options), rough_iter_(10), refine_iter_(5),
      first_scan_(true) {
  is_ready_ = false;
  cloud_in_ = PointCloudXYZ::Ptr(new PointCloudXYZ);
  // this->declare_parameter("use_sim_time", false);
  double rough_leaf_size = this->declare_parameter("rough_leaf_size", 0.4);
  double refine_leaf_size = this->declare_parameter("refine_leaf_size", 0.1);
  voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size,
                                  rough_leaf_size);
  voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size,
                                   refine_leaf_size);

  pcd_path_ = this->declare_parameter("pcd_path", std::string(""));
  if (!std::filesystem::exists(pcd_path_)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid pcd path: %s", pcd_path_.c_str());
    throw std::runtime_error("Invalid pcd path");
  }
  // Read the pcd file
  pcl::PCDReader reader;
  PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
  reader.read(pcd_path_, *cloud);
  voxel_refine_filter_.setInputCloud(cloud);
  voxel_refine_filter_.filter(*cloud);

  // Add normal to the pointcloud
  refine_map_ = addNorm(cloud);
  PointCloudXYZ::Ptr point_rough(new PointCloudXYZ);
  PointCloudXYZ::Ptr filterd_point_rough(new PointCloudXYZ);
  pcl::copyPointCloud(*refine_map_, *point_rough);
  voxel_rough_filter_.setInputCloud(point_rough);
  voxel_rough_filter_.filter(*filterd_point_rough);
  rough_map_ = addNorm(filterd_point_rough);

  icp_rough_.setMaximumIterations(rough_iter_);
  icp_rough_.setInputTarget(rough_map_);

  icp_refine_.setMaximumIterations(refine_iter_);
  icp_refine_.setInputTarget(refine_map_);

  RCLCPP_INFO(this->get_logger(), "pcd point size: %ld, %ld",
              refine_map_->size(), rough_map_->size());

  // Parameters
  map_frame_id_ = this->declare_parameter("map_frame_id", std::string("map"));
  odom_frame_id_ =
      this->declare_parameter("odom_frame_id", std::string("odom"));
  laser_frame_id_ =
      this->declare_parameter("laser_frame_id", std::string("laser"));
  thresh_ = this->declare_parameter("thresh", 0.15);
  xy_offset_ = this->declare_parameter("xy_offset", 0.2);
  yaw_offset_ = this->declare_parameter("yaw_offset", 30.0) * M_PI / 180.0;
  yaw_resolution_ =
      this->declare_parameter("yaw_resolution", 10.0) * M_PI / 180.0;
  continuous_relocalization_enabled_ =
      this->declare_parameter("continuous_relocalization", true);
  continuous_relocalization_frequency_ =
      this->declare_parameter("continuous_relocalization_frequency", 1.0);
  max_translation_jump_ =
      this->declare_parameter("max_translation_jump", 0.8);
  max_yaw_jump_ = this->declare_parameter("max_yaw_jump_deg", 15.0) *
                  M_PI / 180.0;
  std::vector<double> initial_pose_offset_vec = this->declare_parameter(
      "initial_pose_offset", std::vector<double>{0.0, 0.0, 0.0});
  std::vector<double> initial_pose_vec = this->declare_parameter(
      "initial_pose", std::vector<double>{0, 0, 0, 0, 0, 0});
  try {
    initial_pose_offset_.x() = initial_pose_offset_vec.at(0);
    initial_pose_offset_.y() = initial_pose_offset_vec.at(1);
    initial_pose_offset_.z() = initial_pose_offset_vec.at(2);
    initial_pose_.position.x = initial_pose_vec.at(0);
    initial_pose_.position.y = initial_pose_vec.at(1);
    initial_pose_.position.z = initial_pose_vec.at(2);
    tf2::Quaternion q;
    q.setRPY(initial_pose_vec.at(3), initial_pose_vec.at(4),
             initial_pose_vec.at(5));
  } catch (const std::out_of_range &ex) {
    RCLCPP_ERROR(this->get_logger(),
                 "initial_pose is not a vector with 6 elements, what():%s",
                 ex.what());
  }

  // Set up the pointcloud subscriber
  std::string pointcloud_topic = this->declare_parameter(
      "pointcloud_topic", std::string("/livox/lidar/pointcloud"));
  RCLCPP_INFO(this->get_logger(), "pointcloud_topic: %s",
              pointcloud_topic.c_str());
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, qos,
      std::bind(&IcpNode::pointcloudCallback, this, std::placeholders::_1));
  // Set up the initial pose subscriber
  initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", qos,
          [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            initialPoseCallback(msg);
          });

  // Set up the transform broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (continuous_relocalization_enabled_ &&
      continuous_relocalization_frequency_ > 0.0) {
    const auto period = std::chrono::milliseconds(static_cast<int>(
        std::max(200.0, 1000.0 / continuous_relocalization_frequency_)));
    relocalization_timer_ = this->create_wall_timer(
        period, std::bind(&IcpNode::continuousRelocalizationCallback, this));
  }

  // Set up the timer
  // timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
  //   if (is_ready_) {
  //     map_to_odom_.header.stamp = now();
  //     map_to_odom_.header.frame_id = map_frame_id_;
  //     map_to_odom_.child_frame_id = odom_frame_id_;
  //     tf_broadcaster_->sendTransform(map_to_odom_);
  //   }
  // });

  tf_publisher_thread_ = std::make_unique<std::thread>([this]() {
    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
      {
        std::lock_guard lock(mutex_);
        if (is_ready_) {
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "Publishing tf"
                                 << map_to_odom_.transform.translation.x << " "
                                 << map_to_odom_.transform.translation.y << " "
                                 << map_to_odom_.transform.translation.z);
          map_to_odom_.header.stamp = now();
          map_to_odom_.header.frame_id = map_frame_id_;
          map_to_odom_.child_frame_id = odom_frame_id_;
          tf_broadcaster_->sendTransform(map_to_odom_);
        }
      }
      rate.sleep();
    }
  });

  RCLCPP_INFO(this->get_logger(), "icp_registration initialized");
}

IcpNode::~IcpNode() {
  if (tf_publisher_thread_->joinable()) {
    tf_publisher_thread_->join();
  }
}

void IcpNode::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  PointCloudXYZ incoming_cloud;
  pcl::fromROSMsg(*msg, incoming_cloud);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    *cloud_in_ = incoming_cloud;
  }
  if (first_scan_ || !is_ready_) {
    auto pose_msg =
        std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pose_msg->header = msg->header;
    pose_msg->pose.pose = initial_pose_;
    initialPoseCallback(pose_msg);
    if (is_ready_) {
      first_scan_ = false;
    }
  }
}

void IcpNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> align_lock(align_mutex_);
  PointCloudXYZ::Ptr cloud_snapshot(new PointCloudXYZ);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (cloud_in_->empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Skipping ICP init because no point cloud has been received yet");
      return;
    }
    *cloud_snapshot = *cloud_in_;
  }

  // Set the initial pose
  Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
  pos += initial_pose_offset_;
  Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  Eigen::Matrix4d initial_guess;
  initial_guess.block<3, 3>(0, 0) = q.toRotationMatrix();
  initial_guess.block<3, 1>(0, 3) = pos;
  initial_guess(3, 3) = 1;

  // Align the pointcloud
  RCLCPP_INFO(this->get_logger(), "Aligning the pointcloud");
  RCLCPP_INFO(this->get_logger(), "Applying initial pose offset: %.3f, %.3f, %.3f",
              initial_pose_offset_.x(), initial_pose_offset_.y(), initial_pose_offset_.z());
  Eigen::Matrix4d map_to_laser = multiAlignSync(cloud_snapshot, initial_guess);
  // Eigen::Matrix4d result = align(cloud_in_, initial_guess);
  if (!success_) {
    map_to_laser = initial_guess;
    RCLCPP_ERROR(this->get_logger(), "ICP failed");
  }

  // Eigen::Isometry3d laser_to_odom;
  Eigen::Matrix4d laser_to_odom = Eigen::Matrix4d::Identity();
  try {
    // Get odom to laser transform
    auto transform =
        tf_buffer_->lookupTransform(laser_frame_id_, odom_frame_id_,
                                    tf2::TimePointZero);
    // RCLCPP_INFO(get_logger(), "%s", transform.header.frame_id.c_str());
    Eigen::Vector3d t(transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z);
    Eigen::Quaterniond q(
        transform.transform.rotation.w, transform.transform.rotation.x,
        transform.transform.rotation.y, transform.transform.rotation.z);
    // laser_to_odom.prerotate(q);
    // laser_to_odom.translate(t);
    laser_to_odom.block<3, 3>(0, 0) = q.toRotationMatrix();
    laser_to_odom.block<3, 1>(0, 3) = t;
  } catch (tf2::TransformException &ex) {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    is_ready_ = false;
    return;
  }
  Eigen::Matrix4d result = map_to_laser * laser_to_odom.matrix().cast<double>();
  {
    std::lock_guard lock(mutex_);
    const Eigen::Matrix3d rotation = result.block<3, 3>(0, 0);
    const double yaw = yawFromRotation(rotation);
    q = quaternionFromYaw(yaw);

    map_to_odom_.transform.translation.x = result(0, 3);
    map_to_odom_.transform.translation.y = result(1, 3);
    map_to_odom_.transform.translation.z = 0.0;
    map_to_odom_.transform.rotation.w = q.w();
    map_to_odom_.transform.rotation.x = q.x();
    map_to_odom_.transform.rotation.y = q.y();
    map_to_odom_.transform.rotation.z = q.z();
    is_ready_ = true;
  }
}

void IcpNode::continuousRelocalizationCallback() {
  if (!continuous_relocalization_enabled_ || first_scan_) {
    return;
  }

  std::unique_lock<std::mutex> align_lock(align_mutex_, std::try_to_lock);
  if (!align_lock.owns_lock()) {
    return;
  }

  PointCloudXYZ::Ptr cloud_snapshot(new PointCloudXYZ);
  geometry_msgs::msg::TransformStamped current_map_to_odom;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_ready_ || cloud_in_->empty()) {
      return;
    }
    *cloud_snapshot = *cloud_in_;
    current_map_to_odom = map_to_odom_;
  }

  Eigen::Matrix4d laser_to_odom = Eigen::Matrix4d::Identity();
  try {
    auto transform =
        tf_buffer_->lookupTransform(laser_frame_id_, odom_frame_id_,
                                    tf2::TimePointZero);
    Eigen::Vector3d t(transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z);
    Eigen::Quaterniond q(transform.transform.rotation.w,
                         transform.transform.rotation.x,
                         transform.transform.rotation.y,
                         transform.transform.rotation.z);
    laser_to_odom.block<3, 3>(0, 0) = q.toRotationMatrix();
    laser_to_odom.block<3, 1>(0, 3) = t;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Skipping continuous ICP: %s", ex.what());
    return;
  }

  const Eigen::Matrix4d current_map_to_odom_matrix =
      transformToMatrix(current_map_to_odom.transform);
  const Eigen::Matrix4d initial_guess =
      current_map_to_odom_matrix * laser_to_odom.inverse();

  Eigen::Matrix4d map_to_laser = multiAlignSync(cloud_snapshot, initial_guess);
  if (!success_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Continuous ICP update rejected because alignment failed");
    return;
  }

  const Eigen::Matrix4d candidate_map_to_odom =
      map_to_laser * laser_to_odom.matrix().cast<double>();
  const double dx =
      candidate_map_to_odom(0, 3) - current_map_to_odom_matrix(0, 3);
  const double dy =
      candidate_map_to_odom(1, 3) - current_map_to_odom_matrix(1, 3);
  const double translation_jump = std::hypot(dx, dy);
  const double current_yaw =
      yawFromRotation(current_map_to_odom_matrix.block<3, 3>(0, 0));
  const double candidate_yaw =
      yawFromRotation(candidate_map_to_odom.block<3, 3>(0, 0));
  const double yaw_jump = std::abs(normalizeAngle(candidate_yaw - current_yaw));

  if (translation_jump > max_translation_jump_ || yaw_jump > max_yaw_jump_) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Rejecting continuous ICP jump: dxy=%.3f m dyaw=%.3f rad score=%.4f",
        translation_jump, yaw_jump, score_);
    return;
  }

  const Eigen::Quaterniond planar_q = quaternionFromYaw(candidate_yaw);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_to_odom_.transform.translation.x = candidate_map_to_odom(0, 3);
    map_to_odom_.transform.translation.y = candidate_map_to_odom(1, 3);
    map_to_odom_.transform.translation.z = 0.0;
    map_to_odom_.transform.rotation.w = planar_q.w();
    map_to_odom_.transform.rotation.x = planar_q.x();
    map_to_odom_.transform.rotation.y = planar_q.y();
    map_to_odom_.transform.rotation.z = planar_q.z();
    is_ready_ = true;
  }
}

// Eigen::Matrix4d IcpNode::align(PointCloudXYZI::Ptr source,
//                                const Eigen::Matrix4d &init_guess) {
//   success_ = false;
//   // Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);

//   pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(
//       new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(
//       new pcl::PointCloud<pcl::PointXYZI>);

//   voxel_rough_filter_.setInputCloud(source);
//   voxel_rough_filter_.filter(*rough_source);
//   voxel_refine_filter_.setInputCloud(source);
//   voxel_refine_filter_.filter(*refine_source);

//   PointCloudXYZIN::Ptr rough_source_norm = addNorm(rough_source);
//   PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);
//   PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);
//   auto tic = std::chrono::system_clock::now();
//   icp_rough_.setInputSource(rough_source_norm);
//   icp_rough_.align(*align_point, init_guess.cast<float>());

//   score_ = icp_rough_.getFitnessScore();
//   if (!icp_rough_.hasConverged())
//     return Eigen::Matrix4d::Zero();

//   icp_refine_.setInputSource(refine_source_norm);
//   icp_refine_.align(*align_point, icp_rough_.getFinalTransformation());
//   score_ = icp_refine_.getFitnessScore();

//   if (!icp_refine_.hasConverged())
//     return Eigen::Matrix4d::Zero();
//   if (score_ > thresh_)
//     return Eigen::Matrix4d::Zero();
//   success_ = true;
//   auto toc = std::chrono::system_clock::now();
//   std::chrono::duration<double> duration = toc - tic;
//   RCLCPP_INFO(this->get_logger(), "align used: %f ms", duration.count() *
//   1000); RCLCPP_INFO(this->get_logger(), "score: %f", score_);

//   return icp_refine_.getFinalTransformation().cast<double>();
// }

Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZ::Ptr source,
                                        const Eigen::Matrix4d &init_guess) {
  static auto rotate2rpy = [](Eigen::Matrix3d &rot) -> Eigen::Vector3d {
    double roll = std::atan2(rot(2, 1), rot(2, 2));
    double pitch = asin(-rot(2, 0));
    double yaw = std::atan2(rot(1, 0), rot(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw);
  };

  success_ = false;
  Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
  Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);
  Eigen::Vector3d rpy = rotate2rpy(rotation);
  Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());
  std::vector<Eigen::Matrix4f> candidates;
  Eigen::Matrix4f temp_pose;
  const int yaw_steps =
      yaw_resolution_ > 1e-6 ? static_cast<int>(std::round(yaw_offset_ / yaw_resolution_)) : 0;

  RCLCPP_INFO(this->get_logger(), "initial guess: %f, %f, %f, %f, %f, %f",
              xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));
  RCLCPP_INFO(this->get_logger(), "ICP search windows: xy_offset=%.3f yaw_steps=%d yaw_resolution=%.3f rad",
              xy_offset_, yaw_steps, yaw_resolution_);

  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      for (int k = -yaw_steps; k <= yaw_steps; k++) {
        Eigen::Vector3f pos(xyz(0) + i * xy_offset_, xyz(1) + j * xy_offset_,
                            xyz(2));
        Eigen::AngleAxisf yawAngle(rpy(2) + k * yaw_resolution_,
                                   Eigen::Vector3f::UnitZ());
        temp_pose.setIdentity();
        temp_pose.block<3, 3>(0, 0) =
            (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
        temp_pose.block<3, 1>(0, 3) = pos;
        candidates.push_back(temp_pose);
      }
    }
  }
  PointCloudXYZ::Ptr rough_source(new PointCloudXYZ);
  PointCloudXYZ::Ptr refine_source(new PointCloudXYZ);

  voxel_rough_filter_.setInputCloud(source);
  voxel_rough_filter_.filter(*rough_source);
  voxel_refine_filter_.setInputCloud(source);
  voxel_refine_filter_.filter(*refine_source);

  PointCloudXYZN::Ptr rough_source_norm = addNorm(rough_source);
  PointCloudXYZN::Ptr refine_source_norm = addNorm(refine_source);
  PointCloudXYZN::Ptr align_point(new PointCloudXYZN);

  Eigen::Matrix4f best_rough_transform;
  double best_rough_score = 10.0;
  bool rough_converge = false;
  auto tic = std::chrono::system_clock::now();
  for (Eigen::Matrix4f &init_pose : candidates) {
    icp_rough_.setInputSource(rough_source_norm);
    icp_rough_.align(*align_point, init_pose);
    if (!icp_rough_.hasConverged())
      continue;
    double rough_score = icp_rough_.getFitnessScore();
    if (rough_score > 2 * thresh_)
      continue;
    if (rough_score < best_rough_score) {
      best_rough_score = rough_score;
      rough_converge = true;
      best_rough_transform = icp_rough_.getFinalTransformation();
    }
  }

  if (!rough_converge)
    return Eigen::Matrix4d::Zero();

  icp_refine_.setInputSource(refine_source_norm);
  icp_refine_.align(*align_point, best_rough_transform);
  score_ = icp_refine_.getFitnessScore();

  if (!icp_refine_.hasConverged())
    return Eigen::Matrix4d::Zero();
  if (score_ > thresh_)
    return Eigen::Matrix4d::Zero();
  success_ = true;
  auto toc = std::chrono::system_clock::now();
  std::chrono::duration<double> duration = toc - tic;
  RCLCPP_INFO(this->get_logger(), "align used: %f ms", duration.count() * 1000);
  RCLCPP_INFO(this->get_logger(), "score: %f", score_);

  return icp_refine_.getFinalTransformation().cast<double>();
}

PointCloudXYZN::Ptr
IcpNode::addNorm(PointCloudXYZ::Ptr cloud) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  searchTree->setInputCloud(cloud);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(15);
  normalEstimator.compute(*normals);
  PointCloudXYZN::Ptr out(new PointCloudXYZN);
  pcl::concatenateFields(*cloud, *normals, *out);
  return out;
}

} // namespace icp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(icp::IcpNode)
