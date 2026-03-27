#ifndef ICP_REGISTRATION_HPP
#define ICP_REGISTRATION_HPP

// std
#include <filesystem>
#include <mutex>

// ros
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/impl/point_types.hpp>

namespace icp {
using PointType = pcl::PointNormal;
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZN = pcl::PointCloud<pcl::PointNormal>;

// Get initial map to odom pose estimation using ICP algorithm
class IcpNode : public rclcpp::Node {
public:
  IcpNode(const rclcpp::NodeOptions &options);
  ~IcpNode();
private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void initialPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  static PointCloudXYZN::Ptr addNorm(PointCloudXYZ::Ptr cloud);

  // Eigen::Matrix4d align(PointCloudXYZ::Ptr source,
  //                       const Eigen::Matrix4d &init_guess);

  Eigen::Matrix4d multiAlignSync(PointCloudXYZ::Ptr source,
                                 const Eigen::Matrix4d &init_guess);

  // ROS2 part
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  // rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  std::unique_ptr<std::thread> tf_publisher_thread_;

  // Voxelfilter used to downsample the pointcloud
  pcl::VoxelGrid<pcl::PointXYZ> voxel_rough_filter_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_refine_filter_;

  // ICP
  int rough_iter_;
  int refine_iter_;
  pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_rough_;
  pcl::IterativeClosestPointWithNormals<PointType, PointType> icp_refine_;

  // Store
  PointCloudXYZ::Ptr cloud_in_;
  PointCloudXYZN::Ptr refine_map_;
  PointCloudXYZN::Ptr rough_map_;
  geometry_msgs::msg::TransformStamped map_to_odom_;
  std::filesystem::path pcd_path_;
  std::string map_frame_id_, odom_frame_id_, laser_frame_id_;
  bool success_;
  double score_;
  double thresh_;
  double xy_offset_;
  double yaw_offset_;
  double yaw_resolution_;
  Eigen::Vector3d initial_pose_offset_{Eigen::Vector3d::Zero()};
  geometry_msgs::msg::Pose initial_pose_;

  bool is_ready_;
  bool first_scan_;
};
} // namespace icp
#endif
