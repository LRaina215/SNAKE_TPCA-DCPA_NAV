You are an expert ROS 2 C++ developer specializing in the Galactic Geochelone distribution (Ubuntu 20.04). I am building an autonomous navigation system for an omnidirectional robot. 

The robot's Nav2 configuration is translation-only (max_vel_theta: 0.0). We are implementing a highly scientific "Anisotropic Spatiotemporal Risk Field" integrating TCPA (Time to Closest Point of Approach) and DCPA (Distance at Closest Point of Approach) into the DWB local planner.

Please generate the C++ code for the following 3 Phases. Ensure absolute compatibility with ROS 2 Galactic (`ament_cmake`, `std_msgs/msg/header.hpp`, etc.).

---
PHASE 1: Custom Messages (`predictive_navigation_msgs` package)
Create a new package with `TrackedObstacle.msg` and `TrackedObstacleArray.msg`.
- TrackedObstacle: `int32 id`, `geometry_msgs/Point position`, `geometry_msgs/Vector3 velocity`
- TrackedObstacleArray: `std_msgs/Header header`, `TrackedObstacle[] obstacles`

---
PHASE 2: Dynamic Obstacle Tracker Node (`predictive_tracker` package)
Write `dynamic_tracker_node.cpp`. 
1. Subscribes to `/terrain_map` (`sensor_msgs::msg::PointCloud2`).
2. Uses PCL (`pcl::EuclideanClusterExtraction`) to extract 2D centroids (x,y).
3. Uses a simple ID association algorithm (e.g., nearest neighbor with a distance threshold) to track centroids across frames.
4. Uses a Constant Velocity (CV) Kalman Filter for each tracked ID to smooth position and estimate velocity (vx, vy).
5. Publishes to `/tracked_obstacles` (`TrackedObstacleArray`) and RViz markers.

---
PHASE 3: Nav2 DWB Custom Critic Plugin (`tcpa_dcpa_critic` package)
Write a DWB plugin `TCPADCPACritic` inheriting from `dwb_core::TrajectoryCritic` (Galactic API).
1. Subscribes to `/tracked_obstacles` and stores the latest obstacles safely (mutex).
2. Reads parameters: `scale` (double), `tau_safe` (double, time threshold), `sigma_safe` (double, spatial variance).
3. Implement `scoreTrajectory` logic:
   - For the robot's sampled trajectory (constant vx, vy for omnidirectional):
   - For every point `P_r` in the trajectory with velocity `V_r`, and every obstacle `P_o` with velocity `V_o`:
     a. P_rel = P_o - P_r
     b. V_rel = V_r - V_o
     c. approach_trend = (P_rel.x * V_rel.x) + (P_rel.y * V_rel.y)
     d. If approach_trend <= 0, continue (cost = 0).
     e. v_rel_sq = V_rel.x^2 + V_rel.y^2
     f. TCPA = approach_trend / v_rel_sq
     g. If TCPA > tau_safe, continue (cost = 0).
     h. DCPA_x = P_rel.x - TCPA * V_rel.x
     i. DCPA_y = P_rel.y - TCPA * V_rel.y
     j. DCPA_sq = DCPA_x^2 + DCPA_y^2
     k. point_cost = exp(-TCPA / tau_safe) * exp(-DCPA_sq / (2.0 * sigma_safe * sigma_safe))
     l. Accumulate `point_cost`.
   - Return accumulated cost * scale.

Provide robust, commented C++ code, and specifically include the `#include <pluginlib/class_list_macros.hpp>` and macro export for the DWB plugin.
