## 基于 TCPA-DCPA 风险场的全向 DWB 预测导航

#### 1. 理论源泉与模型“缝合”逻辑

**领域 A：目标跟踪雷达系统 (IEEE TAES 级)**

- **摘取模型：** 带有恒速 (CV) 模型的**扩展卡尔曼滤波 (EKF)**。用于平滑聚类后的点云质心，提取高置信度的动态障碍物速度矢量。

**领域 B：航空/航海防碰撞系统 (IEEE T-ITS 级)**

- **摘取模型：** **TCPA (Time to Closest Point of Approach, 最近会遇时间)** 与 **DCPA (Distance at Closest Point of Approach, 最近会遇距离)**。传统机器人只看当前距离，而我们将引入航空领域预测两架飞机是否相撞的核心数学工具。

**领域 C：移动机器人局部控制 (IEEE T-RO 级)**

- **摘取模型：** **DWB 速度空间采样与代价函数 (Critic)**。



#### 2. 核心数学模型推导

在你的论文 Methodology 章节，你需要这样展示你的创新代价函数：

设在预测周期内，机器人的采样位置为 $\mathbf{p}_r$，采样速度为 $\mathbf{v}_r$（全向平移）；动态障碍物的预测位置为 $\mathbf{p}_o$，滤波后的速度为 $\mathbf{v}_o$。

**步骤 1：定义相对运动学态势**

相对位置矢量：$\mathbf{p}_{rel} = \mathbf{p}_o - \mathbf{p}_r$

相对速度矢量：$\mathbf{v}_{rel} = \mathbf{v}_r - \mathbf{v}_o$

**步骤 2：判断是否具有碰撞趋势**

只有当相对速度方向指向相对位置方向时（即内积大于 0），两者才在相互靠近：

$$\text{Approach Trend} = \mathbf{p}_{rel} \cdot \mathbf{v}_{rel} > 0$$

*(如果 $\le 0$，说明在互相远离或平行，该障碍物带来的代价直接记为 0)*

**步骤 3：计算 TCPA 与 DCPA（核心创新）**

如果正在靠近，计算它们达到最近距离所需的时间 (TCPA)：

$$t_{cpa} = \frac{\mathbf{p}_{rel} \cdot \mathbf{v}_{rel}}{||\mathbf{v}_{rel}||^2}$$

计算在 $t_{cpa}$ 时刻，两者之间的最近几何距离 (DCPA)：

$$d_{cpa} = ||\mathbf{p}_{rel} - t_{cpa} \mathbf{v}_{rel}||$$

**步骤 4：构建各向异性时空风险场代价函数 (Cost Function)**

我们将时间紧迫性（$t_{cpa}$）和空间危险性（$d_{cpa}$）用二维高斯势场进行融合：

$$Cost_{dynamic} = \alpha \cdot \exp \left( - \frac{t_{cpa}}{\tau_{safe}} \right) \cdot \exp \left( - \frac{d_{cpa}^2}{2\sigma_{safe}^2} \right)$$

**创新点解释（如何向审稿人吹嘘）：**

- 这不仅考虑了“多久后会撞上”（$t_{cpa}$），还考虑了“最危险的时候离得有多近”（$d_{cpa}$）。
- 它形成了一个**“各向异性（Anisotropic）”**的排斥场。障碍物正前方的排斥力极大，而侧面的排斥力呈高斯衰减。这让你的全向轮机器人敢于在敌人侧面进行极速的“擦肩而过”（侧滑），而不是在远处就傻傻地刹车。



#### 3.导航障碍物读取数据流

  /livox/lidar -> point_lio -> /cloud_registered -> terrain_analysis -> /segmentation/obstacle ->
  pointcloud_to_laserscan -> /scan -> Nav2



#### 4.进展

• 已经按阶段落地并验证了前两步。

  PHASE 1 已创建并编译通过：
  src/predictive_navigation_msgs/package.xml
  src/predictive_navigation_msgs/CMakeLists.txt
  src/predictive_navigation_msgs/msg/TrackedObstacle.msg
  src/predictive_navigation_msgs/msg/TrackedObstacleArray.msg

  PHASE 2 也已经继续完成并编译通过：
  src/predictive_tracker/package.xml
  src/predictive_tracker/CMakeLists.txt
  src/predictive_tracker/src/dynamic_tracker_node.cpp

  当前 tracker 实现了这些内容：

  - 订阅 /terrain_map
  - 用 pcl::EuclideanClusterExtraction 做 2D 聚类
  - 最近邻 + 距离阈值做 ID 关联
  - 常速度 Kalman Filter 估计 x, y, vx, vy
  - 发布 /tracked_obstacles
  - 发布 RViz MarkerArray

  编译验证我实际跑过：

  source /opt/ros/galactic/setup.bash
  colcon build --packages-select predictive_navigation_msgs
  colcon build --packages-select predictive_tracker --packages-up-to predictive_tracker

  结果都成功。没有做运行时 ros topic/rviz 联调，只做了静态编译验证。



• 已经完成两部分，并且都编译通过。

  predictive_tracker 现在有外露参数 YAML：
  src/predictive_tracker/config/dynamic_tracker.yaml

  我同时把它安装到 share 目录了：
  src/predictive_tracker/CMakeLists.txt

  PHASE 3 也已经落地成独立 DWB critic 插件包：
  src/tcpa_dcpa_critic/package.xml
  src/tcpa_dcpa_critic/CMakeLists.txt
  src/tcpa_dcpa_critic/include/tcpa_dcpa_critic/tcpa_dcpa_critic.hpp
  src/tcpa_dcpa_critic/src/tcpa_dcpa_critic.cpp
  src/tcpa_dcpa_critic/tcpa_dcpa_critic.xml

  这个 critic 已实现：

  - 订阅 /tracked_obstacles
  - 用 std::mutex 安全缓存最新障碍物
  - 读取 scale、tau_safe、sigma_safe
  - 按你在 src/PLANS.md 里的 TCPA/DCPA 公式对 Trajectory2D 打分
  - 包含 #include <pluginlib/class_list_macros.hpp> 和 PLUGINLIB_EXPORT_CLASS(...)

  我实际做过验证：

  source /opt/ros/galactic/setup.bash
  colcon build --packages-select predictive_navigation_msgs predictive_tracker tcpa_dcpa_critic

  结果：

  - predictive_navigation_msgs 通过
  - predictive_tracker 通过
  - tcpa_dcpa_critic 通过

  如果你要，我下一步可以直接把这个 critic 接进你当前的 src/rm_navi/rm_navigation/navi/params/nav2_params.yaml，把
  FollowPath.critics 和对应参数段补齐。



 已修改
  Tracker 现在会先把 /terrain_map 点云变换到 odom，再做 VoxelGrid 和聚类：
  src/predictive_tracker/src/dynamic_tracker_node.cpp

  Tracker 的参数 YAML 现在外露了 target_frame: "odom" 和 voxel_leaf_size：
  src/predictive_tracker/config/dynamic_tracker.yaml

  Critic 的 scoreTrajectory() 已改成严格的 O(M)，只在 t=0 对每个障碍物计算一次 TCPA/DCPA：
  src/tcpa_dcpa_critic/src/tcpa_dcpa_critic.cpp

  Nav2 已经把自定义 critic 接进 DWB：
  src/rm_navi/rm_navigation/navi/params/nav2_params.yaml



Nav2 接入
  DWB 现在加了：

  critics: ["Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist", "TCPADCPA"]
  TCPADCPA.plugin: "tcpa_dcpa_critic::TCPADCPACritic"
  TCPADCPA.scale: 10.0
  TCPADCPA.tau_safe: 2.0
  TCPADCPA.sigma_safe: 0.5
  TCPADCPA.tracked_topic: "/tracked_obstacles"

  验证
  我实际验证过：

  - predictive_tracker 编译通过
  - tcpa_dcpa_critic 编译通过
  - src/predictive_tracker/config/dynamic_tracker.yaml 可解析
  - src/rm_navi/rm_navigation/navi/params/nav2_params.yaml 可解析

  如果你要，我下一步可以继续把 predictive_tracker 的 launch 文件也补上，让它直接加载这个 YAML，而不是手动传参。



总体：

• 本轮已经把启动链和 Nav2 接线补齐，并完成验证。

  关键接线

  - src/pre.sh:1 现在已经把 terrain_analysis -> predictive_tracker -> pointcloud_to_laserscan 串进预启动链，实际链
    路是 /livox/lidar -> point_lio -> terrain_analysis(/segmentation/obstacle) -> predictive_tracker(/
    tracked_obstacles, odom) -> pointcloud_to_laserscan(/scan) -> Nav2。
  - src/predictive_tracker/config/dynamic_tracker.yaml:1 已对齐到 terrain_analysis 的真实输出，input_topic 改为 /
    segmentation/obstacle，目标坐标系固定为 odom。
  - src/rm_navi/rm_navigation/navi/launch/navigation_launch.py:10 已修复语法错误，补全了 controller_server 节点定
    义，并把行为树路径改成包内相对路径，不再依赖失效的绝对路径。
  - src/rm_navi/rm_navigation/navi/params/nav2_params.yaml:126 已把 TCPADCPA critic 接进 FollowPath.critics，并补
    齐 plugin/scale/tau_safe/sigma_safe/tracked_topic；同时本地 costmap 的 /scan sensor_frame 已修成 base_link。

  本次代码更新

  - PHASE 1:
      - 新增消息包 src/predictive_navigation_msgs
      - 新增 src/predictive_navigation_msgs/msg/TrackedObstacle.msg
      - 新增 src/predictive_navigation_msgs/msg/TrackedObstacleArray.msg
  - PHASE 2:
      - 新增 tracker 包 src/predictive_tracker
      - src/predictive_tracker/src/dynamic_tracker_node.cpp:64 已加入 Galactic 风格参数声明
      - src/predictive_tracker/src/dynamic_tracker_node.cpp:82 已加入 tf2_ros::Buffer 和 TransformListener
      - src/predictive_tracker/src/dynamic_tracker_node.cpp:97 已在回调里先把点云变换到 odom
      - src/predictive_tracker/src/dynamic_tracker_node.cpp:116 已加入 VoxelGrid 下采样，voxel_leaf_size 参数外露
      - 新增 src/predictive_tracker/launch/dynamic_tracker.launch.py
      - 新增 src/predictive_tracker/config/dynamic_tracker.yaml
  - PHASE 3:
      - 新增 critic 包 src/tcpa_dcpa_critic
      - src/tcpa_dcpa_critic/src/tcpa_dcpa_critic.cpp:57 的 scoreTrajectory() 已改成 O(M)，只在 t=0 计算一次相对运
        动并累计风险场

  验证结果

  - bash -n src/pre.sh src/nav.sh 通过
  - python3 -m py_compile 校验 src/predictive_tracker/launch/dynamic_tracker.launch.py、src/rm_navi/rm_navigation/
    navi/launch/navigation_launch.py、src/rm_navi/rm_navigation/navi/launch/localization_launch.py、src/rm_navi/
    rm_navigation/navi/launch/slam_launch.py 全部通过
  - colcon build --packages-select predictive_navigation_msgs predictive_tracker tcpa_dcpa_critic navi 通过

  现在的启动方式保持不变，先跑 src/pre.sh，再跑 src/nav.sh 即可。另有一处既有未关联改动 src/rm_navi/
  rm_localization/pcd2pgm，我没有碰。



## 论文具体

### 🎨 绘制草案：图层与模块化布局 (Visual Layout)

整张图建议采用**自左向右**或**自上而下**的流水线布局，分为三个核心大框（用带圆角的虚线框包围，并加上浅色背景区分）。

#### 框 1：感知与定位底座 (Perception & Localization)

- **视觉风格**：灰色或浅蓝色背景。
- **组件模块**：
  - [传感器图标] **3D LiDAR & IMU** $\rightarrow$ 产生原始数据。
  - **Point-LIO** (前端) $\rightarrow$ 输出：高频里程计 (High-frequency Odometry)。
  - **ICP Matching** (后端) $\rightarrow$ 输出：全局重定位 (Global Pose)。
  - **Terrain Analysis** $\rightarrow$ 输出：滤除地面的障碍物点云 (Non-ground Cloud)。

#### 框 2：动态追踪流水线 (Dynamic Tracking Pipeline) —— 你的工程亮点

- **视觉风格**：浅绿色或浅橙色背景（突出这是你的核心工作）。
- **组件模块**（按顺序用箭头串联）：
  1. **VoxelGrid Filter** (体素降采样)：用一个小漏斗图标表示算力优化。
  2. **Euclidean Clustering** (欧式聚类)：用几个散点变成包围盒的图标表示。
  3. **Data Association** (数据关联)：标明使用 Greedy/Hungarian 策略。
  4. **EKF (Constant Velocity)**：输出最终的动态目标状态向量 $[x, y, v_x, v_y]^T$。

#### 框 3：各向异性时空规划 (Anisotropic Spatiotemporal Planning) —— 你的学术亮点

- **视觉风格**：浅紫色或强调色背景（这是整篇论文的灵魂）。
- **组件模块**：
  - **Theta\* Global Planner** $\rightarrow$ 接收地图，输出平滑全局路径。
  - **Predictive DWB Local Planner** (大框，内部包含细节)：
    - *Trajectory Sampling* (轨迹采样：仅平移全向采样)。
    - **TCPA-DCPA Risk Evaluator (核心插件)**：画两个箭头汇聚到这里，一个来自机器人的采样速度，一个来自框 2 的障碍物速度。可以在这个模块旁边画一个极小的“高斯势场排斥圈”示意图。
    - *Traditional Critics* (传统评价器：如 GoalDist, PathDist)。
- **最终输出** $\rightarrow$ [机器人底盘图标] 全向速度指令 $(v_x, v_y)$。



当你的图画好后，你需要一段严谨的学术英语在 **"III. PROPOSED METHODOLOGY"** 的第一小节 **"A. System Overview"** 中描述这张图。

你可以直接使用或微调以下这段为你量身定制的英文文本：

**A. System Overview**

The overall architecture of the proposed predictive navigation framework is illustrated in **Fig. 1**. The system is designed for omnidirectional mobile robots operating in highly dynamic and adversarial environments, and it comprises three tightly coupled subsystems: Localization and Perception, Dynamic Target Tracking, and Spatiotemporal Predictive Planning.

Firstly, to ensure robust state estimation under aggressive maneuvers, **Point-LIO** combined with **ICP** matching provides high-frequency odometry and drift-free global localization. The raw 3D point cloud is pre-processed by a terrain analysis module to extract non-ground obstacles.

Secondly, to mitigate the computational burden on edge computing devices (e.g., mini-PCs), a **VoxelGrid filter** is applied to downsample the obstacle cloud before it is fed into the Euclidean clustering algorithm. The extracted spatial centroids are then continuously tracked and smoothed by a **Constant Velocity (CV) Extended Kalman Filter (EKF)**, which accurately estimates the velocity vectors of multiple dynamic obstacles.

Finally, the tracking results are integrated into a customized **Dynamic Window Approach (DWB)** local planner. While a **Theta*** global planner provides a smooth, grid-independent reference path, the core contribution lies within the DWB's evaluation phase. We introduce a novel **Anisotropic Spatiotemporal Risk Field** based on the Time to Closest Point of Approach (TCPA) and the Distance at Closest Point of Approach (DCPA). This allows the omnidirectional chassis to proactively slide past moving obstacles with smooth velocity transitions rather than triggering freezing or abrupt stops.

*(图注/Caption)* **Fig. 1.** Overall system architecture of the proposed predictive navigation framework, highlighting the VoxelGrid-optimized dynamic tracking pipeline and the TCPA-DCPA integrated DWB local planner.
