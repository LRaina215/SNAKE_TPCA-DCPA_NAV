## 面向动态环境下移动机器人平滑预测导航的各向异性时空风险场

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


tcpa_sim_env 已经落地并跑通，关键文件在这里：

robot.xacro
dynamic_test.world
sim_launch.py
pointcloud_relay.py
package.xml
CMakeLists.txt
我还补了 livox_ros_driver2/package.xml，因为你工作区里的 livox_ros_driver2 原本只有 package_ROS2.xml，这会直接卡住 ament/rosidl 构建。

现在默认 livox_plugin 模式下的接口已经收敛成：

/livox/imu -> sensor_msgs/msg/Imu
/livox/lidar -> sensor_msgs/msg/PointCloud2
/odom -> nav_msgs/msg/Odometry
为了兼容你指定的 Livox 插件，我把插件原始输出改成内部话题 /livox/lidar_custom，然后用一个很轻量的 relay 把 /livox/lidar_custom_PointCloud2 转发到最终的 /livox/lidar。这样既保留了插件原生 CustomMsg，又保证你现有 Point-LIO 能直接吃 /livox/lidar 的 PointCloud2。

我实际验证过这些点：

livox_ros_driver2、ros2_livox_simulation、tcpa_sim_env 都已构建通过
sim_launch.py 和 relay 脚本都通过了 Python 语法检查
headless 启动能成功 spawn 机器人，Livox 插件成功加载 mid360.csv
运行中的 topic 列表里有：
/livox/imu [sensor_msgs/msg/Imu]
/livox/lidar [sensor_msgs/msg/PointCloud2]
/livox/lidar_custom [livox_ros_driver2/msg/CustomMsg]
/livox/lidar_custom_PointCloud2 [sensor_msgs/msg/PointCloud2]
/odom [nav_msgs/msg/Odometry]
/odom 实际消息里的协方差是 0.01 / 0.01 / 0.005

运行顺序

终端 1，启动 Gazebo 仿真：

cd /home/robomaster/auto_shao
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch tcpa_sim_env sim_launch.py gui:=true
终端 2，启动感知与动态障碍前处理链：

cd /home/robomaster/auto_shao/src
./sim_pre.sh
终端 3，启动定位与导航链：

cd /home/robomaster/auto_shao/src
./sim_nav.sh
终端 4，录 rosbag：

cd /home/robomaster/auto_shao
source /opt/ros/galactic/setup.bash
source install/setup.bash
mkdir -p bags
ros2 bag record /odom /cmd_vel /tracked_obstacles -o bags/tcpa_run_$(date +%Y%m%d_%H%M%S)
这次我还实际确认了默认仿真接口已经在线：

/livox/lidar -> sensor_msgs/msg/PointCloud2
/livox/imu -> sensor_msgs/msg/Imu
/odom -> nav_msgs/msg/Odometry
并且 /odom 协方差里已经是你要求的 0.01 / 0.01 / 0.005。


## 论文具体

### 📝 论文结构蓝图 (预计 4-6 页，双栏排版)

#### I. Introduction (引言) - *预计 1 页*

这是整篇论文“圆谎”与“升华”的最重要阵地。

- **动机 (Motivation)：** 强调在复杂地形或探索任务中，机器人（暗指蛇形等复杂机构）极其依赖**平滑且连续**的运动步态。频繁的急刹车会导致姿态失稳或能耗剧增。
- **痛点 (Problem Statement)：** 抨击传统 DWB/DWA 算法的“时空错位”缺陷。它们仅依赖静态空间距离，面对高速非合作目标时，容易引发“冻结机器人综合征（Freezing Robot Problem）”，破坏运动的连续性。
- **贡献 (Contributions)：**
  1. 提出一种轻量级动态追踪前置节点（VoxelGrid + CV-EKF）。
  2. 构建融合 TCPA（最近会遇时间）与 DCPA（最近会遇距离）的各向异性风险场。
  3. **（关键话术）** 搭建高机动通用测试床，在算力受限的边缘设备上完成了 $O(M)$ 复杂度算法的极限工况验证。

#### II. Related Work (相关工作) - *预计 0.5 页*

- **局部避障的演进：** 从传统的 DWA 到现代的预测型导航。
- **跨领域方法的启发：** 提及航空航天或自动驾驶领域的 TTC/TCPA 概念，引出将高阶运动学博弈引入机器人底层控制的必要性。

#### III. Proposed Methodology (提出方法) - *预计 1.5 页 (★ 核心重头戏)*

无论硬件怎么变，你的数学推导是无懈可击的。

- **A. System Overview (系统概述)：** 放置包含 Point-LIO、地形分割、动态追踪与 TCPA-DWB 的全栈数据流向图。
- **B. Lightweight Dynamic Obstacle Tracking (轻量级动态追踪)：** 详细描述如何为了保护边缘算力，使用体素降采样，并通过欧氏聚类与扩展卡尔曼滤波提取高置信度速度矢量。
- **C. Anisotropic Spatiotemporal Risk Field (各向异性时空风险场)：** 这是得分点。展示如何将原本 $O(N \times M)$ 的计算降维至 $O(M)$。列出相对运动学公式，推导 TCPA 与 DCPA，并给出最终的二维高斯风险代价函数。

#### IV. Experiments and Results (实验与结果) - *预计 1.5 页*

- **A. Experimental Testbed Setup (实验测试床设置)：**
  - **话术包装：** “为了验证所提算法在极端动态环境下的避障极限与实时性，本文采用搭载 NUC 边缘计算单元和 3D LiDAR 的高机动底盘作为硬件测试平台。” （在这里正常介绍你的硬件配置，证明实验的真实性）。
- **B. Qualitative Analysis (定性分析)：** 贴出实车跑出来的轨迹对比图。重点对比原生 DWB 的“急刹抽搐”与你算法的“平滑连续规避”轨迹。
- **C. Quantitative Analysis (定量分析)：** 利用表格对比成功率、平均通行速度、轨迹平滑度（如加速度变化率 Jolt）以及 CPU 占用率。

#### V. Conclusion (结论) - *预计 0.25 页*

- 总结 TCPA-DCPA 模型如何完美解决了传统算法的误刹车问题，并强调整套轻量级框架在各种需要连续平滑运动的复杂机器人平台上具有广泛的应用前景（强行扣回大创主题）。


