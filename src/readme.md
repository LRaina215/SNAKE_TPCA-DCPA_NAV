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

