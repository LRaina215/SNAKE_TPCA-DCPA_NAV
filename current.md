\documentclass[conference]{IEEEtran}
\IEEEoverridecommandlockouts
% The preceding line is only needed to identify funding in the first footnote. If that is unneeded, please comment it out.
%Template version as of 6/27/2024

\usepackage{cite}
\usepackage{amsmath,amssymb,amsfonts}
\usepackage{algorithmic}
\usepackage{graphicx}
\usepackage{textcomp}
\usepackage{xcolor}
\def\BibTeX{{\rm B\kern-.05em{\sc i\kern-.025em b}\kern-.08em
		T\kern-.1667em\lower.7ex\hbox{E}\kern-.125emX}}
\begin{document}
	
	\title{Anisotropic Spatiotemporal Risk Field for Smooth Predictive Navigation of Mobile Robots in Dynamic Environments\\
		\thanks{Identify applicable funding agency here. If none, delete this.}
	}
	
	\author{\IEEEauthorblockN{1\textsuperscript{st} Given Name Surname}
		\IEEEauthorblockA{\textit{dept. name of organization (of Aff.)} \\
			\textit{name of organization (of Aff.)}\\
			City, Country \\
			email address or ORCID}
		\and
		\IEEEauthorblockN{2\textsuperscript{nd} Given Name Surname}
		\IEEEauthorblockA{\textit{dept. name of organization (of Aff.)} \\
			\textit{name of organization (of Aff.)}\\
			City, Country \\
			email address or ORCID}
	}
	
	\maketitle
	
	\begin{abstract}
		This document is a placeholder for the abstract. The abstract will summarize the motivation (smooth navigation for complex robots), the proposed TCPA-DCPA risk field methodology, and the key experimental results on the mobile testbed.
	\end{abstract}
	
	\begin{IEEEkeywords}
		Mobile Robots, Predictive Navigation, Obstacle Avoidance, Dynamic Environments, Spatiotemporal Risk Field.
	\end{IEEEkeywords}
	
	\section{Introduction}
	Navigating safely and efficiently in highly dynamic and cluttered environments remains a critical challenge for mobile robots, particularly for those deployed in complex exploration tasks. Robots operating in such scenarios---including highly articulated platforms that rely on continuous locomotion gaits---require navigation frameworks that provide not only collision-free paths but also extremely smooth velocity transitions. Abrupt stops or jerky maneuvers in dynamic environments can severely disrupt a robot's kinematic rhythm, decrease energy efficiency, and lead to mission failure. Therefore, generating proactive and continuous obstacle avoidance trajectories is of paramount importance.
	
	Traditional local trajectory planners, such as the Dynamic Window Approach (DWA) and its ROS 2 variant (DWB), have been widely adopted for their computational efficiency. However, they inherently rely on static spatial distance thresholds for obstacle evaluation. This leads to a severe ``spatiotemporal misalignment'' in highly dynamic scenarios. For instance, when a dynamic obstacle is passing closely but moving parallel to or away from the robot, traditional planners conservatively assign an infinite cost based solely on proximity. This over-reaction frequently triggers the ``freezing robot problem,'' forcing the robot to execute unnecessary emergency brakes rather than executing smooth evasive maneuvers.
	
	To address these limitations and satisfy the strict requirements for smooth continuous motion, this paper proposes a novel predictive local planning framework based on an Anisotropic Spatiotemporal Risk Field. Inspired by collision avoidance strategies in aerospace systems, we integrate the Time to Closest Point of Approach (TCPA) and the Distance at Closest Point of Approach (DCPA) directly into the trajectory evaluation loop. By actively predicting the velocity vectors of dynamic obstacles via a lightweight Extended Kalman Filter (EKF) and VoxelGrid optimization, the proposed framework replaces the binary collision check with a continuous spatial-temporal risk gradient. To rigorously validate the robustness and real-time computational efficiency of the proposed algorithm under extreme dynamic conditions, comprehensive hardware experiments were conducted on a highly maneuverable wheel-based mobile platform equipped with edge-computing hardware.
	
	\section{Related Work}
	
	\subsection{Local Trajectory Planning}
	Traditional local planners, such as the Dynamic Window Approach (DWA) and the Time Elastic Band (TEB), have been extensively deployed in mobile robotics. DWA samples velocities and evaluates them based on clearance to the nearest static obstacles, while TEB optimizes a sequence of poses considering kinematic constraints. However, in highly dynamic environments, these spatial-only evaluation strategies often struggle. They lack explicit time-domain prediction, frequently triggering the freezing robot problem where the robot unnecessarily halts to avoid moving obstacles that are in close proximity but moving away.
	
	\subsection{Predictive Collision Avoidance}
	To address dynamic obstacles, predictive methods such as Velocity Obstacles (VO) and Model Predictive Control (MPC) have been proposed. While VO computes a set of velocities that lead to a collision, it typically assumes linear obstacle trajectories and struggles in dense, highly cluttered scenarios. MPC provides excellent kinematic constraint satisfaction and receding horizon optimization, but its high computational overhead limits its deployment on resource-constrained edge-computing devices. Our approach introduces a lightweight spatiotemporal risk field into the computationally efficient DWB planner, achieving predictive avoidance without the heavy computational burden of MPC.
	
	\section{Proposed Methodology}
	
	\subsection{System Overview}
	The overall architecture of the proposed predictive navigation framework is illustrated in Fig. \ref{fig:architecture}. The system is specifically designed for highly maneuverable mobile robots operating in cluttered and dynamic environments, where continuous and smooth locomotion is strictly required. The framework consists of three tightly coupled subsystems: Localization and Perception, Lightweight Dynamic Tracking, and Spatiotemporal Predictive Planning.
	
	\begin{figure}[htbp]
		\centerline{\includegraphics[width=\linewidth]{TCPA-DCPA.drawio.png}}
		\caption{Overall architecture of the proposed predictive navigation framework. The system integrates a VoxelGrid-optimized dynamic tracking pipeline with a TCPA-DCPA enhanced local planner to achieve smooth evasive maneuvers.}
		\label{fig:architecture}
	\end{figure}
	
	Firstly, to ensure robust state estimation under aggressive maneuvers, the perception module utilizes 3D LiDAR point clouds. High-frequency odometry and drift-free global localization are provided by an integrated Lidar-Inertial Odometry (LIO) and Iterative Closest Point (ICP) matching system. The raw point clouds are subsequently pre-processed by a terrain analysis module to extract non-ground obstacles, constructing a reliable spatial representation of the surroundings.
	
	Secondly, to capture the dynamic nature of the environment without exhausting the computational resources of edge devices, a lightweight dynamic tracking pipeline is introduced. The extracted obstacle cloud is aggressively downsampled via a VoxelGrid filter. Spatial centroids are then extracted through Euclidean clustering and continuously tracked using a Constant Velocity Extended Kalman Filter (CV-EKF). This subsystem actively estimates and predicts the velocity vectors of multiple non-cooperative targets.
	
	Finally, the tracking results are integrated into a customized local planner. While a Theta* global planner provides a smooth, grid-independent reference path, the core contribution lies within the local trajectory evaluation phase. By injecting the tracked velocities into the evaluation loop, we construct an Anisotropic Spatiotemporal Risk Field. This allows the mobile base to proactively generate smooth, sliding trajectories around moving obstacles rather than triggering abrupt emergency stops.
	
	\subsection{Lightweight Dynamic Obstacle Tracking}
	Accurate velocity estimation of non-cooperative dynamic obstacles is a prerequisite for proactive collision avoidance. However, processing dense 3D point clouds on resource-constrained edge computing devices often leads to severe computational bottlenecks and high latency. To ensure real-time performance, we propose a lightweight tracking pipeline.
	
	First, the non-ground obstacle cloud extracted from the terrain analysis module is aggressively downsampled using a VoxelGrid filter. This step preserves the macroscopic geometric structure of the obstacles while reducing the point density by over 90\%. Subsequently, Euclidean Cluster Extraction is applied to segment the sparse point cloud into individual obstacle clusters, yielding the spatial centroid $\mathbf{z}_k = [x_k, y_k]^T$ of each target at frame $k$.
	
	To maintain track continuity across consecutive frames, a nearest-neighbor data association strategy based on the Euclidean distance matrix is employed. Once associated, the kinematic state of each dynamic obstacle is smoothed and predicted utilizing an Extended Kalman Filter (EKF) with a Constant Velocity (CV) model. The state vector is defined as $\mathbf{x}_k = [x_k, y_k, v_{x,k}, v_{y,k}]^T$, where $(x_k, y_k)$ is the position and $(v_{x,k}, v_{y,k})$ represents the velocity components in the global frame. The measurement model simply updates the positional components:
	\begin{equation}
		\mathbf{z}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k
	\end{equation}
	where $\mathbf{H}$ is the measurement matrix mapping the state space to the observation space, and $\mathbf{v}_k$ represents the Gaussian measurement noise.
	
	Through this lightweight pipeline, the framework robustly outputs the filtered velocity vector $\mathbf{v}_o = [v_{ox}, v_{oy}]^T$ and position $\mathbf{p}_o$ for each tracked target. These variables are subsequently fed directly into the local planner's risk evaluation loop.
	
	\subsection{Anisotropic Spatiotemporal Risk Field}
	Traditional local planners rely heavily on static spatial distance limits, which inherently cause spatiotemporal misalignment when facing high-speed non-cooperative targets. To evaluate the true collision risk proactively, we project the trajectories of both the robot and the dynamic obstacles into the time domain.
	
	Let $\mathbf{p}_r$ and $\mathbf{v}_r$ denote the sampled position and velocity of the robot, respectively. Similarly, let $\mathbf{p}_o$ and $\mathbf{v}_o$ represent the predicted position and velocity of a dynamic obstacle at time $t=0$. The relative position $\mathbf{p}_{rel}$ and relative velocity $\mathbf{v}_{rel}$ are defined as:
	\begin{equation}
		\mathbf{p}_{rel} = \mathbf{p}_o - \mathbf{p}_r, \quad \mathbf{v}_{rel} = \mathbf{v}_r - \mathbf{v}_o
	\end{equation}
	
	An approaching trend exists strictly when the inner product of the relative vectors is positive:
	\begin{equation}
		\mathbf{p}_{rel} \cdot \mathbf{v}_{rel} > 0
	\end{equation}
	If this condition is not met, the obstacle is moving away or parallel to the robot, yielding a zero risk cost. For approaching targets, the Time to Closest Point of Approach (TCPA), denoted as $t_{cpa}$, is calculated by:
	\begin{equation}
		t_{cpa} = \frac{\mathbf{p}_{rel} \cdot \mathbf{v}_{rel}}{\|\mathbf{v}_{rel}\|^2}
	\end{equation}
	
	Subsequently, the Distance at Closest Point of Approach (DCPA), denoted as $d_{cpa}$, represents the minimum geometric clearance at $t_{cpa}$:
	\begin{equation}
		d_{cpa} = \|\mathbf{p}_{rel} - t_{cpa}\mathbf{v}_{rel}\|
	\end{equation}
	
	To generate smooth evasive maneuvers and prevent the freezing robot problem, we formulate an anisotropic spatiotemporal risk field. The dynamic cost $C_{dyn}$ for a sampled velocity command is defined as a bivariate Gaussian function of $t_{cpa}$ and $d_{cpa}$:
	\begin{equation}
		C_{dyn} = \alpha \exp\left(-\frac{t_{cpa}}{\tau_{safe}}\right) \exp\left(-\frac{d_{cpa}^2}{2\sigma_{safe}^2}\right)
	\end{equation}
	where $\alpha$ is a scaling factor, $\tau_{safe}$ is the time prediction threshold, and $\sigma_{safe}$ dictates the spatial safety variance. This formulation naturally generates a continuous risk gradient, enabling the highly maneuverable robot to perform seamless sliding behaviors around dynamic obstacles.
	
	\section{Experiments and Results}
	
	\subsection{Experimental Testbed Setup}
	To rigorously validate the robustness and real-time computational efficiency of the proposed predictive navigation framework, comprehensive experiments were conducted. The hardware testbed features a highly maneuverable wheel-based mobile platform. The perception suite comprises a Livox 3D LiDAR and an inertial measurement unit (IMU). All proposed algorithms, including the VoxelGrid tracking pipeline and the TCPA-DCPA enhanced DWB planner, are deployed on an onboard edge-computing unit (Intel NUC) running the Robot Operating System 2 (ROS 2). The local planner operates at a control frequency of 20 Hz, ensuring rapid response to highly dynamic threats.
	
	\subsection{Qualitative Analysis: Evasive Trajectories}
	To intuitively demonstrate the advantages of the Anisotropic Spatiotemporal Risk Field, we conducted comparative trials between the standard DWB planner and our proposed method in a scenario with high-speed non-cooperative obstacles.
	
	\begin{figure}[htbp]
		\centerline{\includegraphics[width=\linewidth]{fig1.png}}
		\caption{Qualitative comparison of navigation trajectories. The traditional method (red dashed line) exhibits sharp turns and emergency stops (the freezing robot problem) due to spatial distance violations. In contrast, the proposed TCPA-DCPA method (blue solid line) proactive generates a smooth, continuous sliding maneuver around the dynamic obstacle.}
		\label{fig:trajectory}
	\end{figure}
	
	As illustrated in Fig. \ref{fig:trajectory}, the traditional local planner purely relies on spatial inflation. When the dynamic obstacle approaches at high speed, the sudden intersection of their spatial inflation radii triggers an infinite cost, causing the robot to execute severe emergency braking and jerky recovery maneuvers. Conversely, the proposed method evaluates the risk in the temporal domain. Because the TCPA indicates a safe passing window, the risk gradient smoothly guides the robot to perform a continuous sliding evasion, perfectly preserving its kinematic momentum.
	
	\subsection{Quantitative Analysis: Navigation Performance}
	We further quantified the system's performance across 50 dynamic encounter trials. The evaluated metrics include the Success Rate (collision-free arrivals), Average Velocity, and the maximum jerk (to reflect ride smoothness). The comparative results are summarized in Table \ref{tab:performance}.
	
	\begin{table}[htbp]
		\caption{Quantitative Performance Comparison in Highly Dynamic Scenarios}
		\begin{center}
			\begin{tabular}{|c|c|c|c|}
				\hline
				\textbf{Navigation} & \multicolumn{3}{|c|}{\textbf{Evaluation Metrics}} \\
				\cline{2-4} 
				\textbf{Framework} & \textbf{\textit{Success Rate}}& \textbf{\textit{Avg. Velocity}}& \textbf{\textit{Smoothness (Jerk)}} \\
				\hline
				Standard DWB & 72\% & 0.85 m/s & High (Abrupt) \\
				\hline
				\textbf{Proposed Method} & \textbf{96\%} & \textbf{1.32 m/s} & \textbf{Low (Smooth)} \\
				\hline
				\multicolumn{4}{l}{$^{\mathrm{a}}$Data averaged over 50 autonomous trials.}
			\end{tabular}
			\label{tab:performance}
		\end{center}
	\end{table}
	
	The empirical data demonstrates that the proposed algorithm significantly reduces conservative braking, leading to a substantial increase in average velocity while maintaining a superior collision avoidance success rate.
	
	\section{Conclusion}
	In this paper, we presented a predictive local navigation framework designed for mobile robots operating in highly dynamic environments. By addressing the spatiotemporal misalignment inherent in traditional spatial-only local planners, we successfully mitigated the freezing robot problem. The proposed Anisotropic Spatiotemporal Risk Field, driven by a lightweight VoxelGrid and CV-EKF tracking pipeline, actively evaluates the TCPA and DCPA of non-cooperative targets. Real-world validations confirm that our approach enables highly maneuverable robots to execute remarkably smooth and proactive evasive maneuvers without exceeding the computational limits of onboard edge devices. Future work will focus on integrating intent-aware prediction models to further enhance social compliance in dense human-robot environments.
	
	\bibliographystyle{IEEEtran}
	\begin{thebibliography}{00}
		\bibitem{b1} D. Fox, W. Burgard, and S. Thrun, ``The dynamic window approach to collision avoidance,'' IEEE Robotics \& Automation Magazine, vol. 4, no. 1, pp. 23--33, 1997.
		\bibitem{b2} C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann, and T. Bertram, ``Trajectory modification considering dynamic constraints of autonomous robots,'' in ROBOTIK 2012; 7th German Conference on Robotics, 2012, pp. 1--6.
		\bibitem{b3} P. Fiorini and Z. Shiller, ``Motion planning in dynamic environments using velocity obstacles,'' The International Journal of Robotics Research, vol. 17, no. 7, pp. 760--772, 1998.
	\end{thebibliography}
	
\end{document}
