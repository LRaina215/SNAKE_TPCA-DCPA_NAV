# SNAKE_TPCA-DCPA_NAV

**Anisotropic spatiotemporal risk field (TCPA/DCPA) for smooth predictive navigation of mobile robots in dynamic environments — a Nav2/DWB trajectory critic.**

This repository is the companion code for our ACIRS 2026 paper (first author). It implements a lightweight dynamic-obstacle prediction pipeline and a custom DWB trajectory critic that evaluates candidate velocities with a TCPA/DCPA-based risk field, plus hesitation-suppression extensions for decisive motion in dynamic encounters.

## Overview

Conventional DWB local planners score trajectories mainly by instantaneous spatial distance to obstacles. In dynamic scenes this causes the well-known *freezing robot problem* and velocity hesitation: emergency stops in front of oncoming obstacles, forward/backward oscillation, or waiting in place despite passable space.

This work adds a lightweight prediction link to the `Nav2 + DWB` stack for an omnidirectional base:

- **Dynamic obstacle tracking** — constant-velocity Kalman filtering over clustered obstacle point clouds, publishing filtered position/velocity estimates.
- **TCPA/DCPA risk field** — time-to-closest-point-of-approach and distance-at-closest-point-of-approach form an anisotropic spatiotemporal risk cost on each sampled DWB trajectory.
- **Hesitation-suppression extensions** — additional cost terms that penalize in-place waiting, reward lateral escape, prefer passing behind crossing obstacles, and suppress velocity direction flips.

In Gazebo dynamic-obstacle scenarios, the full method improved navigation success rate from **60% (standard DWB) to 100%**, with visibly smoother velocity profiles and fewer hesitation oscillations.

## Method

### 1. Lightweight dynamic obstacle tracking (`predictive_tracker`)

A C++ ROS 2 node (`DynamicTrackerNode`) that turns segmented obstacle point clouds into tracked dynamic obstacles:

1. Transform input cloud to the `odom` frame and VoxelGrid-downsample it.
2. Project to 2D and run Euclidean clustering; compute each cluster's 2D centroid.
3. Associate clusters to tracks with nearest-neighbor + distance gating.
4. Estimate `[x, y, vx, vy]` per track with a **constant-velocity Kalman filter**.
5. Publish a track as a dynamic obstacle only after enough confirmed hits **and** consecutive frames above a speed threshold (suppresses ghost detections); short prediction coasting bridges brief occlusions.

Output: `/tracked_obstacles` (`predictive_navigation_msgs/TrackedObstacleArray`: id, filtered position, filtered velocity) and `/tracked_obstacle_markers` for RViz. Key parameters live in `predictive_tracker/config/dynamic_tracker.yaml`.

### 2. Anisotropic spatiotemporal risk field (`tcpa_dcpa_critic`)

A Nav2 DWB plugin (`tcpa_dcpa_critic::TCPADCPACritic : dwb_core::TrajectoryCritic`, registered via `tcpa_dcpa_critic.xml`) that subscribes to `/tracked_obstacles` and adds a risk score to every sampled trajectory.

For robot state `(p_r, v_r)` and obstacle state `(p_o, v_o)`, with relative position `p_rel = p_o − p_r` and relative velocity `v_rel = v_r − v_o`:

- If `p_rel · v_rel ≤ 0`, the trajectory is not closing on the obstacle → risk 0.
- Otherwise compute time and distance of closest approach:

```
TCPA = (p_rel · v_rel) / ||v_rel||²
DCPA = ||p_rel − TCPA · v_rel||
```

- Base risk cost:

```
Cost_risk = exp(−TCPA / τ_safe) · exp(−DCPA² / (2σ_safe²))
```

Risk is evaluated once per obstacle at the trajectory start, keeping the cost near **O(M)** in the number of dynamic obstacles.

### 3. Hesitation-suppression extensions

Pure TCPA/DCPA risk alone still hesitates or picks poor escape directions in close encounters. The critic therefore adds, under urgent-interaction conditions:

| Term | Purpose |
|---|---|
| `hesitation_penalty` | Penalizes near-zero candidate speeds ("waiting it out") |
| `lateral_escape_penalty` | Penalizes insufficient lateral escape velocity against side-approaching obstacles |
| `goal_progress_penalty` | Penalizes trajectories that dodge but make poor progress toward the goal |
| `escape_alignment_penalty` | Prefers combined forward + lateral escape directions |
| `rear_passing_penalty` | Discourages mirroring a crossing obstacle's lateral motion; prefers passing behind it |
| `swept_corridor_penalty` | Penalizes trajectories inside the obstacle's swept front corridor; prefers its wake region |
| `direction_flip_penalty` | Suppresses candidate velocities opposing the current motion direction |

Net behavior: instead of merely "staying farther from obstacles", the planner prefers **decisive lateral escapes and passing behind crossing obstacles**.

### 4. DWB integration

The local planner is `DWBLocalPlanner`, configured for an omnidirectional base (no in-place rotation), with the critic chain:

```yaml
critics: ["Oscillation", "BaseObstacle", "TCPADCPA", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

`BaseObstacle` handles static geometric safety; `TCPADCPA` handles dynamic spatiotemporal risk and hesitation suppression; `PathDist`/`GoalDist` keep global task progress. The dynamic-risk chain (`/tracked_obstacles`) is intentionally decoupled from the costmap input.

## Key results

Gazebo experiments with dynamic obstacles (crossing, narrow-corridor, and random-crowd scenarios; obstacles start moving on first navigation goal for clean ablations):

- **Goal-reaching rate: 60.0% → 100.0%** vs. standard DWB in the main dynamic-crossing scenario.
- **Collision count: 0.98 → 0.40** per trial on average.
- **Longitudinal velocity sign-flip count: 13.6 → 9.0** (a hesitation metric on `cmd_vel`); smoother `vx` profiles (see `ablation_eval_output/smoothness_plots/`).
- Sharper, more decisive lateral escapes against side-approaching obstacles; emergent "pass behind the crossing obstacle" behavior.
- Critic scoring overhead stays lightweight: per-obstacle single evaluation, O(M) complexity (see `ablation_eval_output/paper_overhead_table.csv`).

Ablation configurations: `Full` (this method) vs `DWB Baseline` (native critics only) vs `DWB RiskOnly` (base risk field without the hesitation extensions) vs `TEB` (cross-planner reference).

## Repository structure

```
SNAKE_TPCA-DCPA_NAV/
├── tcpa_dcpa_critic/          # DWB plugin: TCPA/DCPA risk critic (C++)
├── predictive_tracker/        # Dynamic obstacle tracking node (C++)
│   ├── config/dynamic_tracker.yaml
│   └── launch/dynamic_tracker.launch.py
├── predictive_navigation_msgs/# TrackedObstacle / TrackedObstacleArray messages
├── tcpa_sim_env/              # Gazebo simulation: worlds, obstacle mover, launch
│   ├── worlds/{dynamic_test,narrow_corridor,random_crowd}.world
│   └── scripts/obstacle_mover.py
├── rm_navi/                   # Full navigation stack (rm_navigation, rm_perception,
│                              # rm_localization, smart_escape, ...)
├── costmap_converter/         # Costmap utilities
├── livox_laser_simulation_ros2/
├── rm_communication/          # Upper/lower-computer communication packages
├── rm_description/            # Robot description
├── sim_pre.sh / sim_nav.sh    # Recommended simulation bring-up (see Quick start)
├── sim_nav_dwb_baseline.sh / sim_nav_dwb_risk_only.sh / sim_nav_teb.sh
├── sim_pre_narrow.sh / sim_pre_random.sh / sim_nav_narrow.sh ...  # multi-scenario
├── run_ablation_eval.py       # Automated ablation evaluation → CSV tables
├── prepare_paper_tables.py    # Derives paper-ready tables from raw trial CSVs
├── extract_bag_to_csv.py / plot_smoothness.py / plot_trajectory.py
├── ablation_eval_output/      # Paper tables (CSV) and smoothness/trajectory plots
└── docs/
    └── old-readme.md          # Archived: the original development-notes README
```

## Dependencies

- ROS 2 (the repo's scripts target **Galactic**)
- Nav2 stack (`nav2_bringup`, `dwb_core`, `nav2_costmap_2d`, `pluginlib`)
- PCL (`pcl`, `pcl_conversions`) and Eigen (tracking node)
- Gazebo 11 (Gazebo classic — the latest Gazebo release available on Ubuntu 20.04) with ROS 2 integration for simulation
- Python 3 with `numpy`, `matplotlib` (evaluation/plotting scripts)

## Build

```bash
# clone this repo into your colcon workspace's src/ directory, e.g.:
cd ~/auto_shao/src
git clone https://github.com/LRaina215/SNAKE_TPCA-DCPA_NAV.git
cd ~/auto_shao
colcon build --symlink-install
source install/setup.bash
```

The `sim_*.sh` scripts are run from the repository root (the workspace working directory) — see Quick start.

## Quick start (simulation)

Terminal 1 — perception & tracking stack:

```bash
cd <path-to-this-repo>
./sim_pre.sh
```

This launches the Gazebo world (`tcpa_sim_env`), Point-LIO, ground segmentation, `predictive_tracker`, and pointcloud-to-laserscan.

Terminal 2 — Nav2 with the full method:

```bash
cd <path-to-this-repo>
./sim_nav.sh
```

Then send a navigation goal in RViz. **Note:** dynamic obstacles stay still until the first goal (or first non-zero `cmd_vel`) is received — this is intentional for clean ablations, not a bug.

Key topics to watch: `/odom`, `/cmd_vel`, `/scan_nav`, `/segmentation/obstacle`, `/tracked_obstacles`, `/tracked_obstacle_markers`.

Ablation variants (swap terminal 2's script):

| Script | Configuration |
|---|---|
| `sim_nav.sh` | Full method (TCPA/DCPA + hesitation extensions) |
| `sim_nav_dwb_baseline.sh` | DWB baseline (native critics only) |
| `sim_nav_dwb_risk_only.sh` | DWB + base risk field, extensions off |
| `sim_nav_teb.sh` | TEB reference |

Narrow-corridor / random-crowd scenarios: use `sim_pre_narrow.sh` + `sim_nav_narrow.sh`, or `sim_pre_random.sh` + corresponding nav scripts.

## Reproducing the paper experiments

```bash
source /opt/ros/galactic/setup.bash
source <workspace>/install/setup.bash

# quick smoke test: 1 trial per group
python3 run_ablation_eval.py --trials-per-group 1

# full ablation (default 50 trials/group; add TEB with --include-teb)
python3 run_ablation_eval.py --trials-per-group 50

# multi-scenario robustness only
python3 run_ablation_eval.py --skip-ablation --run-multi-scenario --multi-scenario-trials 20

# derive paper-ready tables from raw trial CSVs
python3 prepare_paper_tables.py
```

Outputs land in `ablation_eval_output/`: per-trial CSVs, `paper_dynamic_test_table.csv`, `paper_overhead_table.csv`, `paper_multi_scenario_table.csv` (+ `_wide` variant), and smoothness/trajectory plots. The committed CSVs/plots in `ablation_eval_output/` are the exact artifacts used for the paper tables.

## Citation

```bibtex
@inproceedings{luan2026anisotropic,
  author    = {Junhui Luan and Yuqi Liang and Zixuan Lin and Zhong Huang},
  title     = {Anisotropic Spatiotemporal Risk Field for Smooth Predictive Navigation of Mobile Robots in Dynamic Environments},
  booktitle = {Proc.\ 11th Asia-Pacific Conference on Intelligent Robot Systems (ACIRS)},
  year      = {2026},
  note      = {to appear}
}
```

> The proceedings are not yet indexed on IEEE Xplore — the citation above is a placeholder and will be updated with page numbers / DOI once available.

## License

This project is licensed under the Apache License 2.0 — see the [LICENSE](LICENSE) file for details.

## Contact

- Junhui Luan — 20243007059@hainanu.edu.cn
- GitHub: [LRaina215](https://github.com/LRaina215)

Issues and pull requests are welcome.
