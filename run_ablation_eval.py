#!/usr/bin/env python3

import argparse
import math
import os
import signal
import subprocess
import sys
import tempfile
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import pandas as pd
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Empty

try:
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
except ImportError:  # pragma: no cover - depends on local ROS install
    EntityState = None
    DeleteEntity = None
    SetEntityState = None
    SpawnEntity = None

try:
    from gazebo_msgs.msg import ModelStates
except ImportError:  # pragma: no cover - depends on local ROS install
    ModelStates = None

try:
    from predictive_navigation_msgs.msg import TrackedObstacleArray
except ImportError:  # pragma: no cover - depends on local ROS build
    TrackedObstacleArray = None


ROBOT_MODEL_NAME = "tcpa_robot"
DYNAMIC_OBSTACLE_NAMES = ("obs1", "obs2")
TRACKED_ENTITY_NAMES = (ROBOT_MODEL_NAME, *DYNAMIC_OBSTACLE_NAMES)
TRIAL_CLEANUP_PATTERNS = (
    "ros2 launch tcpa_sim_env sim_launch.py",
    "ros2 launch navi bringup_dwb_baseline_launch.py",
    "ros2 launch navi bringup_dwb_risk_only_launch.py",
    "ros2 launch nav2_bringup navigation_launch.py",
    "ros2 launch point_lio mapping_mid360.launch.py",
    "ros2 launch predictive_tracker dynamic_tracker.launch.py",
    "ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py",
    "ros2 launch linefit_ground_segmentation_ros segmentation.launch.py",
    "ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map odom",
    "obstacle_mover.py",
    "gzserver",
)


@dataclass(frozen=True)
class ExperimentGroup:
    name: str
    script: str


@dataclass
class TrialResult:
    group: str
    trial_index: int
    success: int
    outcome: str
    navigation_time_s: float
    average_speed_mps: float
    min_clearance_m: float
    velocity_sign_flip_count: int
    path_length_m: float


@dataclass(frozen=True)
class EntitySnapshot:
    name: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    vx: float
    vy: float
    vz: float
    wx: float
    wy: float
    wz: float

    @classmethod
    def from_pose_twist(cls, name: str, pose, twist) -> "EntitySnapshot":
        return cls(
            name=name,
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
            qx=pose.orientation.x,
            qy=pose.orientation.y,
            qz=pose.orientation.z,
            qw=pose.orientation.w,
            vx=twist.linear.x,
            vy=twist.linear.y,
            vz=twist.linear.z,
            wx=twist.angular.x,
            wy=twist.angular.y,
            wz=twist.angular.z,
        )

    def to_entity_state(self) -> EntityState:
        state = EntityState()
        state.name = self.name
        state.reference_frame = "world"
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = self.z
        state.pose.orientation.x = self.qx
        state.pose.orientation.y = self.qy
        state.pose.orientation.z = self.qz
        state.pose.orientation.w = self.qw
        state.twist.linear.x = self.vx
        state.twist.linear.y = self.vy
        state.twist.linear.z = self.vz
        state.twist.angular.x = self.wx
        state.twist.angular.y = self.wy
        state.twist.angular.z = self.wz
        return state

    def position_xy(self) -> Tuple[float, float]:
        return (self.x, self.y)


class AblationEvaluator(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ablation_evaluator")
        self.args = args
        self._subscriptions = []

        self.goal_client = ActionClient(self, NavigateToPose, args.goal_action)
        goal_topics = []
        for topic_name in (
            args.goal_pose_topic,
            "/goal_pose",
            "/move_base_simple/goal",
        ):
            if topic_name and topic_name not in goal_topics:
                goal_topics.append(topic_name)
        self.goal_pose_publishers = [
            self.create_publisher(PoseStamped, topic_name, 10)
            for topic_name in goal_topics
        ]
        self.reset_clients = [
            self.create_client(Empty, service_name)
            for service_name in (
                args.reset_service,
                "/reset_world",
                "/gazebo/reset_world",
                "/reset_simulation",
                "/gazebo/reset_simulation",
            )
            if service_name
        ]
        self.motion_reset_clients = [
            self.create_client(Empty, service_name)
            for service_name in (
                args.obstacle_reset_service,
                "/reset_motion",
                "reset_motion",
                "/obstacle_mover/reset_motion",
                "obstacle_mover/reset_motion",
            )
            if service_name
        ]
        self.set_entity_state_clients = []
        if SetEntityState is not None:
            self.set_entity_state_clients = [
                self.create_client(SetEntityState, service_name)
                for service_name in (
                    args.set_entity_state_service,
                    "/set_entity_state",
                    "/gazebo/set_entity_state",
                )
                if service_name
            ]
        self.spawn_entity_clients = []
        if SpawnEntity is not None:
            self.spawn_entity_clients = [
                self.create_client(SpawnEntity, service_name)
                for service_name in (
                    args.spawn_entity_service,
                    "/spawn_entity",
                )
                if service_name
            ]
        self.delete_entity_clients = []
        if DeleteEntity is not None:
            self.delete_entity_clients = [
                self.create_client(DeleteEntity, service_name)
                for service_name in (
                    args.delete_entity_service,
                    "/delete_entity",
                )
                if service_name
            ]

        self._subscriptions.append(self.create_subscription(Odometry, args.odom_topic, self._odom_cb, 50))
        self._subscriptions.append(self.create_subscription(Twist, args.cmd_vel_topic, self._cmd_vel_cb, 50))
        if ModelStates is not None:
            model_state_topics = []
            for topic in (args.model_states_topic, "/gazebo/model_states", "/model_states"):
                if topic and topic not in model_state_topics:
                    model_state_topics.append(topic)
            for topic in model_state_topics:
                self._subscriptions.append(self.create_subscription(ModelStates, topic, self._model_states_cb, 20))
        if TrackedObstacleArray is not None:
            self._subscriptions.append(
                self.create_subscription(
                    TrackedObstacleArray, args.tracked_obstacles_topic, self._tracked_obstacles_cb, 20
                )
            )

        self._have_odom = False
        self._have_model_states = False
        self._robot_odom_xy: Optional[Tuple[float, float]] = None
        self._robot_truth_xy: Optional[Tuple[float, float]] = None
        self._truth_obstacles: Dict[str, Tuple[float, float]] = {}
        self._tracked_obstacles_xy: List[Tuple[float, float]] = []
        self._latest_entity_snapshots: Dict[str, EntitySnapshot] = {}
        self._initial_entity_snapshots: Dict[str, EntitySnapshot] = {}

        self._trial_active = False
        self._trial_start_wall = 0.0
        self._last_odom_xy: Optional[Tuple[float, float]] = None
        self._path_length = 0.0
        self._min_clearance = math.inf
        self._collision_detected = False
        self._collision_observed = False
        self._velocity_sign_flips = 0
        self._last_nonzero_cmd_sign = 0
        self._last_nonzero_cmd_wall = 0.0
        self._obstacle_motion_start_wall: Optional[float] = None
        self._robot_xml: Optional[str] = None

    def wait_for_pre_stack_ready(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._have_odom:
                return
        raise RuntimeError("Timed out waiting for the pre-simulation stack")

    def wait_for_nav_stack_ready(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._have_odom and self.goal_client.wait_for_server(timeout_sec=0.0):
                return
        raise RuntimeError("Timed out waiting for /odom and /navigate_to_pose")

    def wait_for_robot_near_pose(self, target_xy: Tuple[float, float], tolerance_m: float, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            robot_xy = self._robot_truth_xy or self._robot_odom_xy
            if robot_xy is None:
                continue
            if math.hypot(robot_xy[0] - target_xy[0], robot_xy[1] - target_xy[1]) <= tolerance_m:
                return
        robot_xy = self._robot_truth_xy or self._robot_odom_xy
        raise RuntimeError(
            f"Timed out waiting for robot near initial pose {target_xy}; latest pose={robot_xy}"
        )

    def clear_stack_runtime_state(self) -> None:
        self._have_odom = False
        self._have_model_states = False
        self._robot_odom_xy = None
        self._robot_truth_xy = None
        self._truth_obstacles = {}
        self._tracked_obstacles_xy = []
        self._latest_entity_snapshots = {}
        self._initial_entity_snapshots = {}
        self._obstacle_motion_start_wall = None

    def wait_for_entity_states_ready(self, timeout_s: float, names: Sequence[str] = TRACKED_ENTITY_NAMES) -> None:
        deadline = time.monotonic() + timeout_s
        required_names = tuple(names)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self._have_model_states:
                continue
            if all(name in self._latest_entity_snapshots for name in required_names):
                return
        missing = [name for name in required_names if name not in self._latest_entity_snapshots]
        raise RuntimeError(f"Timed out waiting for Gazebo model states for entities: {missing}")

    def refresh_initial_entity_snapshots(self, timeout_s: float) -> None:
        try:
            self.wait_for_entity_states_ready(timeout_s)
            self._initial_entity_snapshots = {
                name: self._latest_entity_snapshots[name]
                for name in TRACKED_ENTITY_NAMES
            }
            summary = ", ".join(
                f"{name}=({snapshot.x:.2f}, {snapshot.y:.2f}, {snapshot.z:.2f})"
                for name, snapshot in self._initial_entity_snapshots.items()
            )
            self.get_logger().info(f"Captured initial Gazebo snapshots: {summary}")
        except RuntimeError as exc:
            self._initial_entity_snapshots = self._build_fallback_initial_entity_snapshots()
            self.get_logger().warn(
                f"{exc}. Falling back to configured initial poses for entity reset."
            )

    def call_empty_service(self, clients: Sequence, label: str, timeout_s: float = 10.0) -> bool:
        for client in clients:
            if not client.wait_for_service(timeout_sec=0.5):
                continue
            future = client.call_async(Empty.Request())
            if self._spin_until_future(future, timeout_s):
                self.get_logger().info(f"{label}: called {client.srv_name}")
                return True
        self.get_logger().warn(f"{label}: no available service among {[c.srv_name for c in clients]}")
        return False

    @staticmethod
    def _has_available_service(clients: Sequence, timeout_sec: float = 0.2) -> bool:
        for client in clients:
            if client.wait_for_service(timeout_sec=timeout_sec):
                return True
        return False

    def reset_trial_environment(self) -> bool:
        if not self._initial_entity_snapshots:
            self.refresh_initial_entity_snapshots(self.args.initial_snapshot_timeout_s)

        self.call_empty_service(self.motion_reset_clients, "obstacle motion reset (pre)")
        if not self.call_empty_service(self.reset_clients, "simulation reset"):
            return False

        have_verified_model_states = True
        try:
            self.wait_for_entity_states_ready(self.args.entity_wait_timeout_s)
        except RuntimeError as exc:
            self.get_logger().warn(str(exc))
            have_verified_model_states = False

        for attempt in range(1, self.args.reset_retry_count + 1):
            if self.reset_entities_to_initial_state():
                self.call_empty_service(self.motion_reset_clients, "obstacle motion reset (post)")
                self._wait_for_pose_update(self.args.settle_time_s)
                if not have_verified_model_states or self.verify_entities_at_initial_state():
                    return True

            if attempt < self.args.reset_retry_count:
                self.get_logger().warn(
                    f"Entity restore verification failed on attempt "
                    f"{attempt}/{self.args.reset_retry_count}; retrying."
                )
                self._wait_for_pose_update(self.args.reset_retry_delay_s)

        self.get_logger().error("Failed to restore Gazebo entities to their initial state.")
        return False

    def reset_entities_to_initial_state(self) -> bool:
        if not self.set_entity_state_clients or not self._has_available_service(self.set_entity_state_clients):
            return self._respawn_robot_to_initial_state()

        success = True
        for name in TRACKED_ENTITY_NAMES:
            snapshot = self._initial_entity_snapshots.get(name)
            if snapshot is None:
                self.get_logger().warn(f"Missing initial snapshot for entity {name}")
                success = False
                continue
            if not self.call_set_entity_state(snapshot.to_entity_state()):
                success = False
        return success

    def call_set_entity_state(self, state: EntityState, timeout_s: float = 5.0) -> bool:
        if not self.set_entity_state_clients:
            self.get_logger().warn("set_entity_state: no available client configured")
            return False

        for client in self.set_entity_state_clients:
            if not client.wait_for_service(timeout_sec=0.5):
                continue

            request = SetEntityState.Request()
            request.state = state
            future = client.call_async(request)
            if not self._spin_until_future(future, timeout_s):
                continue

            response = future.result()
            if response is not None and response.success:
                self.get_logger().debug(f"set_entity_state succeeded via {client.srv_name} for {state.name}")
                return True
            if response is not None:
                status_message = getattr(response, "status_message", "")
                self.get_logger().warn(
                    f"set_entity_state via {client.srv_name} failed for {state.name}: {status_message}"
                )

        self.get_logger().warn(f"Failed to set initial state for entity {state.name}")
        return False

    def _respawn_robot_to_initial_state(self) -> bool:
        robot_snapshot = self._initial_entity_snapshots.get(ROBOT_MODEL_NAME)
        if robot_snapshot is None:
            self.get_logger().warn("Missing initial snapshot for tcpa_robot")
            return False

        if not self._delete_robot_entity():
            return False

        self._have_odom = False
        self._robot_odom_xy = None
        self._robot_truth_xy = None

        if not self._spawn_robot_entity(robot_snapshot):
            return False

        try:
            self.wait_for_pre_stack_ready(self.args.robot_respawn_timeout_s)
        except RuntimeError as exc:
            self.get_logger().warn(f"Timed out waiting for /odom after robot respawn: {exc}")
            return False

        return True

    def _delete_robot_entity(self) -> bool:
        if not self.delete_entity_clients:
            self.get_logger().warn("delete_entity: no available client configured")
            return False

        for client in self.delete_entity_clients:
            if not client.wait_for_service(timeout_sec=0.5):
                continue

            request = DeleteEntity.Request()
            request.name = ROBOT_MODEL_NAME
            future = client.call_async(request)
            if not self._spin_until_future(future, self.args.robot_delete_timeout_s):
                self.get_logger().warn(
                    f"delete_entity via {client.srv_name} timed out for {ROBOT_MODEL_NAME}"
                )
                continue

            response = future.result()
            if response is None:
                continue

            if response.success:
                self.get_logger().info(f"Deleted entity {ROBOT_MODEL_NAME} via {client.srv_name}")
                self._wait_for_pose_update(self.args.robot_delete_settle_s)
                return True

            status_message = getattr(response, "status_message", "")
            if "does not exist" in status_message.lower():
                self.get_logger().info(f"Entity {ROBOT_MODEL_NAME} was already absent before respawn.")
                self._wait_for_pose_update(self.args.robot_delete_settle_s)
                return True

            self.get_logger().warn(
                f"delete_entity via {client.srv_name} failed for {ROBOT_MODEL_NAME}: {status_message}"
            )

        return False

    def _spawn_robot_entity(self, snapshot: EntitySnapshot) -> bool:
        if not self.spawn_entity_clients:
            self.get_logger().warn("spawn_entity: no available client configured")
            return False

        robot_xml = self._load_robot_xml()
        if robot_xml is None:
            return False

        for client in self.spawn_entity_clients:
            if not client.wait_for_service(timeout_sec=0.5):
                continue

            request = SpawnEntity.Request()
            request.name = ROBOT_MODEL_NAME
            request.xml = robot_xml
            request.robot_namespace = ""
            request.reference_frame = "world"
            request.initial_pose.position.x = snapshot.x
            request.initial_pose.position.y = snapshot.y
            request.initial_pose.position.z = snapshot.z
            request.initial_pose.orientation.x = snapshot.qx
            request.initial_pose.orientation.y = snapshot.qy
            request.initial_pose.orientation.z = snapshot.qz
            request.initial_pose.orientation.w = snapshot.qw
            future = client.call_async(request)
            if not self._spin_until_future(future, self.args.robot_spawn_timeout_s):
                self.get_logger().warn(
                    f"spawn_entity via {client.srv_name} timed out for {ROBOT_MODEL_NAME}"
                )
                continue

            response = future.result()
            if response is not None and response.success:
                self.get_logger().info(
                    f"Respawned {ROBOT_MODEL_NAME} at ({snapshot.x:.2f}, {snapshot.y:.2f}) via {client.srv_name}"
                )
                return True

            if response is not None:
                status_message = getattr(response, "status_message", "")
                self.get_logger().warn(
                    f"spawn_entity via {client.srv_name} failed for {ROBOT_MODEL_NAME}: {status_message}"
                )

        return False

    def _load_robot_xml(self) -> Optional[str]:
        if self._robot_xml is not None:
            return self._robot_xml

        xacro_path = self.args.robot_xacro_path.expanduser().resolve()
        if not xacro_path.exists():
            self.get_logger().warn(f"Robot xacro does not exist: {xacro_path}")
            return None

        completed = subprocess.run(
            [
                "xacro",
                str(xacro_path),
                f"lidar_backend:={self.args.robot_lidar_backend}",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            self.get_logger().warn(
                f"Failed to render robot xacro {xacro_path}: {completed.stderr.strip()}"
            )
            return None

        self._robot_xml = completed.stdout
        return self._robot_xml

    def verify_entities_at_initial_state(self) -> bool:
        if not self._have_model_states:
            self.get_logger().warn("Skipping restore verification because /gazebo/model_states is unavailable.")
            return True

        self._wait_for_pose_update(self.args.reset_verify_timeout_s)
        all_ok = True
        for name, snapshot in self._initial_entity_snapshots.items():
            current = self._latest_entity_snapshots.get(name)
            if current is None:
                self.get_logger().warn(f"Restore verification missing current Gazebo state for {name}")
                all_ok = False
                continue

            position_error = math.hypot(current.x - snapshot.x, current.y - snapshot.y)
            z_error = abs(current.z - snapshot.z)
            if position_error > self.args.pose_reset_tolerance_m or z_error > self.args.z_reset_tolerance_m:
                self.get_logger().warn(
                    f"Restore verification failed for {name}: "
                    f"expected=({snapshot.x:.2f}, {snapshot.y:.2f}, {snapshot.z:.2f}) "
                    f"actual=({current.x:.2f}, {current.y:.2f}, {current.z:.2f}) "
                    f"xy_error={position_error:.3f} z_error={z_error:.3f}"
                )
                all_ok = False

        return all_ok

    @staticmethod
    def _make_failed_result(trial_index: int, outcome: str) -> TrialResult:
        return TrialResult(
            group="",
            trial_index=trial_index,
            success=0,
            outcome=outcome,
            navigation_time_s=float("nan"),
            average_speed_mps=float("nan"),
            min_clearance_m=float("nan"),
            velocity_sign_flip_count=0,
            path_length_m=0.0,
        )

    def _build_fallback_initial_entity_snapshots(self) -> Dict[str, EntitySnapshot]:
        return {
            ROBOT_MODEL_NAME: EntitySnapshot(
                name=ROBOT_MODEL_NAME,
                x=self.args.robot_initial_x,
                y=self.args.robot_initial_y,
                z=self.args.robot_initial_z,
                qx=0.0,
                qy=0.0,
                qz=0.0,
                qw=1.0,
                vx=0.0,
                vy=0.0,
                vz=0.0,
                wx=0.0,
                wy=0.0,
                wz=0.0,
            ),
            "obs1": EntitySnapshot(
                name="obs1",
                x=self.args.obs1_initial_x,
                y=self.args.obs1_initial_y,
                z=self.args.obs1_initial_z,
                qx=0.0,
                qy=0.0,
                qz=0.0,
                qw=1.0,
                vx=0.0,
                vy=0.0,
                vz=0.0,
                wx=0.0,
                wy=0.0,
                wz=0.0,
            ),
            "obs2": EntitySnapshot(
                name="obs2",
                x=self.args.obs2_initial_x,
                y=self.args.obs2_initial_y,
                z=self.args.obs2_initial_z,
                qx=0.0,
                qy=0.0,
                qz=0.0,
                qw=1.0,
                vx=0.0,
                vy=0.0,
                vz=0.0,
                wx=0.0,
                wy=0.0,
                wz=0.0,
            ),
        }

    def run_trial(self, trial_index: int, perform_reset: bool = True) -> TrialResult:
        if perform_reset:
            if not self.reset_trial_environment():
                return self._make_failed_result(trial_index, "reset_failed")
        else:
            self.call_empty_service(self.motion_reset_clients, "obstacle motion reset (startup)")
            self._wait_for_pose_update(self.args.startup_settle_time_s)

        self.call_empty_service(self.motion_reset_clients, "obstacle motion reset (arm)")
        self._start_metrics()

        goal_handle = None
        last_outcome = "goal_rejected"
        for attempt in range(1, self.args.goal_retry_count + 1):
            self._publish_manual_goal_topics()
            goal_future = self.goal_client.send_goal_async(self._make_goal())
            if not self._spin_until_future(goal_future, self.args.server_timeout_s):
                last_outcome = "goal_send_timeout"
                continue

            goal_handle = goal_future.result()
            if goal_handle is not None and goal_handle.accepted:
                break

            last_outcome = "goal_rejected"
            if attempt < self.args.goal_retry_count:
                self.get_logger().warn(
                    f"Goal rejected on attempt {attempt}/{self.args.goal_retry_count}; retrying after startup delay."
                )
                self._wait_for_pose_update(self.args.goal_retry_delay_s)

        if goal_handle is None or not goal_handle.accepted:
            self._stop_metrics()
            return self._build_result(trial_index, success=0, outcome=last_outcome)

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + self.args.trial_timeout_s
        outcome = "unknown"
        success = 0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self._collision_detected:
                outcome = "collision"
                goal_handle.cancel_goal_async()
                break

            if time.monotonic() >= deadline:
                outcome = "timeout"
                goal_handle.cancel_goal_async()
                break

            if result_future.done():
                result = result_future.result()
                if result is None:
                    outcome = "result_missing"
                    break
                outcome = self._status_to_text(result.status)
                success = int(result.status == GoalStatus.STATUS_SUCCEEDED and not self._collision_detected)
                if success:
                    robot_xy = self._current_robot_xy()
                    goal_distance = float("inf")
                    if robot_xy is not None:
                        goal_distance = math.hypot(robot_xy[0] - self.args.goal_x, robot_xy[1] - self.args.goal_y)

                    if goal_distance > self.args.final_goal_tolerance_m:
                        success = 0
                        outcome = "false_success_goal_miss"
                    elif self._path_length < self.args.min_success_path_length_m:
                        success = 0
                        outcome = "false_success_short_path"
                break

        self._stop_metrics()
        if success == 0 and outcome == "unknown":
            outcome = "aborted"
        return self._build_result(trial_index, success=success, outcome=outcome)

    def _make_goal(self) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose = self._make_goal_pose_stamped()
        return goal

    def _make_goal_pose_stamped(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.args.goal_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.args.goal_x
        pose.pose.position.y = self.args.goal_y
        pose.pose.orientation.w = 1.0
        return pose

    def _publish_manual_goal_topics(self) -> None:
        if not self.args.publish_manual_goal_topics:
            return

        pose = self._make_goal_pose_stamped()
        published_topics = []
        for publisher in self.goal_pose_publishers:
            publisher.publish(pose)
            published_topics.append(publisher.topic_name)

        if published_topics:
            self.get_logger().info(
                f"Published goal pose topics before action dispatch: {published_topics}"
            )
            self._wait_for_pose_update(self.args.goal_topic_lead_s)

    def _start_metrics(self) -> None:
        self._trial_active = True
        self._trial_start_wall = time.monotonic()
        self._last_odom_xy = self._robot_odom_xy
        self._path_length = 0.0
        self._min_clearance = math.inf
        self._collision_detected = False
        self._collision_observed = False
        self._velocity_sign_flips = 0
        self._last_nonzero_cmd_sign = 0
        self._last_nonzero_cmd_wall = 0.0
        self._obstacle_motion_start_wall = None
        self._update_clearance()

    def _stop_metrics(self) -> None:
        self._trial_active = False

    def _build_result(self, trial_index: int, success: int, outcome: str) -> TrialResult:
        elapsed = max(0.0, time.monotonic() - self._trial_start_wall)
        avg_speed = self._path_length / elapsed if elapsed > 1e-9 else float("nan")
        nav_time = elapsed if success else float("nan")
        if math.isinf(self._min_clearance):
            min_clearance = float("nan")
        else:
            min_clearance = self._min_clearance
        return TrialResult(
            group="",
            trial_index=trial_index,
            success=success,
            outcome=outcome,
            navigation_time_s=nav_time,
            average_speed_mps=avg_speed if success else float("nan"),
            min_clearance_m=min_clearance,
            velocity_sign_flip_count=self._velocity_sign_flips,
            path_length_m=self._path_length,
        )

    def _current_robot_xy(self) -> Optional[Tuple[float, float]]:
        return self._robot_truth_xy or self._robot_odom_xy

    def _odom_cb(self, msg: Odometry) -> None:
        xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._robot_odom_xy = xy
        self._have_odom = True

        if not self._trial_active:
            return

        if self._last_odom_xy is not None:
            self._path_length += math.hypot(xy[0] - self._last_odom_xy[0], xy[1] - self._last_odom_xy[1])
        self._last_odom_xy = xy
        self._update_clearance()

    def _cmd_vel_cb(self, msg: Twist) -> None:
        if not self._trial_active:
            return

        vx = msg.linear.x
        if abs(vx) < self.args.cmd_sign_threshold:
            return

        sign = 1 if vx > 0.0 else -1
        now = time.monotonic()
        if self._obstacle_motion_start_wall is None:
            self._obstacle_motion_start_wall = now
        if self._last_nonzero_cmd_sign != 0 and sign != self._last_nonzero_cmd_sign:
            if now - self._last_nonzero_cmd_wall <= self.args.flip_window_s:
                self._velocity_sign_flips += 1
        self._last_nonzero_cmd_sign = sign
        self._last_nonzero_cmd_wall = now

    def _model_states_cb(self, msg) -> None:
        self._have_model_states = True
        entity_snapshots = {
            name: EntitySnapshot.from_pose_twist(name, pose, twist)
            for name, pose, twist in zip(msg.name, msg.pose, msg.twist)
        }
        self._latest_entity_snapshots = entity_snapshots

        robot_snapshot = entity_snapshots.get(ROBOT_MODEL_NAME)
        if robot_snapshot is not None:
            self._robot_truth_xy = robot_snapshot.position_xy()

        truth_obstacles = {
            obstacle_name: entity_snapshots[obstacle_name].position_xy()
            for obstacle_name in DYNAMIC_OBSTACLE_NAMES
            if obstacle_name in entity_snapshots
        }
        if truth_obstacles:
            self._truth_obstacles = truth_obstacles

        if self._trial_active:
            self._update_clearance()

    def _tracked_obstacles_cb(self, msg) -> None:
        self._tracked_obstacles_xy = [
            (obstacle.position.x, obstacle.position.y) for obstacle in msg.obstacles
        ]
        if self._trial_active:
            self._update_clearance()

    def _update_clearance(self) -> None:
        robot_xy = self._robot_truth_xy or self._robot_odom_xy
        if robot_xy is None:
            return

        obstacle_positions = list(self._truth_obstacles.values())
        if not obstacle_positions:
            obstacle_positions = self._tracked_obstacles_xy
        if not obstacle_positions and self.args.use_analytic_obstacle_clearance:
            obstacle_positions = self._get_analytic_obstacle_positions(time.monotonic())
        if not obstacle_positions:
            return

        for obstacle_xy in obstacle_positions:
            center_distance = math.hypot(robot_xy[0] - obstacle_xy[0], robot_xy[1] - obstacle_xy[1])
            clearance = center_distance - self.args.robot_radius - self.args.obstacle_radius
            self._min_clearance = min(self._min_clearance, clearance)
            if clearance <= self.args.collision_clearance_threshold:
                self._collision_observed = True
                if self.args.abort_on_estimated_collision:
                    self._collision_detected = True

    def _get_analytic_obstacle_positions(self, now_wall: float) -> List[Tuple[float, float]]:
        obs1_snapshot = self._initial_entity_snapshots.get("obs1")
        obs2_snapshot = self._initial_entity_snapshots.get("obs2")
        if obs1_snapshot is None or obs2_snapshot is None:
            return []

        if self._obstacle_motion_start_wall is None:
            return [obs1_snapshot.position_xy(), obs2_snapshot.position_xy()]

        elapsed = max(0.0, now_wall - self._obstacle_motion_start_wall)
        phase = elapsed % 8.0

        obs1_y = obs1_snapshot.y + 1.2 * phase if phase < 4.0 else obs1_snapshot.y + 4.8 - 1.2 * (phase - 4.0)
        obs2_x = obs2_snapshot.x - 1.0 * phase if phase < 4.0 else obs2_snapshot.x - 4.0 + 1.0 * (phase - 4.0)

        return [
            (obs1_snapshot.x, obs1_y),
            (obs2_x, obs2_snapshot.y),
        ]

    def _wait_for_pose_update(self, settle_time_s: float) -> None:
        deadline = time.monotonic() + settle_time_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _spin_until_future(self, future, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                return True
        return future.done()

    def close(self) -> None:
        if getattr(self, "goal_client", None) is not None:
            try:
                self.goal_client.destroy()
            except Exception:
                pass
            self.goal_client = None

    @staticmethod
    def _status_to_text(status: int) -> str:
        mapping = {
            GoalStatus.STATUS_UNKNOWN: "unknown",
            GoalStatus.STATUS_ACCEPTED: "accepted",
            GoalStatus.STATUS_EXECUTING: "executing",
            GoalStatus.STATUS_CANCELING: "canceling",
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted",
        }
        return mapping.get(status, f"status_{status}")


def launch_stack(script_path: Path, log_dir: Path) -> List[int]:
    pid_file = Path(tempfile.mkstemp(prefix="ablation_pids_", suffix=".txt")[1])
    env = os.environ.copy()
    env["AUTO_EVAL_HEADLESS"] = "1"
    env["AUTO_EVAL_SKIP_RVIZ"] = "1"
    env["AUTO_EVAL_PID_FILE"] = str(pid_file)
    env["AUTO_EVAL_LOG_DIR"] = str(log_dir)

    completed = subprocess.run(
        ["/bin/bash", str(script_path)],
        cwd=str(script_path.parent),
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"Failed to launch {script_path.name}\nSTDOUT:\n{completed.stdout}\nSTDERR:\n{completed.stderr}"
        )

    if not pid_file.exists():
        return []

    pids = [
        int(line.strip())
        for line in pid_file.read_text().splitlines()
        if line.strip()
    ]
    pid_file.unlink(missing_ok=True)
    return pids


def stop_stack(pids: Sequence[int], grace_period_s: float = 5.0) -> None:
    for pid in pids:
        try:
            os.killpg(pid, signal.SIGTERM)
        except ProcessLookupError:
            continue

    deadline = time.monotonic() + grace_period_s
    while time.monotonic() < deadline:
        alive = []
        for pid in pids:
            try:
                os.kill(pid, 0)
                alive.append(pid)
            except ProcessLookupError:
                continue
        if not alive:
            return
        time.sleep(0.2)

    for pid in pids:
        try:
            os.killpg(pid, signal.SIGKILL)
        except ProcessLookupError:
            continue


def force_cleanup_processes(patterns: Sequence[str]) -> None:
    for pattern in patterns:
        subprocess.run(["pkill", "-f", pattern], check=False, capture_output=True, text=True)


def cleanup_active_stacks(pre_pids: Sequence[int], nav_pids: Sequence[int], settle_s: float) -> None:
    if nav_pids:
        stop_stack(nav_pids)
    if pre_pids:
        stop_stack(pre_pids)
    force_cleanup_processes(TRIAL_CLEANUP_PATTERNS)
    time.sleep(settle_s)


def build_trial_log_dirs(output_dir: Path, group_name: str, trial_index: int) -> Dict[str, Path]:
    trial_root = output_dir / "logs" / group_name.lower() / f"trial_{trial_index:03d}"
    return {
        "trial_root": trial_root,
        "sim_pre": trial_root / "sim_pre",
        "nav": trial_root / "nav",
    }


def start_pre_stack(node: AblationEvaluator, args: argparse.Namespace, pre_script: Path, log_dir: Path) -> List[int]:
    node.clear_stack_runtime_state()
    pre_pids = launch_stack(pre_script, log_dir)
    node.wait_for_pre_stack_ready(args.stack_ready_timeout_s)
    node.wait_for_robot_near_pose(
        (args.robot_initial_x, args.robot_initial_y),
        args.initial_pose_tolerance_m,
        args.initial_pose_timeout_s,
    )
    node._wait_for_pose_update(args.pre_stack_warmup_s)
    node.refresh_initial_entity_snapshots(args.initial_snapshot_timeout_s)
    return pre_pids


def start_group_stack(
    node: AblationEvaluator,
    args: argparse.Namespace,
    workspace_src: Path,
    group: ExperimentGroup,
    log_dir: Path,
) -> List[int]:
    nav_pids = launch_stack(workspace_src / group.script, log_dir)
    node.wait_for_nav_stack_ready(args.stack_ready_timeout_s)
    node._wait_for_pose_update(args.group_warmup_s)
    return nav_pids


def summarize_results(results_df: pd.DataFrame) -> pd.DataFrame:
    rows = []
    grouped = results_df.groupby("group", sort=False)
    for group_name, group_df in grouped:
        success_pct = group_df["success"] * 100.0
        row = {
            "group": group_name,
            "success_rate_pct_mean": success_pct.mean(),
            "success_rate_pct_var": success_pct.var(ddof=1),
            "mean_navigation_time_s_mean": group_df["navigation_time_s"].mean(),
            "mean_navigation_time_s_var": group_df["navigation_time_s"].var(ddof=1),
            "average_translational_speed_mps_mean": group_df["average_speed_mps"].mean(),
            "average_translational_speed_mps_var": group_df["average_speed_mps"].var(ddof=1),
            "minimum_clearance_m_mean": group_df["min_clearance_m"].mean(),
            "minimum_clearance_m_var": group_df["min_clearance_m"].var(ddof=1),
            "velocity_sign_flip_count_mean": group_df["velocity_sign_flip_count"].mean(),
            "velocity_sign_flip_count_var": group_df["velocity_sign_flip_count"].var(ddof=1),
        }
        rows.append(row)
    return pd.DataFrame(rows)


def parse_args() -> argparse.Namespace:
    default_root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Run Gazebo navigation ablation experiments automatically.")
    parser.add_argument("--workspace-src", type=Path, default=default_root, help="Path to the workspace src directory.")
    parser.add_argument("--trials-per-group", type=int, default=50, help="Number of trials per group.")
    parser.add_argument("--goal-x", type=float, default=4.0, help="Goal x position in map frame.")
    parser.add_argument("--goal-y", type=float, default=0.0, help="Goal y position in map frame.")
    parser.add_argument("--goal-frame", default="map", help="Goal frame id.")
    parser.add_argument(
        "--goal-pose-topic",
        default="/goal_pose",
        help="Pose topic published before the NavigateToPose action to mimic manual goal setting.",
    )
    parser.add_argument(
        "--goal-topic-lead-s",
        type=float,
        default=0.2,
        help="Delay between publishing the manual goal topic and sending the NavigateToPose action.",
    )
    parser.add_argument("--trial-timeout-s", type=float, default=45.0, help="Maximum trial duration.")
    parser.add_argument("--server-timeout-s", type=float, default=30.0, help="Timeout while waiting for servers.")
    parser.add_argument("--stack-ready-timeout-s", type=float, default=60.0, help="Timeout after starting a stack.")
    parser.add_argument(
        "--initial-pose-timeout-s",
        type=float,
        default=30.0,
        help="Timeout while waiting for the robot to appear near the configured initial pose.",
    )
    parser.add_argument(
        "--initial-pose-tolerance-m",
        type=float,
        default=0.60,
        help="Allowed XY error when validating that a fresh trial started from the configured initial pose.",
    )
    parser.add_argument(
        "--final-goal-tolerance-m",
        type=float,
        default=0.60,
        help="Required final XY distance to the goal before a SUCCEEDED action is accepted as a valid success.",
    )
    parser.add_argument(
        "--min-success-path-length-m",
        type=float,
        default=2.0,
        help="Minimum odom path length required before a SUCCEEDED action is accepted as a valid success.",
    )
    parser.add_argument("--settle-time-s", type=float, default=2.0, help="Wait time after reset before sending the goal.")
    parser.add_argument(
        "--startup-settle-time-s",
        type=float,
        default=2.0,
        help="Wait time before sending the first goal on a freshly launched stack.",
    )
    parser.add_argument(
        "--pre-stack-warmup-s",
        type=float,
        default=2.0,
        help="Extra wait after launching Gazebo before capturing initial entity states.",
    )
    parser.add_argument("--group-warmup-s", type=float, default=5.0, help="Wait time after launching each nav stack.")
    parser.add_argument(
        "--entity-wait-timeout-s",
        type=float,
        default=15.0,
        help="Timeout while waiting for tracked entities to appear in /gazebo/model_states.",
    )
    parser.add_argument(
        "--initial-snapshot-timeout-s",
        type=float,
        default=15.0,
        help="Timeout while capturing the initial Gazebo entity snapshot.",
    )
    parser.add_argument(
        "--reset-retry-count",
        type=int,
        default=3,
        help="Number of restore attempts before marking a trial as reset_failed.",
    )
    parser.add_argument(
        "--reset-retry-delay-s",
        type=float,
        default=1.0,
        help="Delay between repeated restore attempts.",
    )
    parser.add_argument(
        "--reset-verify-timeout-s",
        type=float,
        default=1.0,
        help="Extra spin time before checking whether Gazebo entities returned to the target pose.",
    )
    parser.add_argument(
        "--pose-reset-tolerance-m",
        type=float,
        default=0.12,
        help="Allowed XY error when validating a restored entity pose.",
    )
    parser.add_argument(
        "--z-reset-tolerance-m",
        type=float,
        default=0.15,
        help="Allowed Z error when validating a restored entity pose.",
    )
    parser.add_argument(
        "--max-trial-restarts",
        type=int,
        default=2,
        help="How many times a single trial may relaunch the stacks after reset_failed.",
    )
    parser.add_argument(
        "--no-restart-on-reset-failure",
        action="store_false",
        dest="restart_on_reset_failure",
        help="Disable automatic stack relaunch when Gazebo state restoration fails.",
    )
    parser.add_argument(
        "--inter-trial-cleanup-s",
        type=float,
        default=3.0,
        help="Delay after stopping stacks and force-killing leftover processes before the next launch.",
    )
    parser.add_argument("--cmd-sign-threshold", type=float, default=0.05, help="Minimum |vx| to consider a sign.")
    parser.add_argument("--flip-window-s", type=float, default=0.5, help="Maximum sign-change interval counted as hesitation.")
    parser.add_argument(
        "--publish-manual-goal-topics",
        action="store_true",
        dest="publish_manual_goal_topics",
        help="Also publish /goal_pose style topics before dispatching the action goal.",
    )
    parser.add_argument(
        "--disable-analytic-obstacle-clearance",
        action="store_false",
        dest="use_analytic_obstacle_clearance",
        help="Disable the built-in obstacle trajectory model used when Gazebo model states are unavailable.",
    )
    parser.add_argument("--goal-retry-count", type=int, default=3, help="Retries for transient goal rejection.")
    parser.add_argument("--goal-retry-delay-s", type=float, default=2.0, help="Delay between goal send retries.")
    parser.add_argument("--robot-radius", type=float, default=0.36, help="Collision radius of the robot.")
    parser.add_argument("--obstacle-radius", type=float, default=0.30, help="Collision radius of the dynamic obstacles.")
    parser.add_argument(
        "--collision-clearance-threshold",
        type=float,
        default=-0.10,
        help="Clearance threshold at or below which a collision is declared. Negative values tolerate modeling error.",
    )
    parser.add_argument(
        "--abort-on-estimated-collision",
        action="store_true",
        dest="abort_on_estimated_collision",
        help="Abort the trial immediately when the evaluator's clearance model estimates a collision.",
    )
    parser.add_argument(
        "--reuse-running-stacks",
        action="store_true",
        dest="reuse_running_stacks",
        help="Reuse the same Gazebo and Nav2 stacks across trials instead of cold-starting each trial.",
    )
    parser.add_argument("--odom-topic", default="/odom", help="Odometry topic.")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel", help="Command velocity topic.")
    parser.add_argument("--tracked-obstacles-topic", default="/tracked_obstacles", help="Tracked obstacles topic.")
    parser.add_argument("--model-states-topic", default="/gazebo/model_states", help="Gazebo model states topic.")
    parser.add_argument("--goal-action", default="/navigate_to_pose", help="NavigateToPose action name.")
    parser.add_argument("--reset-service", default="/reset_world", help="Gazebo reset world service.")
    parser.add_argument(
        "--obstacle-reset-service",
        default="/obstacle_mover/reset_motion",
        help="Service used to reset obstacle motion gating.",
    )
    parser.add_argument(
        "--set-entity-state-service",
        default="/set_entity_state",
        help="Gazebo service used to restore entity poses without rewinding /clock.",
    )
    parser.add_argument(
        "--spawn-entity-service",
        default="/spawn_entity",
        help="Gazebo service used to spawn the robot when direct entity-state reset is unavailable.",
    )
    parser.add_argument(
        "--delete-entity-service",
        default="/delete_entity",
        help="Gazebo service used to delete the robot before respawning it.",
    )
    parser.add_argument(
        "--robot-xacro-path",
        type=Path,
        default=default_root / "tcpa_sim_env" / "urdf" / "robot.xacro",
        help="Robot xacro used to respawn tcpa_robot if set_entity_state is unavailable.",
    )
    parser.add_argument(
        "--robot-lidar-backend",
        default="livox_plugin",
        help="lidar_backend argument passed to xacro when respawning the robot.",
    )
    parser.add_argument(
        "--robot-delete-timeout-s",
        type=float,
        default=5.0,
        help="Timeout while calling delete_entity during robot respawn.",
    )
    parser.add_argument(
        "--robot-delete-settle-s",
        type=float,
        default=1.0,
        help="Wait after delete_entity before respawning the robot.",
    )
    parser.add_argument(
        "--robot-spawn-timeout-s",
        type=float,
        default=40.0,
        help="Timeout while calling spawn_entity during robot respawn.",
    )
    parser.add_argument(
        "--robot-respawn-timeout-s",
        type=float,
        default=40.0,
        help="Timeout while waiting for /odom to resume after robot respawn.",
    )
    parser.add_argument("--robot-initial-x", type=float, default=-4.0)
    parser.add_argument("--robot-initial-y", type=float, default=0.0)
    parser.add_argument("--robot-initial-z", type=float, default=0.1)
    parser.add_argument("--obs1-initial-x", type=float, default=-1.0)
    parser.add_argument("--obs1-initial-y", type=float, default=-3.0)
    parser.add_argument("--obs1-initial-z", type=float, default=0.75)
    parser.add_argument("--obs2-initial-x", type=float, default=4.0)
    parser.add_argument("--obs2-initial-y", type=float, default=0.0)
    parser.add_argument("--obs2-initial-z", type=float, default=0.75)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=default_root / "ablation_eval_output",
        help="Directory for raw and summary CSV outputs.",
    )
    parser.set_defaults(restart_on_reset_failure=True)
    parser.set_defaults(use_analytic_obstacle_clearance=True)
    parser.set_defaults(publish_manual_goal_topics=False)
    parser.set_defaults(reuse_running_stacks=False)
    parser.set_defaults(abort_on_estimated_collision=False)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    workspace_src = args.workspace_src.resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    groups = [
        ExperimentGroup("Baseline", "sim_nav_dwb_baseline.sh"),
        ExperimentGroup("RiskOnly", "sim_nav_dwb_risk_only.sh"),
        ExperimentGroup("Full", "sim_nav.sh"),
    ]

    pre_script = workspace_src / "sim_pre.sh"
    if not pre_script.exists():
        raise FileNotFoundError(f"Missing pre-stack script: {pre_script}")

    for group in groups:
        script_path = workspace_src / group.script
        if not script_path.exists():
            raise FileNotFoundError(f"Missing group launch script: {script_path}")

    pre_pids: List[int] = []
    nav_pids: List[int] = []
    trial_results: List[TrialResult] = []

    rclpy.init(args=None)
    node = AblationEvaluator(args)

    try:
        if args.reuse_running_stacks:
            cleanup_active_stacks([], [], args.inter_trial_cleanup_s)
            shared_pre_logs = args.output_dir / "logs" / "shared" / "sim_pre"
            shared_pre_logs.mkdir(parents=True, exist_ok=True)
            pre_pids = start_pre_stack(node, args, pre_script, shared_pre_logs)

            for group in groups:
                node.get_logger().info(f"Starting group {group.name}")
                group_shared_logs = args.output_dir / "logs" / group.name.lower() / "shared_nav"
                group_shared_logs.mkdir(parents=True, exist_ok=True)
                nav_pids = start_group_stack(node, args, workspace_src, group, group_shared_logs)

                for trial_index in range(1, args.trials_per_group + 1):
                    node.get_logger().info(f"[{group.name}] Trial {trial_index}/{args.trials_per_group}")
                    restart_count = 0
                    result = node.run_trial(trial_index, perform_reset=True)
                    while (
                        result.outcome in {"reset_failed", "startup_failed"}
                        and args.restart_on_reset_failure
                        and restart_count < args.max_trial_restarts
                    ):
                        restart_count += 1
                        node.get_logger().warn(
                            f"[{group.name}] Trial {trial_index} failed during setup ({result.outcome}); restarting stacks "
                            f"({restart_count}/{args.max_trial_restarts})."
                        )
                        cleanup_active_stacks(pre_pids, nav_pids, args.inter_trial_cleanup_s)
                        pre_pids = []
                        nav_pids = []

                        try:
                            pre_pids = start_pre_stack(node, args, pre_script, shared_pre_logs)
                            nav_pids = start_group_stack(node, args, workspace_src, group, group_shared_logs)
                            result = node.run_trial(trial_index, perform_reset=True)
                        except Exception as exc:
                            node.get_logger().error(
                                f"[{group.name}] Trial {trial_index} startup failed after relaunch: {exc}"
                            )
                            result = node._make_failed_result(trial_index, "startup_failed")

                    result.group = group.name
                    trial_results.append(result)

                cleanup_active_stacks(pre_pids, nav_pids, args.inter_trial_cleanup_s)
                nav_pids = []
                pre_pids = []
        else:
            for group in groups:
                node.get_logger().info(f"Starting group {group.name}")
                for trial_index in range(1, args.trials_per_group + 1):
                    restart_count = 0
                    result = None
                    while True:
                        node.get_logger().info(f"[{group.name}] Trial {trial_index}/{args.trials_per_group}")
                        trial_log_dirs = build_trial_log_dirs(args.output_dir, group.name, trial_index)
                        for log_dir in trial_log_dirs.values():
                            log_dir.mkdir(parents=True, exist_ok=True)

                        cleanup_active_stacks(pre_pids, nav_pids, args.inter_trial_cleanup_s)
                        pre_pids = []
                        nav_pids = []

                        try:
                            pre_pids = start_pre_stack(node, args, pre_script, trial_log_dirs["sim_pre"])
                            nav_pids = start_group_stack(node, args, workspace_src, group, trial_log_dirs["nav"])
                            result = node.run_trial(trial_index, perform_reset=False)
                        except Exception as exc:
                            node.get_logger().error(
                                f"[{group.name}] Trial {trial_index} startup failed: {exc}"
                            )
                            result = node._make_failed_result(trial_index, "startup_failed")

                        if result.outcome not in {"reset_failed", "startup_failed"}:
                            break

                        restart_count += 1
                        node.get_logger().warn(
                            f"[{group.name}] Trial {trial_index} failed during setup ({result.outcome}); restarting cold stacks "
                            f"({restart_count}/{args.max_trial_restarts})."
                        )
                        cleanup_active_stacks(pre_pids, nav_pids, args.inter_trial_cleanup_s)
                        pre_pids = []
                        nav_pids = []

                        if not args.restart_on_reset_failure or restart_count >= args.max_trial_restarts:
                            break

                    if result is None:
                        result = node._make_failed_result(trial_index, "startup_failed")
                    result.group = group.name
                    trial_results.append(result)

                    cleanup_active_stacks(pre_pids, nav_pids, args.inter_trial_cleanup_s)
                    pre_pids = []
                    nav_pids = []

    finally:
        cleanup_active_stacks(pre_pids, nav_pids, args.inter_trial_cleanup_s)
        node.close()
        node.destroy_node()
        rclpy.shutdown()

    results_df = pd.DataFrame(asdict(result) for result in trial_results)
    raw_csv = args.output_dir / "ablation_trials.csv"
    results_df.to_csv(raw_csv, index=False)

    summary_df = summarize_results(results_df)
    summary_csv = args.output_dir / "ablation_results.csv"
    summary_df.to_csv(summary_csv, index=False)

    print(f"Saved per-trial results to {raw_csv}")
    print(f"Saved summary results to {summary_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
