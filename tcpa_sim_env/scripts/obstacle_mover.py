#!/usr/bin/env python3

import math
import random
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


@dataclass
class ObstacleState:
    name: str
    x: float
    y: float
    speed: float
    mode: str
    min_x: float = -5.0
    max_x: float = 5.0
    min_y: float = -5.0
    max_y: float = 5.0
    heading: float = 0.0
    turn_interval_min: float = 2.0
    turn_interval_max: float = 4.0
    switch_period: float = 8.0
    axis: str = 'x'
    direction: float = 1.0
    turn_deadline: float = 0.0

    def reset_runtime(self) -> None:
        self.turn_deadline = 0.0


class ObstacleMover(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_mover')

        self.scene_mode = self.declare_parameter('scene_mode', 'dynamic_test').value
        self.random_seed = int(self.declare_parameter('random_seed', 42).value)
        self.start_speed_threshold = float(
            self.declare_parameter('start_speed_threshold', 0.05).value
        )

        self.motion_enabled = False
        self.motion_start_time: Optional[float] = None
        self.last_update_time: Optional[float] = None
        self.rng = random.Random(self.random_seed)
        self.random_walk_wall_margin = 1.0
        self.random_walk_wall_gain = 1.3
        self.random_walk_neighbor_gain = 1.9
        self.random_walk_neighbor_radius = 1.6
        self.random_walk_hard_separation = 0.95

        self.obstacle_states = self._create_scene_states(self.scene_mode)
        self.cmd_publishers = {
            name: self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            for name in self.obstacle_states
        }

        self.goal_subs = [
            self.create_subscription(PoseStamped, '/goal_pose', self._on_goal_pose, 10),
            self.create_subscription(PoseStamped, 'goal_pose', self._on_goal_pose, 10),
            self.create_subscription(
                PoseStamped, '/move_base_simple/goal', self._on_goal_pose, 10
            ),
            self.create_subscription(
                PoseStamped, 'move_base_simple/goal', self._on_goal_pose, 10
            ),
        ]
        self.cmd_vel_subs = [
            self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10),
            self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10),
        ]
        self.reset_srv = self.create_service(Empty, 'reset_motion', self._handle_reset_motion)
        self.create_timer(0.05, self._publish_commands)

        obstacle_names = ', '.join(self.obstacle_states.keys())
        self.get_logger().info(
            f'Obstacle mover ready for scene_mode={self.scene_mode}; obstacles=[{obstacle_names}]'
        )
        self.get_logger().info(
            'Waiting for first nav goal or non-zero cmd_vel before starting obstacle motion.'
        )

    def _create_scene_states(self, scene_mode: str) -> Dict[str, ObstacleState]:
        if scene_mode == 'dynamic_test':
            return {
                'obs1': ObstacleState(
                    name='obs1',
                    x=-1.0,
                    y=-3.0,
                    speed=1.2,
                    mode='oscillate',
                    axis='y',
                    direction=1.0,
                    switch_period=8.0,
                ),
                'obs2': ObstacleState(
                    name='obs2',
                    x=4.0,
                    y=0.0,
                    speed=1.0,
                    mode='oscillate',
                    axis='x',
                    direction=-1.0,
                    switch_period=8.0,
                ),
            }

        if scene_mode == 'narrow_corridor':
            return {
                'obs1': ObstacleState(
                    name='obs1',
                    x=2.4,
                    y=0.0,
                    speed=1.1,
                    mode='oscillate',
                    axis='x',
                    direction=-1.0,
                    switch_period=7.0,
                    min_x=-3.0,
                    max_x=3.0,
                    min_y=-0.2,
                    max_y=0.2,
                ),
            }

        if scene_mode == 'random_crowd':
            states = {
                'obs1': ObstacleState(
                    name='obs1',
                    x=-1.2,
                    y=-2.1,
                    speed=0.9,
                    mode='random_walk',
                    min_x=-4.25,
                    max_x=4.25,
                    min_y=-4.25,
                    max_y=4.25,
                    heading=0.35,
                ),
                'obs2': ObstacleState(
                    name='obs2',
                    x=1.7,
                    y=-1.4,
                    speed=0.8,
                    mode='random_walk',
                    min_x=-4.25,
                    max_x=4.25,
                    min_y=-4.25,
                    max_y=4.25,
                    heading=2.35,
                ),
                'obs3': ObstacleState(
                    name='obs3',
                    x=-0.4,
                    y=1.8,
                    speed=0.85,
                    mode='random_walk',
                    min_x=-4.25,
                    max_x=4.25,
                    min_y=-4.25,
                    max_y=4.25,
                    heading=-0.95,
                ),
                'obs4': ObstacleState(
                    name='obs4',
                    x=2.0,
                    y=2.2,
                    speed=0.75,
                    mode='random_walk',
                    min_x=-4.25,
                    max_x=4.25,
                    min_y=-4.25,
                    max_y=4.25,
                    heading=-2.4,
                ),
                'obs5': ObstacleState(
                    name='obs5',
                    x=-2.3,
                    y=0.9,
                    speed=0.85,
                    mode='random_walk',
                    min_x=-4.25,
                    max_x=4.25,
                    min_y=-4.25,
                    max_y=4.25,
                    heading=0.9,
                ),
            }
            for index, state in enumerate(states.values()):
                state.turn_interval_min = 1.6 + 0.1 * index
                state.turn_interval_max = 3.2 + 0.15 * index
            return states

        self.get_logger().warn(
            f'Unknown scene_mode={scene_mode}; falling back to dynamic_test profile.'
        )
        return self._create_scene_states('dynamic_test')

    def _sim_time_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _enable_motion(self, reason: str) -> None:
        if self.motion_enabled:
            return

        self.motion_enabled = True
        now = self._sim_time_now()
        self.motion_start_time = now
        self.last_update_time = now
        for state in self.obstacle_states.values():
            self._initialize_state_runtime(state, now)
        self.get_logger().info(f'Starting obstacle motion: {reason}')

    def _initialize_state_runtime(self, state: ObstacleState, now: float) -> None:
        state.reset_runtime()
        if state.mode == 'random_walk':
            state.turn_deadline = now + self._next_turn_interval(state)

    def _next_turn_interval(self, state: ObstacleState) -> float:
        if state.turn_interval_max <= state.turn_interval_min:
            return state.turn_interval_min
        return self.rng.uniform(state.turn_interval_min, state.turn_interval_max)

    def _choose_random_heading(self, state: ObstacleState) -> None:
        jitter = self.rng.uniform(-1.15, 1.15)
        candidate_heading = state.heading + jitter
        wall_x, wall_y = self._compute_wall_avoidance_vector(state)
        if math.hypot(wall_x, wall_y) > 1e-6:
            inward_heading = math.atan2(wall_y, wall_x)
            candidate_heading = self._blend_angles(candidate_heading, inward_heading, 0.55)
        state.heading = self._wrap_angle(candidate_heading)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _blend_angles(self, from_angle: float, to_angle: float, weight: float) -> float:
        delta = self._wrap_angle(to_angle - from_angle)
        return self._wrap_angle(from_angle + weight * delta)

    @staticmethod
    def _normalize(x: float, y: float) -> Tuple[float, float, float]:
        norm = math.hypot(x, y)
        if norm < 1e-6:
            return 0.0, 0.0, 0.0
        return x / norm, y / norm, norm

    def _compute_wall_avoidance_vector(self, state: ObstacleState) -> Tuple[float, float]:
        repulse_x = 0.0
        repulse_y = 0.0

        left_gap = state.x - state.min_x
        right_gap = state.max_x - state.x
        bottom_gap = state.y - state.min_y
        top_gap = state.max_y - state.y

        if left_gap < self.random_walk_wall_margin:
            repulse_x += self.random_walk_wall_gain * (
                (self.random_walk_wall_margin - left_gap) / self.random_walk_wall_margin
            )
        if right_gap < self.random_walk_wall_margin:
            repulse_x -= self.random_walk_wall_gain * (
                (self.random_walk_wall_margin - right_gap) / self.random_walk_wall_margin
            )
        if bottom_gap < self.random_walk_wall_margin:
            repulse_y += self.random_walk_wall_gain * (
                (self.random_walk_wall_margin - bottom_gap) / self.random_walk_wall_margin
            )
        if top_gap < self.random_walk_wall_margin:
            repulse_y -= self.random_walk_wall_gain * (
                (self.random_walk_wall_margin - top_gap) / self.random_walk_wall_margin
            )

        return repulse_x, repulse_y

    def _compute_neighbor_avoidance_vector(
        self,
        state: ObstacleState,
        positions: Dict[str, Tuple[float, float]],
    ) -> Tuple[float, float]:
        repulse_x = 0.0
        repulse_y = 0.0

        for other_name, (other_x, other_y) in positions.items():
            if other_name == state.name:
                continue

            delta_x = state.x - other_x
            delta_y = state.y - other_y
            distance = math.hypot(delta_x, delta_y)
            if distance < 1e-6:
                escape_heading = self.rng.uniform(-math.pi, math.pi)
                repulse_x += math.cos(escape_heading) * self.random_walk_neighbor_gain
                repulse_y += math.sin(escape_heading) * self.random_walk_neighbor_gain
                continue

            if distance >= self.random_walk_neighbor_radius:
                continue

            strength = self.random_walk_neighbor_gain * (
                (self.random_walk_neighbor_radius - distance) / self.random_walk_neighbor_radius
            )
            repulse_x += strength * delta_x / distance
            repulse_y += strength * delta_y / distance

        return repulse_x, repulse_y

    def _compute_random_walk_step(
        self,
        state: ObstacleState,
        positions: Dict[str, Tuple[float, float]],
        dt: float,
        now: float,
    ) -> Tuple[float, float, float, float]:
        base_x = state.speed * math.cos(state.heading)
        base_y = state.speed * math.sin(state.heading)
        wall_x, wall_y = self._compute_wall_avoidance_vector(state)
        neighbor_x, neighbor_y = self._compute_neighbor_avoidance_vector(state, positions)

        desired_x = base_x + wall_x + neighbor_x
        desired_y = base_y + wall_y + neighbor_y
        dir_x, dir_y, desired_norm = self._normalize(desired_x, desired_y)
        if desired_norm < 1e-6:
            dir_x = math.cos(state.heading)
            dir_y = math.sin(state.heading)

        vx = dir_x * state.speed
        vy = dir_y * state.speed
        next_x = state.x + vx * dt
        next_y = state.y + vy * dt

        if self._reflect_heading_if_needed(state, next_x, next_y):
            base_x = state.speed * math.cos(state.heading)
            base_y = state.speed * math.sin(state.heading)
            dir_x, dir_y, _ = self._normalize(base_x + wall_x, base_y + wall_y)
            if abs(dir_x) < 1e-6 and abs(dir_y) < 1e-6:
                dir_x = math.cos(state.heading)
                dir_y = math.sin(state.heading)
            vx = dir_x * state.speed
            vy = dir_y * state.speed
            next_x = state.x + vx * dt
            next_y = state.y + vy * dt
            state.turn_deadline = now + self._next_turn_interval(state)

        for other_name, (other_x, other_y) in positions.items():
            if other_name == state.name:
                continue

            predicted_distance = math.hypot(next_x - other_x, next_y - other_y)
            if predicted_distance >= self.random_walk_hard_separation:
                continue

            escape_x = state.x - other_x
            escape_y = state.y - other_y
            away_x, away_y, away_norm = self._normalize(escape_x, escape_y)
            if away_norm < 1e-6:
                away_x = -dir_y
                away_y = dir_x

            tangent_x = -away_y
            tangent_y = away_x
            if tangent_x * dir_x + tangent_y * dir_y < 0.0:
                tangent_x = -tangent_x
                tangent_y = -tangent_y

            steer_x = away_x + 0.55 * tangent_x
            steer_y = away_y + 0.55 * tangent_y
            steer_x, steer_y, _ = self._normalize(steer_x, steer_y)
            safe_speed = state.speed * 0.7
            vx = steer_x * safe_speed
            vy = steer_y * safe_speed
            next_x = state.x + vx * dt
            next_y = state.y + vy * dt
            state.heading = math.atan2(steer_y, steer_x)
            state.turn_deadline = min(state.turn_deadline, now + 0.8)
            break

        next_x = min(max(next_x, state.min_x), state.max_x)
        next_y = min(max(next_y, state.min_y), state.max_y)
        state.heading = math.atan2(vy, vx)
        return vx, vy, next_x, next_y

    def _reflect_heading_if_needed(self, state: ObstacleState, next_x: float, next_y: float) -> bool:
        bounced = False
        if next_x <= state.min_x or next_x >= state.max_x:
            state.heading = math.pi - state.heading
            bounced = True
        if next_y <= state.min_y or next_y >= state.max_y:
            state.heading = -state.heading
            bounced = True
        return bounced

    def _on_goal_pose(self, _msg: PoseStamped) -> None:
        self._enable_motion('received navigation goal')

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self.motion_enabled:
            return

        linear_speed = max(abs(msg.linear.x), abs(msg.linear.y))
        angular_speed = abs(msg.angular.z)
        if linear_speed < self.start_speed_threshold and angular_speed < self.start_speed_threshold:
            return

        self._enable_motion('received non-zero cmd_vel')

    def _handle_reset_motion(
        self, _request: Empty.Request, response: Empty.Response
    ) -> Empty.Response:
        self.motion_enabled = False
        self.motion_start_time = None
        self.last_update_time = None
        self.rng.seed(self.random_seed)
        self.obstacle_states = self._create_scene_states(self.scene_mode)
        for state in self.obstacle_states.values():
            state.reset_runtime()
        self._publish_stop()
        self.get_logger().info('Obstacle motion reset; waiting for next goal trigger.')
        return response

    def _publish_stop(self) -> None:
        stop = Twist()
        for publisher in self.cmd_publishers.values():
            publisher.publish(stop)

    def _publish_commands(self) -> None:
        if not self.motion_enabled or self.motion_start_time is None:
            self._publish_stop()
            return

        now = self._sim_time_now()
        if self.last_update_time is None:
            self.last_update_time = now
        dt = max(0.0, min(0.2, now - self.last_update_time))
        self.last_update_time = now
        positions = {
            name: (state.x, state.y)
            for name, state in self.obstacle_states.items()
        }
        random_walk_steps: Dict[str, Tuple[float, float, float, float]] = {}

        for state in self.obstacle_states.values():
            if state.mode != 'random_walk':
                continue

            if now >= state.turn_deadline:
                self._choose_random_heading(state)
                state.turn_deadline = now + self._next_turn_interval(state)

            random_walk_steps[state.name] = self._compute_random_walk_step(
                state, positions, dt, now
            )

        for state in self.obstacle_states.values():
            cmd = Twist()
            if state.mode == 'oscillate':
                elapsed = now - self.motion_start_time
                phase = elapsed % state.switch_period
                direction = state.direction if phase < state.switch_period / 2.0 else -state.direction
                if state.axis == 'x':
                    cmd.linear.x = state.speed * direction
                else:
                    cmd.linear.y = state.speed * direction
            elif state.mode == 'random_walk':
                vx, vy, next_x, next_y = random_walk_steps.get(
                    state.name, (0.0, 0.0, state.x, state.y)
                )
                state.x = next_x
                state.y = next_y
                cmd.linear.x = vx
                cmd.linear.y = vy

            publisher = self.cmd_publishers.get(state.name)
            if publisher is not None:
                publisher.publish(cmd)


def main() -> None:
    rclpy.init()
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
