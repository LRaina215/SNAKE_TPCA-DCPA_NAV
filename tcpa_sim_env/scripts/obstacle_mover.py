#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class ObstacleMover(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_mover')
        self.motion_enabled = False
        self.motion_start_time = None
        self.start_speed_threshold = 0.05

        self.obs1_pub = self.create_publisher(Twist, '/obs1/cmd_vel', 10)
        self.obs2_pub = self.create_publisher(Twist, '/obs2/cmd_vel', 10)
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
        self.get_logger().info(
            'Waiting for first nav goal or non-zero cmd_vel before starting obstacle motion.'
        )

    def _enable_motion(self, reason: str) -> None:
        if self.motion_enabled:
            return

        self.motion_enabled = True
        self.motion_start_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f'Starting obstacle motion: {reason}')

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

    def _handle_reset_motion(self, _request: Empty.Request, response: Empty.Response) -> Empty.Response:
        self.motion_enabled = False
        self.motion_start_time = None
        stop = Twist()
        self.obs1_pub.publish(stop)
        self.obs2_pub.publish(stop)
        self.get_logger().info('Obstacle motion reset; waiting for next goal trigger.')
        return response

    def _publish_commands(self) -> None:
        obs1_cmd = Twist()
        obs2_cmd = Twist()

        if not self.motion_enabled or self.motion_start_time is None:
            self.obs1_pub.publish(obs1_cmd)
            self.obs2_pub.publish(obs2_cmd)
            return

        sim_time = self.get_clock().now().nanoseconds * 1e-9
        elapsed = sim_time - self.motion_start_time
        obs1_phase = elapsed % 8.0
        obs2_phase = elapsed % 8.0

        obs1_cmd.linear.y = 1.2 if obs1_phase < 4.0 else -1.2

        obs2_cmd.linear.x = -1.0 if obs2_phase < 4.0 else 1.0

        self.obs1_pub.publish(obs1_cmd)
        self.obs2_pub.publish(obs2_cmd)


def main() -> None:
    rclpy.init()
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    finally:
        stop = Twist()
        node.obs1_pub.publish(stop)
        node.obs2_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
