#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class ObstacleMover(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_mover')
        self.obs1_pub = self.create_publisher(Twist, '/obs1/cmd_vel', 10)
        self.obs2_pub = self.create_publisher(Twist, '/obs2/cmd_vel', 10)
        self.create_timer(0.05, self._publish_commands)

    def _publish_commands(self) -> None:
        sim_time = self.get_clock().now().nanoseconds * 1e-9
        obs1_phase = sim_time % 8.0
        obs2_phase = sim_time % 8.0

        obs1_cmd = Twist()
        obs1_cmd.linear.y = 1.2 if obs1_phase < 4.0 else -1.2

        obs2_cmd = Twist()
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
