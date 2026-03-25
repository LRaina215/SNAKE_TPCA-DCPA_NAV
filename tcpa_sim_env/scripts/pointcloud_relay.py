#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class PointCloudRelay(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_relay')
        self.declare_parameter('input_topic', '/livox/lidar_custom_PointCloud2')
        self.declare_parameter('output_topic', '/livox/lidar')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self._callback,
            10,
        )

    def _callback(self, msg: PointCloud2) -> None:
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = PointCloudRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
