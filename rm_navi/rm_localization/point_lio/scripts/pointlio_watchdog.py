#!/usr/bin/env python3

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from livox_ros_driver2.msg import CustomMsg


class TopicMonitor:
    def __init__(self, name: str):
        self.name = name
        self.count = 0
        self.last_recv_time = None
        self.recv_times = deque(maxlen=200)

    def tick(self, now_sec: float) -> None:
        self.count += 1
        self.last_recv_time = now_sec
        self.recv_times.append(now_sec)

    def hz(self) -> float:
        if len(self.recv_times) < 2:
            return 0.0
        duration = self.recv_times[-1] - self.recv_times[0]
        if duration <= 0.0:
            return 0.0
        return (len(self.recv_times) - 1) / duration

    def age(self, now_sec: float) -> float:
        if self.last_recv_time is None:
            return float("inf")
        return now_sec - self.last_recv_time


class PointlioWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("pointlio_watchdog")

        sensor_qos = qos_profile_sensor_data
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.monitors = {
            "lidar_custom": TopicMonitor("lidar_custom"),
            "imu": TopicMonitor("imu"),
            "cloud_registered": TopicMonitor("cloud_registered"),
        }

        self.create_subscription(CustomMsg, "/livox/lidar_custom", self.lidar_cb, sensor_qos)
        self.create_subscription(Imu, "/livox/imu", self.imu_cb, sensor_qos)
        self.create_subscription(PointCloud2, "/cloud_registered", self.cloud_cb, cloud_qos)

        self.create_timer(1.0, self.report)
        self.get_logger().info("Point-LIO watchdog started")

    def lidar_cb(self, _msg: CustomMsg) -> None:
        self.monitors["lidar_custom"].tick(self.now_sec())

    def imu_cb(self, _msg: Imu) -> None:
        self.monitors["imu"].tick(self.now_sec())

    def cloud_cb(self, _msg: PointCloud2) -> None:
        self.monitors["cloud_registered"].tick(self.now_sec())

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def report(self) -> None:
        now = self.now_sec()
        node_names = {name.lstrip("/") for name in self.get_node_names()}
        laser_mapping_alive = "laserMapping" in node_names

        lidar = self.monitors["lidar_custom"]
        imu = self.monitors["imu"]
        cloud = self.monitors["cloud_registered"]

        self.get_logger().info(
            "laserMapping=%s | /livox/lidar_custom: %.1f Hz age=%.2fs | "
            "/livox/imu: %.1f Hz age=%.2fs | /cloud_registered: %.1f Hz age=%.2fs"
            % (
                "alive" if laser_mapping_alive else "missing",
                lidar.hz(), lidar.age(now),
                imu.hz(), imu.age(now),
                cloud.hz(), cloud.age(now),
            )
        )


def main() -> None:
    rclpy.init()
    node = PointlioWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
