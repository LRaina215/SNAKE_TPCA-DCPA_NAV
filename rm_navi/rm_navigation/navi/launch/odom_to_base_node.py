#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

def q_inverse(q):
    return [-q[0], -q[1], -q[2], q[3]]

def q_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ]

def qv_mult(q1, v1):
    q2 = [v1[0], v1[1], v1[2], 0.0]
    return q_mult(q_mult(q1, q2), q_inverse(q1))[0:3]

def yaw_from_quat(q):
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def quat_from_yaw(yaw):
    half = yaw * 0.5
    return [0.0, 0.0, math.sin(half), math.cos(half)]

def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class OdomToBaseTF(Node):
    def __init__(self):
        super().__init__(
            'odom_to_base_tf',
            automatically_declare_parameters_from_overrides=True
        )
        self.max_xy_step = self.declare_parameter('max_xy_step', 1.0).value
        self.max_yaw_step = math.radians(
            self.declare_parameter('max_yaw_step_deg', 20.0).value
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.cached_tf = None
        self.latest_pose = None
        self.latest_twist = None
        self.last_accepted_pose = None
        self.subscription = self.create_subscription(Odometry, '/odom_livox', self.odom_callback, 10)
        self.timer = self.create_timer(0.02, self.publish_cached_state)
        
        # 发布专供 Nav2 / 下游过滤链读取的底盘里程计
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

    def odom_callback(self, msg):
        if self.cached_tf is None:
            try:
                self.cached_tf = self.tf_buffer.lookup_transform(
                    'livox_frame', 'base_link', rclpy.time.Time())
                self.get_logger().info("成功缓存 URDF 外参，开始同步 TF 与 底盘速度！")
            except Exception as e:
                return

        # 1. 计算 TF 位置
        pos_ol = msg.pose.pose.position
        q_ol = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        t_ol = [pos_ol.x, pos_ol.y, pos_ol.z]

        pos_lb = self.cached_tf.transform.translation
        q_lb = [self.cached_tf.transform.rotation.x, self.cached_tf.transform.rotation.y, 
                self.cached_tf.transform.rotation.z, self.cached_tf.transform.rotation.w]
        t_lb = [pos_lb.x, pos_lb.y, pos_lb.z]

        q_ob = q_mult(q_ol, q_lb)
        t_rotated = qv_mult(q_ol, t_lb)
        t_ob = [t_ol[0] + t_rotated[0], t_ol[1] + t_rotated[1], t_ol[2] + t_rotated[2]]
        yaw = yaw_from_quat(q_ob)

        if self.last_accepted_pose is not None:
            dx = t_ob[0] - self.last_accepted_pose['x']
            dy = t_ob[1] - self.last_accepted_pose['y']
            dxy = math.hypot(dx, dy)
            dyaw = abs(wrap_angle(yaw - self.last_accepted_pose['yaw']))
            if dxy > self.max_xy_step or dyaw > self.max_yaw_step:
                self.get_logger().warn(
                    f"Rejecting odom jump dxy={dxy:.3f} m dyaw={dyaw:.3f} rad"
                )
                return

        planar_q = quat_from_yaw(yaw)
        self.last_accepted_pose = {
            'x': t_ob[0],
            'y': t_ob[1],
            'yaw': yaw,
        }

        # 2. 用逆矩阵把雷达的速度旋转回底盘！
        v_l = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        w_l = [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]

        q_bl = q_inverse(q_lb) 
        v_b = qv_mult(q_bl, v_l) 
        w_b = qv_mult(q_bl, w_l) 

        # 3. 缓存最新位姿，50Hz timer 会盖上最新仿真时间持续发布，
        # 避免 Point-LIO 帧率下降时下游 TF 饥饿。
        self.latest_pose = {
            'x': t_ob[0],
            'y': t_ob[1],
            'z': 0.0,
            'qx': planar_q[0],
            'qy': planar_q[1],
            'qz': planar_q[2],
            'qw': planar_q[3],
        }
        self.latest_twist = {
            'vx': v_b[0],
            'vy': v_b[1],
            'vz': 0.0,
            'wx': 0.0,
            'wy': 0.0,
            'wz': w_b[2],
        }

    def publish_cached_state(self):
        if self.latest_pose is None or self.latest_twist is None:
            return

        stamp = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.latest_pose['x']
        t.transform.translation.y = self.latest_pose['y']
        t.transform.translation.z = self.latest_pose['z']
        t.transform.rotation.x = self.latest_pose['qx']
        t.transform.rotation.y = self.latest_pose['qy']
        t.transform.rotation.z = self.latest_pose['qz']
        t.transform.rotation.w = self.latest_pose['qw']
        self.tf_broadcaster.sendTransform(t)

        odom_base = Odometry()
        odom_base.header.stamp = stamp
        odom_base.header.frame_id = 'odom'
        odom_base.child_frame_id = 'base_link'

        odom_base.pose.pose.position.x = self.latest_pose['x']
        odom_base.pose.pose.position.y = self.latest_pose['y']
        odom_base.pose.pose.position.z = self.latest_pose['z']
        odom_base.pose.pose.orientation.x = self.latest_pose['qx']
        odom_base.pose.pose.orientation.y = self.latest_pose['qy']
        odom_base.pose.pose.orientation.z = self.latest_pose['qz']
        odom_base.pose.pose.orientation.w = self.latest_pose['qw']

        odom_base.twist.twist.linear.x = self.latest_twist['vx']
        odom_base.twist.twist.linear.y = self.latest_twist['vy']
        odom_base.twist.twist.linear.z = self.latest_twist['vz']
        odom_base.twist.twist.angular.x = self.latest_twist['wx']
        odom_base.twist.twist.angular.y = self.latest_twist['wy']
        odom_base.twist.twist.angular.z = self.latest_twist['wz']

        self.odom_pub.publish(odom_base)

# 👇 就是漏了下面这几行救命的启动代码 👇
def main(args=None):
    rclpy.init(args=args)
    node = OdomToBaseTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
