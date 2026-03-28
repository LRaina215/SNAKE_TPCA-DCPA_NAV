#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def euler_from_quaternion(x, y, z, w):
    """将四元数转换为偏航角(Yaw)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def main(bag_path):
    if not os.path.exists(bag_path):
        print(f"❌ 找不到数据包路径: {bag_path}")
        return

    print(f"📂 正在解析数据包: {bag_path}")
    
    # 提取包名作为输出文件夹名
    bag_name = os.path.basename(os.path.normpath(bag_path))
    output_dir = os.path.join(os.path.dirname(bag_path), f"{bag_name}_csv")
    os.makedirs(output_dir, exist_ok=True)

    # 配置 ROS 2 Bag Reader
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', 
        output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # 获取所有话题的类型映射表
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    # 初始化数据容器
    odom_data = []
    tf_obs_data = []
    local_plan_data = []

    print("⏳ 正在逐帧提取数据，请稍候...")

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        
        # 转换时间戳为秒 (以第一帧为相对起点更好，这里先存绝对秒数)
        timestamp_sec = t * 1e-9 

        # 1. 提取机器人底盘真实轨迹 (/odom)
        if topic == '/odom':
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            yaw = euler_from_quaternion(qx, qy, qz, qw)
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            wz = msg.twist.twist.angular.z
            odom_data.append([timestamp_sec, x, y, yaw, vx, vy, wz])

        # 2. 提取动态障碍物真实轨迹 (/tf)
        elif topic == '/tf':
            for transform in msg.transforms:
                child_id = transform.child_frame_id
                # 假设障碍物的 TF 包含 'obs1' 或 'obs2'
                if 'obs1' in child_id or 'obs2' in child_id:
                    tx = transform.transform.translation.x
                    ty = transform.transform.translation.y
                    tf_obs_data.append([timestamp_sec, child_id, tx, ty])

        # 3. 提取局部避障预测轨迹 (/local_plan)
        # 将轨迹点序列化为字符串，方便在 MATLAB 中分割
        elif topic == '/local_plan':
            xs = [str(pose.pose.position.x) for pose in msg.poses]
            ys = [str(pose.pose.position.y) for pose in msg.poses]
            if xs and ys:
                x_str = ";".join(xs)
                y_str = ";".join(ys)
                local_plan_data.append([timestamp_sec, x_str, y_str])

    # 导出为 CSV 文件
    if odom_data:
        df_odom = pd.DataFrame(odom_data, columns=['time', 'x', 'y', 'yaw', 'vx', 'vy', 'wz'])
        df_odom.to_csv(os.path.join(output_dir, 'robot_odom.csv'), index=False)
        print(f"✅ 已生成: robot_odom.csv ({len(odom_data)} 行)")

    if tf_obs_data:
        df_tf = pd.DataFrame(tf_obs_data, columns=['time', 'obstacle_id', 'x', 'y'])
        df_tf.to_csv(os.path.join(output_dir, 'obstacles_gt.csv'), index=False)
        print(f"✅ 已生成: obstacles_gt.csv ({len(tf_obs_data)} 行)")

    if local_plan_data:
        df_local = pd.DataFrame(local_plan_data, columns=['time', 'path_x', 'path_y'])
        df_local.to_csv(os.path.join(output_dir, 'local_plan.csv'), index=False)
        print(f"✅ 已生成: local_plan.csv ({len(local_plan_data)} 行)")

    print(f"\n🎉 提取完成！所有 CSV 文件已保存在目录:\n {output_dir}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 extract_bag_to_csv.py <数据包文件夹路径>")
    else:
        main(sys.argv[1])
