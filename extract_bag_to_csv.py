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
    
    bag_name = os.path.basename(os.path.normpath(bag_path))
    output_dir = os.path.join(os.path.dirname(bag_path), f"{bag_name}_csv")
    os.makedirs(output_dir, exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', 
        output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    odom_data = []
    tf_obs_data = []
    local_plan_data = []
    
    # 用于调试：记录包里到底有哪些 TF
    unique_tf_frames = set()

    print("⏳ 正在逐帧提取数据，请稍候...")

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        
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

        # 2. 尝试从 TF 提取障碍物 (兜底方案)
        elif topic == '/tf':
            for transform in msg.transforms:
                child_id = transform.child_frame_id
                unique_tf_frames.add(child_id)
                # 扩大搜索范围：只要名字里带 obs 或 track 就抓出来
                if 'obs' in child_id.lower() or 'track' in child_id.lower():
                    tx = transform.transform.translation.x
                    ty = transform.transform.translation.y
                    tf_obs_data.append([timestamp_sec, child_id, tx, ty])

        # 3. 核心大招：直接从感知模块提取追踪到的障碍物位置！
        elif topic == '/tracked_obstacles':
            try:
                # 动态探查你的自定义消息结构 (常见的是 obstacles, tracks 或 array)
                obstacles = getattr(msg, 'obstacles', getattr(msg, 'tracks', None))
                if obstacles is not None:
                    for idx, obs in enumerate(obstacles):
                        # 动态探查位置字段 (position 或者 pose.pose.position)
                        pos = getattr(obs, 'position', None)
                        if pos is None:
                            pose = getattr(obs, 'pose', None)
                            if pose:
                                # 有些嵌套是 pose.pose.position
                                pos = getattr(pose, 'position', getattr(getattr(pose, 'pose', None), 'position', None))
                        
                        if pos is not None:
                            # 伪装成 obs1_link 喂给 MATLAB
                            # 假设只画检测到的第一个障碍物
                            tf_obs_data.append([timestamp_sec, f'obs1_link', pos.x, pos.y])
            except Exception as e:
                pass # 静默跳过解析错误

        # 4. 提取局部避障预测轨迹 (/local_plan)
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
        print(f"✅ 已生成: obstacles_gt.csv ({len(tf_obs_data)} 行) -> 包含感知模块抓取的数据！")
    else:
        print("❌ 警告：TF 和 /tracked_obstacles 中均未找到障碍物数据！")
        print("🔍 供调试参考，包中实际存在的 TF child_frame 如下:")
        print(unique_tf_frames)

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