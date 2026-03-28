#!/bin/bash
# TCPA-DCPA 论文实验数据录制脚本

# 定义存放数据的总目录
DATA_DIR="$HOME/auto_shao/data"
mkdir -p "$DATA_DIR"

# 自动生成时间戳 (格式: 年月日_时分秒)，防止包名重复覆盖
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# 允许用户在运行脚本时传入一个自定义前缀，否则默认为 tcpa_exp
EXP_PREFIX=${1:-"tcpa_exp"}
BAG_NAME="${DATA_DIR}/${EXP_PREFIX}_${TIMESTAMP}"

echo "====================================================="
echo " 🤖 TCPA-DCPA 避障实验录制工具 (ROS 2 Bag Recorder)"
echo "====================================================="
echo "请选择要录制的数据包类型："
echo "  [1] 论文主包 (推荐！轻量级，含核心轨迹与动态障碍数据)"
echo "  [2] 系统分析版 (沉重！含全局/局部 Costmap 和 3D 点云)"
echo "====================================================="
read -p "请输入选项 (1 或 2): " choice

echo ""

if [ "$choice" == "1" ]; then
    echo "▶ 正在录制【论文主包】..."
    echo "📂 保存路径: $BAG_NAME"
    echo "⏳ 随时按 Ctrl+C 停止并保存录制。"
    echo "-----------------------------------------------------"
    ros2 bag record -o "$BAG_NAME" \
        /clock \
        /odom \
        /cmd_vel \
        /plan \
        /local_plan \
        /tf \
        /tf_static \
        /scan_nav \
        /tracked_obstacles \
        /obs1/cmd_vel \
        /obs2/cmd_vel

elif [ "$choice" == "2" ]; then
    echo "▶ 正在录制【系统分析版】..."
    echo "📂 保存路径: $BAG_NAME"
    echo "⏳ 随时按 Ctrl+C 停止并保存录制。"
    echo "⚠️  注意：此模式包含大量栅格和点云数据，可能引起轻微卡顿。"
    echo "-----------------------------------------------------"
    ros2 bag record -o "$BAG_NAME" \
        /clock \
        /odom \
        /cmd_vel \
        /plan \
        /local_plan \
        /tf \
        /tf_static \
        /scan_nav \
        /tracked_obstacles \
        /tracked_obstacle_markers \
        /local_costmap/costmap \
        /global_costmap/costmap \
        /segmentation/obstacle \
        /obs1/cmd_vel \
        /obs2/cmd_vel
else
    echo "❌ 无效选项，脚本已退出。"
    exit 1
fi