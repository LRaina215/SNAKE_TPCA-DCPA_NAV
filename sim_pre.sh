#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WORKSPACE_DIR}/install/setup.bash"

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "Missing workspace setup: ${SETUP_FILE}"
  exit 1
fi

set +u
source "${SETUP_FILE}"
set -u
export RCUTILS_LOGGING_BUFFERED_STREAM=1

tabs=(
  "Sim|ros2 launch tcpa_sim_env sim_launch.py"
  "Point-LIO|ros2 launch point_lio mapping_mid360.launch.py use_sim_time:=True rviz:=false"
  # "Segmentation|ros2 launch terrain_analysis terrain_analysis.launch use_sim_time:=True"
  "Segmentation|ros2 launch linefit_ground_segmentation_ros segmentation.launch.py"
  "Tracker|ros2 launch predictive_tracker dynamic_tracker.launch.py use_sim_time:=True"
  "Scan|ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py use_sim_time:=True target_frame:=base_link"
)

is_first_tab=true

for i in "${!tabs[@]}"; do
  title="${tabs[$i]%%|*}"
  cmd="${tabs[$i]#*|}"

  echo "Launching [${title}] ${cmd}"

  if [ "$is_first_tab" = true ]; then
    # 第一个任务打开新窗口
    gnome-terminal --window --title="${title}" --working-directory="${SCRIPT_DIR}" -- bash -c "source ${SETUP_FILE} && export RCUTILS_LOGGING_BUFFERED_STREAM=1 && ${cmd}; exec bash"
    is_first_tab=false
    sleep 0.5 # 稍作等待，确保主窗口已经建好
  else
    # 后续任务在当前激活的窗口中打开新标签
    gnome-terminal --tab --title="${title}" --working-directory="${SCRIPT_DIR}" -- bash -c "source ${SETUP_FILE} && export RCUTILS_LOGGING_BUFFERED_STREAM=1 && ${cmd}; exec bash"
  fi
done

echo "Opened all pre-simulation tabs. Suggested checks:"
echo "  ros2 topic echo /clock --num 1"
echo "  ros2 topic echo /cloud_registered --num 1"
echo "  ros2 topic echo /segmentation/obstacle --num 1"
