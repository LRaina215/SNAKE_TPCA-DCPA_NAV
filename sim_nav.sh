#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WORKSPACE_DIR}/install/setup.bash"
MAP_FILE="${SCRIPT_DIR}/rm_navi/rm_navigation/navi/maps/arena_map.yaml"

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "Missing workspace setup: ${SETUP_FILE}"
  exit 1
fi

if [[ ! -f "${MAP_FILE}" ]]; then
  echo "Missing map yaml: ${MAP_FILE}"
  exit 1
fi

set +u
source "${SETUP_FILE}"
set -u
export RCUTILS_LOGGING_BUFFERED_STREAM=1

tabs=(
  "Localization|ros2 launch navi localization_launch.py use_sim_time:=True map:=${MAP_FILE}"
  "MapTF|ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map odom"
  "Nav2|ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=${SCRIPT_DIR}/rm_navi/rm_navigation/navi/params/nav2_params.yaml"
  "RVIZ|ros2 run rviz2 rviz2 -d ${SCRIPT_DIR}/rm_navi/rm_navigation/navi/rviz/nav2_default_view.rviz"
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

echo "Opened all navigation tabs. Suggested checks:"
echo "  ros2 run tf2_ros tf2_echo map odom"
echo "  ros2 topic echo /scan --num 1"
echo "  ros2 node list"
