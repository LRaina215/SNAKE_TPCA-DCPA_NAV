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

HEADLESS_MODE="${AUTO_EVAL_HEADLESS:-0}"
PID_FILE="${AUTO_EVAL_PID_FILE:-}"
LOG_DIR="${AUTO_EVAL_LOG_DIR:-${SCRIPT_DIR}/.auto_eval_logs/pre_real}"

tabs=(
  "Livox|ros2 launch livox_ros_driver2 msg_MID360_cloud_launch.py"
  "Description|ros2 launch rm_description model.launch.py"
  "Point-LIO|ros2 launch point_lio mapping_mid360.launch.py use_sim_time:=False rviz:=false config_file:=${SCRIPT_DIR}/rm_navi/rm_localization/point_lio/config/mid360_real.yaml"
  "OdomBridge|python3 ${SCRIPT_DIR}/rm_navi/rm_navigation/navi/launch/odom_to_base_node.py"
  "LidarFilter|ros2 run rm_lidar_filter lidar_filter"
  "Segmentation|ros2 launch linefit_ground_segmentation_ros segmentation_real.launch.py"
  "Tracker|ros2 launch predictive_tracker dynamic_tracker.launch.py use_sim_time:=False"
  "Scan|ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py use_sim_time:=False target_frame:=base_link"
)

launch_headless() {
  local title="$1"
  local cmd="$2"
  local log_file="${LOG_DIR}/${title}.log"

  mkdir -p "${LOG_DIR}"
  echo "Launching headless [${title}] ${cmd}"
  setsid bash -lc "source '${SETUP_FILE}' && export RCUTILS_LOGGING_BUFFERED_STREAM=1 && exec ${cmd}" \
    >"${log_file}" 2>&1 &
  local pid=$!
  if [[ -n "${PID_FILE}" ]]; then
    echo "${pid}" >> "${PID_FILE}"
  fi
}

is_first_tab=true

for i in "${!tabs[@]}"; do
  title="${tabs[$i]%%|*}"
  cmd="${tabs[$i]#*|}"

  if [[ "${HEADLESS_MODE}" = "1" ]]; then
    launch_headless "${title}" "${cmd}"
    sleep 0.5
    continue
  fi

  echo "Launching [${title}] ${cmd}"

  if [[ "${is_first_tab}" = true ]]; then
    gnome-terminal --window --title="${title}" --working-directory="${SCRIPT_DIR}" -- \
      bash -c "source ${SETUP_FILE} && export RCUTILS_LOGGING_BUFFERED_STREAM=1 && ${cmd}; exec bash"
    is_first_tab=false
    sleep 0.5
  else
    gnome-terminal --tab --title="${title}" --working-directory="${SCRIPT_DIR}" -- \
      bash -c "source ${SETUP_FILE} && export RCUTILS_LOGGING_BUFFERED_STREAM=1 && ${cmd}; exec bash"
  fi
done

if [[ "${HEADLESS_MODE}" = "1" ]]; then
  echo "Headless real pre-stack launched."
  exit 0
fi

echo "Opened real pre-stack tabs. Suggested checks:"
echo "  ros2 topic echo /odom --num 1"
echo "  ros2 topic echo /segmentation/obstacle --num 1"
echo "  ros2 topic echo /tracked_obstacles --num 1"
