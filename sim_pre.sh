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
LOG_DIR="${AUTO_EVAL_LOG_DIR:-${SCRIPT_DIR}/.auto_eval_logs/sim_pre}"

tabs=(
  "Sim|ros2 launch tcpa_sim_env sim_launch.py gui:=$([[ \"${HEADLESS_MODE}\" = \"1\" ]] && echo false || echo true)"
  "Point-LIO|ros2 launch point_lio mapping_mid360.launch.py use_sim_time:=True rviz:=false"
  # "Segmentation|ros2 launch terrain_analysis terrain_analysis.launch use_sim_time:=True"
  "Segmentation|ros2 launch linefit_ground_segmentation_ros segmentation.launch.py"
  "Tracker|ros2 launch predictive_tracker dynamic_tracker.launch.py use_sim_time:=True"
  "Scan|ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py use_sim_time:=True target_frame:=base_link"
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
  echo "${pid}"
  if [[ -n "${PID_FILE}" ]]; then
    echo "${pid}" >> "${PID_FILE}"
  fi
}

is_first_tab=true

for i in "${!tabs[@]}"; do
  title="${tabs[$i]%%|*}"
  cmd="${tabs[$i]#*|}"

  if [[ "${HEADLESS_MODE}" = "1" ]]; then
    launch_headless "${title}" "${cmd}" >/dev/null
    sleep 0.5
    continue
  fi

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

if [[ "${HEADLESS_MODE}" = "1" ]]; then
  echo "Headless pre-simulation stack launched."
  exit 0
fi

echo "Opened all pre-simulation tabs. Suggested checks:"
echo "  ros2 topic echo /clock --num 1"
echo "  ros2 topic echo /cloud_registered --num 1"
echo "  ros2 topic echo /segmentation/obstacle --num 1"
