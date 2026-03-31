#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WORKSPACE_DIR}/install/setup.bash"
MAP_FILE="${SCRIPT_DIR}/rm_navi/rm_navigation/navi/maps/arena_map.yaml"
RVIZ_FILE="${SCRIPT_DIR}/rm_navi/rm_navigation/navi/rviz/nav2_default_view.rviz"

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "Missing workspace setup: ${SETUP_FILE}"
  exit 1
fi

if [[ ! -f "${MAP_FILE}" ]]; then
  echo "Missing map yaml: ${MAP_FILE}"
  exit 1
fi

if [[ ! -f "${RVIZ_FILE}" ]]; then
  echo "Missing RViz config: ${RVIZ_FILE}"
  exit 1
fi

set +u
source "${SETUP_FILE}"
set -u
export RCUTILS_LOGGING_BUFFERED_STREAM=1
HEADLESS_MODE="${AUTO_EVAL_HEADLESS:-0}"
SKIP_RVIZ="${AUTO_EVAL_SKIP_RVIZ:-0}"
PID_FILE="${AUTO_EVAL_PID_FILE:-}"
LOG_DIR="${AUTO_EVAL_LOG_DIR:-${SCRIPT_DIR}/.auto_eval_logs/sim_nav_dwb_baseline}"

tabs=(
  "MapTF|ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map odom"
  "Baseline|ros2 launch navi bringup_dwb_baseline_launch.py use_sim_time:=True map:=${MAP_FILE}"
)

if [[ "${SKIP_RVIZ}" != "1" ]]; then
  tabs+=("RVIZ|ros2 run rviz2 rviz2 -d ${RVIZ_FILE}")
fi

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

  if [ "${is_first_tab}" = true ]; then
    gnome-terminal --window --title="${title}" --working-directory="${SCRIPT_DIR}" -- bash -c "source ${SETUP_FILE} && export RCUTILS_LOGGING_BUFFERED_STREAM=1 && ${cmd}; exec bash"
    is_first_tab=false
    sleep 0.5
  else
    gnome-terminal --tab --title="${title}" --working-directory="${SCRIPT_DIR}" -- bash -c "source ${SETUP_FILE} && export RCUTILS_LOGGING_BUFFERED_STREAM=1 && ${cmd}; exec bash"
  fi
done

if [[ "${HEADLESS_MODE}" = "1" ]]; then
  echo "Headless DWB baseline navigation stack launched."
  exit 0
fi

echo "Opened DWB baseline navigation tabs."
