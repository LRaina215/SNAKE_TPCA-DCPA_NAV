#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WORKSPACE_DIR}/install/setup.bash"
DEFAULT_MAP_FILE="${SCRIPT_DIR}/rm_navi/rm_navigation/navi/maps/arena_map.yaml"
DEFAULT_PARAMS_FILE="${SCRIPT_DIR}/rm_navi/rm_navigation/navi/params/nav2_params_real_full.yaml"
DEFAULT_RVIZ_FILE="${SCRIPT_DIR}/rm_navi/rm_navigation/navi/rviz/nav2_default_view.rviz"

MAP_FILE="${REAL_MAP_FILE:-${DEFAULT_MAP_FILE}}"
PARAMS_FILE="${REAL_NAV_PARAMS_FILE:-${DEFAULT_PARAMS_FILE}}"
RVIZ_FILE="${REAL_RVIZ_FILE:-${DEFAULT_RVIZ_FILE}}"

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "Missing workspace setup: ${SETUP_FILE}"
  exit 1
fi

if [[ ! -f "${MAP_FILE}" ]]; then
  echo "Missing map yaml: ${MAP_FILE}"
  exit 1
fi

if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "Missing params yaml: ${PARAMS_FILE}"
  exit 1
fi

set +u
source "${SETUP_FILE}"
set -u
export RCUTILS_LOGGING_BUFFERED_STREAM=1

HEADLESS_MODE="${AUTO_EVAL_HEADLESS:-0}"
SKIP_RVIZ="${AUTO_EVAL_SKIP_RVIZ:-0}"
PID_FILE="${AUTO_EVAL_PID_FILE:-}"
LOG_DIR="${AUTO_EVAL_LOG_DIR:-${SCRIPT_DIR}/.auto_eval_logs/nav_real}"

tabs=(
  "ICP|ros2 launch icp_registration icp.launch.py use_sim_time:=False"
  "Localization|ros2 launch navi localization_launch.py use_sim_time:=False map:=${MAP_FILE} params_file:=${PARAMS_FILE}"
  "Nav2|ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=${PARAMS_FILE}"
)

if [[ "${SKIP_RVIZ}" != "1" && -f "${RVIZ_FILE}" ]]; then
  tabs+=("RViz|ros2 run rviz2 rviz2 -d ${RVIZ_FILE}")
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
  echo "Headless real navigation stack launched."
  exit 0
fi

echo "Opened real navigation tabs. Suggested checks:"
echo "  ros2 run tf2_ros tf2_echo map odom"
echo "  ros2 run tf2_ros tf2_echo odom base_link"
echo "  ros2 topic echo /cmd_vel --num 1"
