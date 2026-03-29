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

tabs=(
  "MapTF|ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map odom"
  "RiskOnly|ros2 launch navi bringup_dwb_risk_only_launch.py use_sim_time:=True map:=${MAP_FILE}"
  "RVIZ|ros2 run rviz2 rviz2 -d ${RVIZ_FILE}"
)

is_first_tab=true

for i in "${!tabs[@]}"; do
  title="${tabs[$i]%%|*}"
  cmd="${tabs[$i]#*|}"

  echo "Launching [${title}] ${cmd}"

  if [ "${is_first_tab}" = true ]; then
    gnome-terminal --window --title="${title}" --working-directory="${SCRIPT_DIR}" -- bash -c "source ${SETUP_FILE} && export RCUTILS_LOGGING_BUFFERED_STREAM=1 && ${cmd}; exec bash"
    is_first_tab=false
    sleep 0.5
  else
    gnome-terminal --tab --title="${title}" --working-directory="${SCRIPT_DIR}" -- bash -c "source ${SETUP_FILE} && export RCUTILS_LOGGING_BUFFERED_STREAM=1 && ${cmd}; exec bash"
  fi
done

echo "Opened DWB risk-only navigation tabs."
