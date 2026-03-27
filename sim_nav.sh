#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WORKSPACE_DIR}/install/setup.bash"
MAP_FILE="${SCRIPT_DIR}/rm_navi/rm_navigation/navi/maps/arena_map.yaml"

WIN_PWD=$(wslpath -w "$SCRIPT_DIR")

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
)

wt_args=(-w 0)

for i in "${!tabs[@]}"; do
  title="${tabs[$i]%%|*}"
  cmd="${tabs[$i]#*|}"

  echo "Launching [${title}] ${cmd}"

  if [[ "$i" -gt 0 ]]; then
    wt_args+=(\;)
  fi

  wt_args+=(
    nt
    --title "$title"
    -d "$WIN_PWD"
    wsl.exe -d Ubuntu-20.04 -- bash -c
    "source ../install/setup.bash && export RCUTILS_LOGGING_BUFFERED_STREAM=1 && ${cmd} \\; exec bash"
  )
done

wt.exe "${wt_args[@]}"

echo "Opened all navigation tabs. Suggested checks:"
echo "  ros2 run tf2_ros tf2_echo map odom"
echo "  ros2 topic echo /scan --num 1"
echo "  ros2 node list"
