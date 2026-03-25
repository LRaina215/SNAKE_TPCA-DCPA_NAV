#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_FILE="${SCRIPT_DIR}/../install/setup.bash"

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "Missing workspace setup: ${SETUP_FILE}"
  exit 1
fi

cmds=(
  "ros2 launch point_lio mapping_mid360.launch.py use_sim_time:=True rviz:=false"
  "ros2 launch terrain_analysis terrain_analysis.launch use_sim_time:=True"
  "ros2 launch predictive_tracker dynamic_tracker.launch.py use_sim_time:=True"
  "ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py use_sim_time:=True"
)

for cmd in "${cmds[@]}"; do
  echo "Current CMD : ${cmd}"
  gnome-terminal -- bash -lc "cd '${SCRIPT_DIR}'; source '${SETUP_FILE}'; ${cmd}; exec bash;"
  sleep 0.3
done
