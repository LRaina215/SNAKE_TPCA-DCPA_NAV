#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WORKSPACE_DIR}/install/setup.bash"

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "Missing workspace setup: ${SETUP_FILE}"
  exit 1
fi

DATA_DIR="${SMOOTHNESS_BAG_DIR:-${HOME}/auto_shao/data/smoothness_bags}"
mkdir -p "${DATA_DIR}"

MODE_PREFIX="${1:-full}"
TIMESTAMP="$(date +"%Y%m%d_%H%M%S")"
BAG_NAME="${DATA_DIR}/${MODE_PREFIX}_smoothness_${TIMESTAMP}"

set +u
source "${SETUP_FILE}"
set -u

echo "====================================================="
echo " Smoothness experiment rosbag recorder"
echo "====================================================="
echo "Saving to: ${BAG_NAME}"
echo "Topics:"
echo "  /odom /cmd_vel /plan /local_plan /tracked_obstacles"
echo "  /obs1/cmd_vel /obs2/cmd_vel /tf /tf_static /clock"
echo ""
echo "After recording starts:"
echo "  1. In RViz, set the navigation goal."
echo "  2. Let the robot finish the run."
echo "  3. Press Ctrl+C here to stop recording."
echo "====================================================="

exec ros2 bag record -o "${BAG_NAME}" \
  /clock \
  /odom \
  /cmd_vel \
  /plan \
  /local_plan \
  /tracked_obstacles \
  /obs1/cmd_vel \
  /obs2/cmd_vel \
  /tf \
  /tf_static
