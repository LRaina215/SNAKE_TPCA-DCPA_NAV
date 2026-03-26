#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_FILE="${SCRIPT_DIR}/../install/setup.bash"

# [新增这行] 将当前脚本的 Linux 路径转换为 Windows 路径，赋给 WIN_PWD
WIN_PWD=$(wslpath -w "$SCRIPT_DIR")

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "Missing workspace setup: ${SETUP_FILE}"
  exit 1
fi

cmds=(
  "ros2 launch icp_registration icp.launch.py use_sim_time:=True"
  "ros2 launch navi localization_launch.py use_sim_time:=True"
  "ros2 launch navi navigation_launch.py use_sim_time:=True"
)

for cmd in "${cmds[@]}"; do
  echo "Current CMD : ${cmd}"
  # gnome-terminal -- bash -lc "cd '${SCRIPT_DIR}'; source '${SETUP_FILE}'; ${cmd}; exec bash;"
  wt.exe -w 0 nt -d "$WIN_PWD" wsl.exe -d Ubuntu-20.04 -- bash -c "source ../install/setup.bash && $cmd \\; exec bash"
  sleep 0.4
done
