#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RAW_PCD="${SCRIPT_DIR}/rm_navi/rm_localization/point_lio/PCD/scans.pcd"
MAP_PCD="${SCRIPT_DIR}/rm_navi/rm_localization/point_lio/PCD/scans_sim_map.pcd"
SHIFT_SCRIPT="${SCRIPT_DIR}/rm_navi/rm_localization/point_lio/scripts/pointlio_pcd_shift.py"

if [[ ! -f "${RAW_PCD}" ]]; then
  echo "Missing raw Point-LIO PCD: ${RAW_PCD}"
  echo "Run Point-LIO with pcd_save_en=true and exit it once to generate scans.pcd first."
  exit 1
fi

python3 "${SHIFT_SCRIPT}" \
  --input "${RAW_PCD}" \
  --output "${MAP_PCD}" \
  --tx -4.0 \
  --ty 0.0 \
  --tz 0.0 \
  --yaw-deg 0.0

echo "Generated map-aligned PCD: ${MAP_PCD}"
