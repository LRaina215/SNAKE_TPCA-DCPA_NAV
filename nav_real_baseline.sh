#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export REAL_NAV_PARAMS_FILE="${SCRIPT_DIR}/rm_navi/rm_navigation/navi/params/nav2_params_real_baseline.yaml"
exec "${SCRIPT_DIR}/nav_real.sh"
