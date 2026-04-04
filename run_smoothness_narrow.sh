#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODE="${1:-full}"
PRE_WAIT_S="${SMOOTHNESS_PRE_WAIT_S:-8}"
NAV_WAIT_S="${SMOOTHNESS_NAV_WAIT_S:-8}"

case "${MODE}" in
  baseline)
    NAV_SCRIPT="${SCRIPT_DIR}/sim_nav_dwb_baseline_narrow.sh"
    BAG_PREFIX="baseline_narrow"
    ;;
  full)
    NAV_SCRIPT="${SCRIPT_DIR}/sim_nav_narrow.sh"
    BAG_PREFIX="full_narrow"
    ;;
  *)
    echo "Usage: $0 [baseline|full]"
    exit 1
    ;;
esac

echo "====================================================="
echo " Narrow-corridor smoothness launcher"
echo "====================================================="
echo "Mode           : ${MODE}"
echo "Pre wait (s)   : ${PRE_WAIT_S}"
echo "Nav wait (s)   : ${NAV_WAIT_S}"
echo ""
echo "Recommended protocol:"
echo "  1. Run once for baseline and once for full"
echo "  2. In RViz, send the same goal near (4.0, 0.0)"
echo "  3. Record only one representative interaction per bag"
echo "====================================================="

"${SCRIPT_DIR}/sim_pre_narrow.sh"
echo "Waiting ${PRE_WAIT_S}s for the narrow pre-stack to stabilize..."
sleep "${PRE_WAIT_S}"

"${NAV_SCRIPT}"
echo "Waiting ${NAV_WAIT_S}s for the navigation stack to stabilize..."
sleep "${NAV_WAIT_S}"

echo ""
echo "Stacks are up. Starting rosbag recording now..."
echo "Use RViz to send a single narrow-corridor goal after recording begins."
echo ""

exec "${SCRIPT_DIR}/record_smoothness_bag.sh" "${BAG_PREFIX}"
