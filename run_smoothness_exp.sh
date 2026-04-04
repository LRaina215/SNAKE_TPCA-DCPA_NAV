#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODE="${1:-full}"
PRE_WAIT_S="${SMOOTHNESS_PRE_WAIT_S:-8}"
NAV_WAIT_S="${SMOOTHNESS_NAV_WAIT_S:-8}"

case "${MODE}" in
  baseline)
    NAV_SCRIPT="${SCRIPT_DIR}/sim_nav_dwb_baseline.sh"
    BAG_PREFIX="baseline"
    ;;
  full)
    NAV_SCRIPT="${SCRIPT_DIR}/sim_nav.sh"
    BAG_PREFIX="full"
    ;;
  riskonly)
    NAV_SCRIPT="${SCRIPT_DIR}/sim_nav_dwb_risk_only.sh"
    BAG_PREFIX="riskonly"
    ;;
  teb)
    NAV_SCRIPT="${SCRIPT_DIR}/sim_nav_teb.sh"
    BAG_PREFIX="teb"
    ;;
  *)
    echo "Usage: $0 [baseline|full|riskonly|teb]"
    exit 1
    ;;
esac

echo "====================================================="
echo " Smoothness experiment launcher"
echo "====================================================="
echo "Mode           : ${MODE}"
echo "Pre wait (s)   : ${PRE_WAIT_S}"
echo "Nav wait (s)   : ${NAV_WAIT_S}"
echo ""
echo "This script will:"
echo "  1. Launch the simulation front-end stack"
echo "  2. Launch the selected navigation stack"
echo "  3. Start rosbag recording for smoothness plots"
echo "====================================================="

"${SCRIPT_DIR}/sim_pre.sh"
echo "Waiting ${PRE_WAIT_S}s for the simulation stack to stabilize..."
sleep "${PRE_WAIT_S}"

"${NAV_SCRIPT}"
echo "Waiting ${NAV_WAIT_S}s for the navigation stack to stabilize..."
sleep "${NAV_WAIT_S}"

echo ""
echo "Stacks are up. Starting rosbag recording now..."
echo "Use RViz to send a single goal after recording begins."
echo ""

exec "${SCRIPT_DIR}/record_smoothness_bag.sh" "${BAG_PREFIX}"
