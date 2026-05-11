#!/usr/bin/env bash
# p0_all.sh - run core P0 scripts in sequence. Each one logs to its own file;
# this script collates a summary. Exploration is opt-in because it requires the
# explore or tare_explore profile and a safe open area.

set -e
cd "$(dirname "$0")"

STAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
SUMMARY="$LOG_DIR/${STAMP}_p0_all_summary.log"
exec > >(tee -a "$SUMMARY") 2>&1

echo "=== P0 ALL - $(date) ==="

ran=0
passed=0

run_one() {
  local name="$1"
  local script="$2"
  shift 2
  ran=$((ran + 1))
  echo ""
  echo "------------ $name ------------"
  if bash "$script" "$@"; then
    echo "[SUMMARY] $name  PASS"
    passed=$((passed + 1))
  else
    echo "[SUMMARY] $name  FAIL (exit $?)"
  fi
}

pause_for_profile() {
  local name="$1"
  local instruction="$2"
  echo ""
  echo "------------ profile switch: $name ------------"
  echo "$instruction"
  echo "Press [Enter] after the profile is running and Gateway is responsive."
  read -r
}

run_one "P0-01 cold boot"   p0_cold_boot.sh
run_one "P0-02 mapping"     p0_mapping.sh
pause_for_profile "navigation/localizer" \
  "Switch LingTu to the nav/localizer profile using the saved active map."
run_one "P0-03 route safety" p0_route_safety.sh 2.0 0.0
run_one "P0-04 goto"        p0_goto.sh 2.0 0.0 60
run_one "P0-05 estop"       p0_estop.sh
if [[ "${LINGTU_P0_RUN_EXPLORE:-0}" == "1" ]]; then
  pause_for_profile "explore/tare_explore" \
    "Switch LingTu to the explore or tare_explore profile in a safe open area."
  run_one "P0-06 explore"   p0_explore.sh "${LINGTU_P0_EXPLORE_DURATION:-30}"
else
  echo ""
  echo "[SUMMARY] P0-06 explore SKIP (set LINGTU_P0_RUN_EXPLORE=1 in a safe explore/tare_explore session)"
fi

echo ""
echo "=== P0 ALL COMPLETE: ${passed}/${ran} passed ==="
echo "Summary log: $SUMMARY"
[[ "$passed" -eq "$ran" ]] || exit 1
