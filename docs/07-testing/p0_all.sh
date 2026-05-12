#!/usr/bin/env bash
# p0_all.sh - run core P0 scripts in sequence. Each one logs to its own file;
# this script collates a summary. Exploration is opt-in because it requires the
# explore or tare_explore profile and a safe open area.

set -e
cd "$(dirname "$0")"

STAMP=$(date +%Y%m%d_%H%M%S)
MAP_NAME="${LINGTU_P0_MAP_NAME:-p0_${STAMP}}"
GOAL_X="${LINGTU_P0_GOAL_X:-${1:-}}"
GOAL_Y="${LINGTU_P0_GOAL_Y:-${2:-}}"
GOTO_TIMEOUT="${LINGTU_P0_GOTO_TIMEOUT:-${3:-60}}"
if [[ -z "$GOAL_X" || -z "$GOAL_Y" ]]; then
  echo "FAIL: set LINGTU_P0_GOAL_X and LINGTU_P0_GOAL_Y, or pass GOAL_X GOAL_Y [TIMEOUT]."
  echo "      The aggregate P0 flow will not choose a motion target automatically."
  exit 2
fi

LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
SUMMARY="$LOG_DIR/${STAMP}_p0_all_summary.log"
exec > >(tee -a "$SUMMARY") 2>&1

echo "=== P0 ALL - $(date) ==="
echo "map=$MAP_NAME goal=($GOAL_X, $GOAL_Y) goto_timeout=${GOTO_TIMEOUT}s"

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
    local code=$?
    echo "[SUMMARY] $name  FAIL (exit $code)"
    return "$code"
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

json_payload() {
  python3 - "$@" <<'PY'
import json
import sys

items = {}
for arg in sys.argv[1:]:
    key, value = arg.split("=", 1)
    items[key] = value
json.dump(items, sys.stdout)
PY
}

session_end() {
  curl -sf -X POST http://localhost:5050/api/v1/session/end >/dev/null 2>&1 || true
}

session_start() {
  local mode="$1"
  local slam_profile="$2"
  local map_name="${3:-}"
  local payload
  if [[ -n "$map_name" ]]; then
    payload="$(json_payload "mode=$mode" "slam_profile=$slam_profile" "map_name=$map_name")"
  else
    payload="$(json_payload "mode=$mode" "slam_profile=$slam_profile")"
  fi
  echo "Starting Gateway session: $payload"
  curl -sf -X POST http://localhost:5050/api/v1/session/start \
    -H 'Content-Type: application/json' \
    -d "$payload" | python3 -m json.tool
}

confirm_after_preview() {
  echo ""
  echo "Route preview passed for goal=($GOAL_X, $GOAL_Y)."
  echo "Confirm the displayed path is safe in the real test area before motion."
  echo "Type RUN to send the goal through /api/v1/goal:"
  read -r answer
  if [[ "$answer" != "RUN" ]]; then
    echo "FAIL: operator did not confirm motion goal"
    exit 3
  fi
}

run_one "P0-01 cold boot"   p0_cold_boot.sh
session_end
session_start "mapping" "fastlio2"
run_one "P0-02 mapping"     p0_mapping.sh "$MAP_NAME"
session_end
session_start "navigating" "localizer" "$MAP_NAME"
run_one "P0-03 route safety" p0_route_safety.sh "$GOAL_X" "$GOAL_Y"
confirm_after_preview
run_one "P0-04 goto"        p0_goto.sh "$GOAL_X" "$GOAL_Y" "$GOTO_TIMEOUT"
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
