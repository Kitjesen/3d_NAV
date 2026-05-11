#!/usr/bin/env bash
# P0-06: autonomous exploration start -> observe -> stop.
#
# Pre-condition: LingTu is running with the explore or tare_explore profile in
# a safe open test area, localization is healthy, and an operator is ready to
# stop the robot manually if needed.

set -e

DURATION="${1:-30}"

LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_explore.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-06 Explore - $(date) - duration=${DURATION}s ==="

json_field() {
  local field="$1"
  python3 -c "import json,sys; d=json.load(sys.stdin); print(d.get('$field'))"
}

stop_explore() {
  curl -sf -X POST http://localhost:5050/api/v1/explore/stop >/dev/null 2>&1 || true
}
trap stop_explore EXIT

echo "[1/5] Baseline health"
curl -sf http://localhost:5050/api/v1/health >/dev/null || {
  echo "FAIL: Gateway down"
  exit 2
}

echo "[2/5] Exploration readiness"
STATUS_JSON="$(curl -sf http://localhost:5050/api/v1/explore/status)"
echo "$STATUS_JSON" | python3 -m json.tool
AVAILABLE="$(echo "$STATUS_JSON" | json_field available)"
CAN_START="$(echo "$STATUS_JSON" | json_field can_start)"
BACKEND="$(echo "$STATUS_JSON" | json_field backend)"
if [[ "$AVAILABLE" != "True" || "$CAN_START" != "True" ]]; then
  echo "FAIL: exploration is not ready (backend=$BACKEND available=$AVAILABLE can_start=$CAN_START)"
  exit 3
fi

echo ""
echo "[3/5] Starting exploration through Gateway"
curl -sf -X POST http://localhost:5050/api/v1/explore/start | python3 -m json.tool

echo "[4/5] Observing exploration status for ${DURATION}s"
DEADLINE=$((SECONDS + DURATION))
SAW_EXPLORING=0
while [[ $SECONDS -lt $DEADLINE ]]; do
  STATUS_JSON="$(curl -sf http://localhost:5050/api/v1/explore/status)"
SUMMARY="$(echo "$STATUS_JSON" | python3 -c '
import json, sys
d = json.load(sys.stdin)
print(
    "backend={} exploring={} frontier_count={} blockers={}".format(
        d.get("backend"),
        d.get("exploring"),
        d.get("frontier_count"),
        d.get("blockers", []),
    )
)
')"
  echo "  $SUMMARY"
  EXPLORING="$(echo "$STATUS_JSON" | json_field exploring)"
  if [[ "$EXPLORING" == "True" ]]; then
    SAW_EXPLORING=1
  fi
  sleep 2
done

if [[ "$SAW_EXPLORING" != "1" ]]; then
  echo "FAIL: exploration never reported exploring=true"
  exit 4
fi

echo "[5/5] Stopping exploration"
stop_explore
trap - EXIT
FINAL_JSON="$(curl -sf http://localhost:5050/api/v1/explore/status)"
echo "$FINAL_JSON" | python3 -m json.tool
FINAL_EXPLORING="$(echo "$FINAL_JSON" | json_field exploring)"
if [[ "$FINAL_EXPLORING" != "False" ]]; then
  echo "FAIL: exploration did not stop cleanly"
  exit 5
fi

echo ""
echo "=== PASS - exploration started, reported active, and stopped cleanly ==="
echo "Log: $LOG"
