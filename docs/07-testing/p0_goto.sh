#!/usr/bin/env bash
# P0-04: point goal -> arrival.
#
# Pre-condition: localizer profile running + active map already loaded + robot
# facing roughly along +X. Goal is sent through /api/v1/goal and the script
# waits for /api/v1/navigation/status to report SUCCESS.

set -e

GOAL_X="${1:-2.0}"
GOAL_Y="${2:-0.0}"
TIMEOUT="${3:-60}"

LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_goto.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-03 Goto - $(date) - goal=($GOAL_X, $GOAL_Y) timeout=${TIMEOUT}s ==="

echo "[1/4] Sanity"
HEALTH="$(curl -sf http://localhost:5050/api/v1/health)"
HAS_ODOM="$(echo "$HEALTH" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("has_odom", False))')"
if [[ "$HAS_ODOM" != "True" ]]; then
  echo "FAIL: no odometry - is localizer running?"
  exit 2
fi

echo "[2/4] Sending goal"
curl -sf -X POST http://localhost:5050/api/v1/goal \
  -H 'Content-Type: application/json' \
  -d "{\"x\":$GOAL_X,\"y\":$GOAL_Y,\"z\":0.0,\"frame_id\":\"map\",\"client_id\":\"p0_goto\",\"request_id\":\"p0-goto-$(date +%s)\"}" \
  | python3 -m json.tool

echo "[3/4] Polling mission state (timeout ${TIMEOUT}s) ..."
DEADLINE=$((SECONDS + TIMEOUT))
LAST_STATE="?"
while [[ $SECONDS -lt $DEADLINE ]]; do
  STATUS_JSON="$(curl -sf http://localhost:5050/api/v1/navigation/status 2>/dev/null || echo '{}')"
  STATUS="$(echo "$STATUS_JSON" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("state","?"))')"
  if [[ "$STATUS" != "$LAST_STATE" ]]; then
    echo "  nav state: $LAST_STATE -> $STATUS"
    LAST_STATE="$STATUS"
  fi
  case "$STATUS" in
    SUCCESS)    echo ""; echo "=== PASS - reached goal in $SECONDS s ==="; exit 0 ;;
    FAILED)     echo ""; echo "FAIL - navigation FAILED state"; exit 3 ;;
    CANCELLED)  echo ""; echo "FAIL - cancelled"; exit 4 ;;
  esac
  sleep 2
done

echo ""
echo "FAIL - timeout after ${TIMEOUT}s, last state = $LAST_STATE"
exit 5
