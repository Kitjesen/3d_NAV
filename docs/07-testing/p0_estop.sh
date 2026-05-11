#!/usr/bin/env bash
# P0-05: emergency stop reflex.
#
# Verifies the Gateway stop command is accepted and that the robot-reported
# navigation speed settles to zero after the stop. This script uses current
# Gateway contracts only: POST /api/v1/stop and GET /api/v1/state.

set -e

LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_estop.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-04 E-stop reflex - $(date) ==="

echo "[1/4] Baseline health"
curl -sf http://localhost:5050/api/v1/health > /dev/null || {
  echo "FAIL: Gateway down"
  exit 2
}

echo "[2/4] Please manually drive the robot so reported speed is non-zero."
echo "      Press [Enter] when ready to issue Gateway stop."
read -r

echo "[3/4] Sending stop command and measuring RPC latency ..."
T0=$(date +%s%3N)
curl -sf -X POST http://localhost:5050/api/v1/stop \
  -H 'Content-Type: application/json' \
  -d '{"request_id":"p0-estop","client_id":"p0_estop"}' \
  | python3 -m json.tool
T1=$(date +%s%3N)
RTT=$((T1 - T0))
echo "  stop RPC round-trip: ${RTT} ms"

echo "[4/4] Waiting for navigation speed -> zero (deadline 2s) ..."
ZERO_TICKS=0
DEADLINE=$((SECONDS + 2))
LAST_SPEED="?"
while [[ $SECONDS -lt $DEADLINE ]]; do
  SPEED="$(curl -sf http://localhost:5050/api/v1/state 2>/dev/null | python3 -c '
import json, sys
d = json.load(sys.stdin)
motion = d.get("navigation", {}).get("motion", {})
value = motion.get("current_speed_mps", 1.0)
print(0.0 if value is None else value)
' 2>/dev/null || echo "1.0")"
  LAST_SPEED="$SPEED"
  if [[ "$(python3 -c "print(abs(float('$SPEED')) < 0.01)")" == "True" ]]; then
    ZERO_TICKS=$((ZERO_TICKS + 1))
    [[ "$ZERO_TICKS" -ge 3 ]] && break
  else
    ZERO_TICKS=0
  fi
  sleep 0.1
done

if [[ "$ZERO_TICKS" -lt 3 ]]; then
  echo "FAIL: reported navigation speed did not reach zero within 2s (speed=$LAST_SPEED)"
  exit 3
fi

echo ""
echo "=== PASS - stop accepted (rpc=${RTT}ms), reported navigation speed zeroed ==="
echo "Log: $LOG"
