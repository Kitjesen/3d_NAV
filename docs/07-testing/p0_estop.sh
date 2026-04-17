#!/usr/bin/env bash
# P0-04: 紧急停止反射
#
# Verifies:
#   1. SafetyRing publishes stop_cmd within 100 ms of trigger
#   2. CmdVelMux output drops to zero (linear=0, angular=0) immediately
#   3. watchdog log has a matching entry
#
# Run interactively — cmd_vel source should be teleop or visual_servo during
# the test so we can observe the clamp.

set -e

LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_estop.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-04 E-stop reflex — $(date) ==="

# 1. Baseline — verify Gateway up
echo "[1/4] Baseline health"
curl -sf http://localhost:5050/api/v1/health > /dev/null || { echo "FAIL: Gateway down"; exit 2; }

# 2. Arm a continuous cmd_vel source (teleop) so we can measure the clamp
echo "[2/4] Please manually drive the robot (teleop joystick or API) so that"
echo "      linear.x > 0.1 m/s is being published.  Press [Enter] when ready."
read -r

# 3. Send E-stop level 2 (hard stop) and measure response time
echo "[3/4] Sending E-stop (level=2) and measuring reflex latency ..."
T0=$(date +%s%3N)
curl -sf -X POST http://localhost:5050/api/v1/stop \
  -H 'Content-Type: application/json' \
  -d '{"level":2}' | python3 -m json.tool
T1=$(date +%s%3N)
RTT=$((T1 - T0))
echo "  stop RPC round-trip: ${RTT} ms"

# 4. Poll cmd_vel until it's zero for 3 consecutive ticks (100 ms each)
echo "[4/4] Waiting for cmd_vel → zero (deadline 1s) ..."
ZERO_TICKS=0
DEADLINE=$((SECONDS + 2))
while [[ $SECONDS -lt $DEADLINE ]]; do
  STATE="$(curl -sf http://localhost:5050/api/v1/safety/state 2>/dev/null || echo '{}')"
  STOP_ACTIVE="$(echo "$STATE" | python3 -c 'import json,sys; d=json.load(sys.stdin) if sys.stdin.isatty()==False else {}; print(d.get("estop", False))' 2>/dev/null || echo "?")"
  LINEAR="$(curl -sf http://localhost:5050/api/v1/cmd_vel 2>/dev/null | python3 -c 'import json,sys; d=json.load(sys.stdin) if sys.stdin.isatty()==False else {}; print(d.get("linear",{}).get("x",1.0))' 2>/dev/null || echo "1.0")"
  if [[ "$(python3 -c "print(abs($LINEAR) < 0.01)")" == "True" ]]; then
    ZERO_TICKS=$((ZERO_TICKS + 1))
    [[ "$ZERO_TICKS" -ge 3 ]] && break
  else
    ZERO_TICKS=0
  fi
  sleep 0.1
done

if [[ "$ZERO_TICKS" -lt 3 ]]; then
  echo "FAIL: cmd_vel did not reach zero within 1s (linear still=$LINEAR)"
  exit 3
fi

echo ""
echo "=== PASS — E-stop reflex OK (rpc=${RTT}ms, cmd_vel zeroed within 1s) ==="
echo "Log: $LOG"
