#!/usr/bin/env bash
# P0-01: S100P 冷启动 — 20+ modules OK, Gateway stable 3 minutes.
#
# Run on sunrise (not local dev):
#   ssh sunrise@192.168.66.190
#   cd ~/data/SLAM/navigation
#   bash docs/07-testing/p0_cold_boot.sh

set -e

LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_cold_boot.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-01 Cold Boot — $(date) ==="

# 1. Ensure service is down first, then launch fresh
echo "[1/5] Stopping any running LingTu ..."
sudo systemctl stop lingtu 2>/dev/null || true
sleep 3

# 2. Start LingTu via systemd (production path)
echo "[2/5] Starting LingTu service ..."
sudo systemctl start lingtu
sleep 10

# 3. Poll Gateway health (max 60s)
echo "[3/5] Polling Gateway /api/v1/health ..."
DEADLINE=$((SECONDS + 60))
while ! curl -sf http://localhost:5050/api/v1/health > /dev/null; do
  if [[ $SECONDS -gt $DEADLINE ]]; then
    echo "FAIL: Gateway not responsive after 60s"
    exit 2
  fi
  sleep 2
done

# 4. Assert ≥20 modules ok
HEALTH_JSON="$(curl -s http://localhost:5050/api/v1/health)"
MODULES_OK="$(echo "$HEALTH_JSON" | python3 -c 'import json,sys; print(json.load(sys.stdin)["modules_ok"])')"
MODULES_FAIL="$(echo "$HEALTH_JSON" | python3 -c 'import json,sys; print(json.load(sys.stdin)["modules_fail"])')"
echo "[4/5] modules_ok=$MODULES_OK  modules_fail=$MODULES_FAIL"
if [[ "$MODULES_OK" -lt 20 ]]; then
  echo "FAIL: only $MODULES_OK modules up (need ≥20)"
  exit 3
fi
if [[ "$MODULES_FAIL" -gt 0 ]]; then
  echo "FAIL: $MODULES_FAIL modules failed"
  echo "$HEALTH_JSON" | python3 -m json.tool
  exit 4
fi

# 5. 3-minute stability watch — no module must flip to fail
echo "[5/5] 3-minute stability watch (polling every 15s) ..."
for i in 1 2 3 4 5 6 7 8 9 10 11 12; do
  sleep 15
  F=$(curl -s http://localhost:5050/api/v1/health | python3 -c 'import json,sys; print(json.load(sys.stdin)["modules_fail"])' 2>/dev/null || echo "?")
  echo "  tick $i/12  modules_fail=$F"
  if [[ "$F" != "0" ]]; then
    echo "FAIL: module failure detected at tick $i"
    exit 5
  fi
done

echo ""
echo "=== PASS — LingTu cold boot stable 3 minutes, $MODULES_OK modules ==="
echo "Log: $LOG"
