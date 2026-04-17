#!/usr/bin/env bash
# P0-02: 建图 → 保存 → 激活 → 定位
#
# Interactive: requires a human walking the robot around for ~3 minutes.
# Pre-condition: LingTu already running (map profile or fastlio2 profile).
#
# Post-condition: a new map saved under ~/data/inovxio/data/maps/<NAME>/
#   containing map.pcd + tomogram.pickle + occupancy.npz + map.pgm + map.yaml
#   and the active symlink points to it; localizer profile can load it.

set -e

MAP_NAME="${1:-p0_$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_mapping.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-02 Mapping — $(date) — map=$MAP_NAME ==="

# 1. Sanity: is SLAM running?
echo "[1/6] Sanity check — Gateway + SLAM responsive ..."
SLAM_HZ="$(curl -sf http://localhost:5050/api/v1/health | python3 -c 'import json,sys; print(json.load(sys.stdin).get("slam_hz", 0))')"
if [[ "$(python3 -c "print($SLAM_HZ > 0.5)")" != "True" ]]; then
  echo "FAIL: SLAM rate $SLAM_HZ Hz — is fastlio2 profile running?"
  exit 2
fi
echo "  SLAM Hz = $SLAM_HZ"

# 2. Prompt human to walk the robot
echo ""
echo "[2/6] Please walk the robot around the target area for ~3 minutes."
echo "      Press [Enter] when mapping is complete ..."
read -r

# 3. Trigger save via MapManager action
echo "[3/6] Triggering map save: $MAP_NAME"
SAVE_RESP="$(curl -sf -X POST http://localhost:5050/api/v1/map/save \
  -H 'Content-Type: application/json' \
  -d "{\"name\":\"$MAP_NAME\"}")"
echo "$SAVE_RESP" | python3 -m json.tool

SUCCESS="$(echo "$SAVE_RESP" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("success", False))')"
if [[ "$SUCCESS" != "True" ]]; then
  echo "FAIL: save failed"
  exit 3
fi

# 4. Verify artifacts
MAP_DIR="${HOME}/data/inovxio/data/maps/$MAP_NAME"
echo "[4/6] Verifying artifacts at $MAP_DIR"
for f in map.pcd tomogram.pickle occupancy.npz map.pgm map.yaml; do
  if [[ -f "$MAP_DIR/$f" ]]; then
    SZ=$(stat -c%s "$MAP_DIR/$f")
    echo "  ✓ $f  ($SZ bytes)"
  else
    echo "  ✗ $f  MISSING"
    exit 4
  fi
done

# 5. Activate
echo "[5/6] Activating map"
curl -sf -X POST http://localhost:5050/api/v1/map/activate \
  -H 'Content-Type: application/json' \
  -d "{\"name\":\"$MAP_NAME\"}" | python3 -m json.tool

ACTIVE_LINK="${HOME}/data/inovxio/data/maps/active"
if [[ "$(readlink "$ACTIVE_LINK")" != *"$MAP_NAME"* ]]; then
  echo "FAIL: active symlink not pointing at $MAP_NAME"
  exit 5
fi

# 6. Done — localizer profile switch is manual (reboot systemd unit)
echo ""
echo "=== PASS — map '$MAP_NAME' saved + activated ==="
echo "Next:"
echo "  sudo systemctl stop lingtu"
echo "  sudo systemctl edit lingtu --full  # switch profile=s100p (localizer)"
echo "  sudo systemctl start lingtu"
echo "  then verify /api/v1/health reports localizer mode"
echo ""
echo "Log: $LOG"
