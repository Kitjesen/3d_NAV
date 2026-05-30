#!/usr/bin/env bash
# P0-02: map -> save -> activate -> localize.
#
# Interactive: requires a human walking the robot around for ~3 minutes.
# Pre-condition: LingTu already running (map profile or fastlio2 profile).
#
# Post-condition: a new map saved under the Gateway map root:
#   $NAV_MAP_DIR, then the Gateway default ~/data/nova/maps.
#   containing map.pcd + tomogram.pickle + occupancy.npz + map.pgm + map.yaml
#   and the active symlink points to it; localizer profile can load it.

set -e

resolve_map_root() {
  if [[ -n "${NAV_MAP_DIR:-}" ]]; then
    echo "$NAV_MAP_DIR"
    return
  fi
  echo "${MAP_DIR:-$HOME/data/nova/maps}"
}

MAP_NAME="${1:-p0_$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date +%Y%m%d_%H%M%S)_p0_mapping.log"
exec > >(tee -a "$LOG") 2>&1

echo "=== P0-02 Mapping - $(date) - map=$MAP_NAME ==="

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

# 1. Sanity: is SLAM running?
echo "[1/6] Sanity check - Gateway + SLAM responsive ..."
SLAM_HZ="$(curl -sf http://localhost:5050/api/v1/health | python3 -c 'import json,sys; print(json.load(sys.stdin).get("slam_hz", 0))')"
if [[ "$(python3 -c "print($SLAM_HZ > 0.5)")" != "True" ]]; then
  echo "FAIL: SLAM rate $SLAM_HZ Hz - is fastlio2 profile running?"
  exit 2
fi
echo "  SLAM Hz = $SLAM_HZ"

# 2. Prompt human to walk the robot
echo ""
echo "[2/6] Please walk the robot around the target area for ~3 minutes."
echo "      Press [Enter] when mapping is complete ..."
read -r

# 3. Trigger save via MapManager action
echo "[3/6] Triggering MapManager save pipeline: $MAP_NAME"
SAVE_RESP="$(curl -sf -X POST http://localhost:5050/api/v1/maps \
  -H 'Content-Type: application/json' \
  -d "$(json_payload action=save "name=$MAP_NAME")")"
echo "$SAVE_RESP" | python3 -m json.tool

SUCCESS="$(echo "$SAVE_RESP" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("success", False))')"
if [[ "$SUCCESS" != "True" ]]; then
  echo "FAIL: save failed"
  exit 3
fi
SAVE_PATH="$(echo "$SAVE_RESP" | python3 -c '
import json, sys
d = json.load(sys.stdin)
print(d.get("map_dir") or d.get("path") or "")
')"

# 4. Verify artifacts
if [[ -n "$SAVE_PATH" ]]; then
  MAP_DIR="$SAVE_PATH"
  MAP_ROOT="$(dirname "$SAVE_PATH")"
else
  MAP_ROOT="$(resolve_map_root)"
  MAP_DIR="$MAP_ROOT/$MAP_NAME"
fi
echo "[4/6] Verifying artifacts at $MAP_DIR"
for f in map.pcd tomogram.pickle occupancy.npz map.pgm map.yaml; do
  if [[ -f "$MAP_DIR/$f" ]]; then
    SZ=$(stat -c%s "$MAP_DIR/$f")
    echo "  OK $f  ($SZ bytes)"
  else
    echo "  MISSING $f"
    exit 4
  fi
done

# 5. Activate
echo "[5/6] Activating map"
curl -sf -X POST http://localhost:5050/api/v1/maps \
  -H 'Content-Type: application/json' \
  -d "$(json_payload action=set_active "name=$MAP_NAME")" | python3 -m json.tool

ACTIVE_LINK="$MAP_ROOT/active"
ACTIVE_TARGET="$(readlink "$ACTIVE_LINK" || true)"
if [[ "$(basename "$ACTIVE_TARGET")" != "$MAP_NAME" ]]; then
  echo "FAIL: active symlink not pointing at $MAP_NAME"
  exit 5
fi

# 6. Done - localizer profile switch is manual (reboot systemd unit)
echo ""
echo "=== PASS - map '$MAP_NAME' saved + activated ==="
echo "Next:"
echo "  POST /api/v1/session/start with mode=navigating, slam_profile=localizer, map_name=$MAP_NAME"
echo "  or run p0_all.sh, which starts that Gateway session before route checks."
echo ""
echo "Log: $LOG"
