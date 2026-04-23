#!/usr/bin/env bash
# mapping_status.sh — 远程 SSH 一屏看建图进度.
#
# 用法:
#   ssh sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/mapping_status.sh"
#
#   或者持续监控:
#   ssh sunrise@192.168.66.190 "watch -n 2 bash ~/data/SLAM/navigation/scripts/mapping_status.sh"
#
# 输出:
#   [1] Session        mode / active_map / 是否可开始建图
#   [2] SLAM 信号      slam_hz / PGO 关键帧数 / 累积点云数
#   [3] 最新地图目录   文件 / 大小 / 修改时间
#   [4] 动态过滤日志    dynamic_filter 最近 5 条日志
set -u
GW=${GW:-http://localhost:5050}
MAP_DIR=${MAP_DIR:-$HOME/data/nova/maps}

# Colors (stripped when piping to watch)
if [ -t 1 ]; then
    BOLD='\033[1m'; GRN='\033[32m'; YEL='\033[33m'; RED='\033[31m'; DIM='\033[2m'; RST='\033[0m'
else
    BOLD=''; GRN=''; YEL=''; RED=''; DIM=''; RST=''
fi

echo -e "${BOLD}=== Lingtu mapping status @ $(date '+%H:%M:%S') ===${RST}"

# [1] Session
SESS=$(curl -s --max-time 3 "$GW/api/v1/session" 2>/dev/null || echo '{}')
MODE=$(echo "$SESS" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("mode","?"))' 2>/dev/null)
MAP=$(echo "$SESS" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("active_map") or "-")' 2>/dev/null)
CAN_M=$(echo "$SESS" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("can_start_mapping",False))' 2>/dev/null)

case "$MODE" in
    mapping)    MODE_C="${GRN}mapping${RST}" ;;
    exploring)  MODE_C="${GRN}exploring${RST}" ;;
    navigating) MODE_C="${YEL}navigating${RST}" ;;
    idle)       MODE_C="${DIM}idle${RST}" ;;
    *)          MODE_C="${RED}$MODE${RST}" ;;
esac

echo -e "${BOLD}[1] Session${RST}  mode=$MODE_C  active_map=$MAP  can_start_mapping=$CAN_M"

# [2] SLAM signal
H=$(curl -s --max-time 3 "$GW/api/v1/health" 2>/dev/null || echo '{}')
HZ=$(echo "$H" | python3 -c 'import sys,json;d=json.load(sys.stdin);print(d["sensors"]["slam"].get("hz","?"))' 2>/dev/null)
CAM_FPS=$(echo "$H" | python3 -c 'import sys,json;d=json.load(sys.stdin);print(d["sensors"]["camera"].get("fps","?"))' 2>/dev/null)
PTS=$(echo "$H" | python3 -c 'import sys,json;print(json.load(sys.stdin).get("map_points",0))' 2>/dev/null)

if awk "BEGIN{exit !($HZ < 5)}"  2>/dev/null; then HZ_C="${RED}${HZ}${RST}"
elif awk "BEGIN{exit !($HZ < 20)}" 2>/dev/null; then HZ_C="${YEL}${HZ}${RST}"
else HZ_C="${GRN}${HZ}${RST}"; fi

echo -e "${BOLD}[2] SLAM${RST}     slam_hz=${HZ_C}Hz  camera_fps=$CAM_FPS  live_points=$PTS"

# [3] Latest map dir (most recently modified)
LATEST=$(ls -dt "$MAP_DIR"/*/ 2>/dev/null | head -1)
if [ -n "$LATEST" ]; then
    NAME=$(basename "$LATEST")
    MTIME=$(stat -c '%y' "$LATEST" 2>/dev/null | cut -d. -f1)
    PCD_SIZE="-"
    PCD_BAK_SIZE="-"
    PATCHES="-"
    [ -f "$LATEST/map.pcd" ] && PCD_SIZE=$(du -h "$LATEST/map.pcd" | cut -f1)
    [ -f "$LATEST/map.pcd.predufo" ] && PCD_BAK_SIZE=$(du -h "$LATEST/map.pcd.predufo" | cut -f1)
    [ -d "$LATEST/patches" ] && PATCHES=$(ls "$LATEST/patches" 2>/dev/null | wc -l)
    echo -e "${BOLD}[3] Latest${RST}   $NAME  ($MTIME)"
    echo -e "             map.pcd=$PCD_SIZE  map.pcd.predufo=$PCD_BAK_SIZE  patches=$PATCHES"
    if [ "$PCD_SIZE" != "-" ] && [ "$PCD_BAK_SIZE" != "-" ]; then
        # Point count = file size / 16 (PCD header + 16B/point xyzi)
        CLEAN_PTS=$(stat -c %s "$LATEST/map.pcd" 2>/dev/null | awk '{printf "%d", $1/16}')
        DIRTY_PTS=$(stat -c %s "$LATEST/map.pcd.predufo" 2>/dev/null | awk '{printf "%d", $1/16}')
        DROPPED=$((DIRTY_PTS - CLEAN_PTS))
        if [ "$DIRTY_PTS" -gt 0 ]; then
            PCT=$(awk -v d=$DROPPED -v t=$DIRTY_PTS 'BEGIN{printf "%.2f", 100*d/t}')
            echo -e "             ${BOLD}DUFOMap dropped $DROPPED pts ($PCT%)${RST}"
        fi
    fi
else
    echo -e "${BOLD}[3] Latest${RST}   ${DIM}(no maps yet in $MAP_DIR)${RST}"
fi

# [4] Recent dynamic_filter log
echo -e "${BOLD}[4] Log${RST}      (last 5 dynamic_filter / dufomap lines)"
journalctl -u lingtu.service --since '10 min ago' --no-pager 2>/dev/null \
    | grep -iE 'dynamic_filter|dufomap' \
    | tail -5 \
    | sed "s/^/             /"
