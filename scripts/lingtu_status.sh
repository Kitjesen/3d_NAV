#!/usr/bin/env bash
# lingtu_status.sh — 统一监控面板 (mapping + navigating 两场景通用).
#
# 用法:
#   ssh sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/lingtu_status.sh"
#
#   持续刷新 (建图/导航时开这个):
#   ssh -t sunrise@192.168.66.190 "watch -c -n 1 bash ~/data/SLAM/navigation/scripts/lingtu_status.sh"
#
# 输出分区:
#   [1] Session       当前模式 / active_map
#   [2] SLAM          slam_hz / localization 评估
#   [3] Robot         x / y / yaw / |v|
#   [4] Mission       state / waypoint 进度 / replan 次数 / degeneracy
#   [5] Path          点数 / 总长 / 距下一个 wp 剩余
#   [6] Teleop        active / clients
#   [7] Map (latest)  最近保存的地图 + DUFOMap 统计
#   [8] Log           dynamic_filter + error 日志
set -u
GW=${GW:-http://localhost:5050}
MAP_DIR=${MAP_DIR:-$HOME/data/nova/maps}

# Colors
if [ -t 1 ]; then
    B='\033[1m'; G='\033[32m'; Y='\033[33m'; R='\033[31m'; C='\033[36m'; D='\033[2m'; N='\033[0m'
else
    B=''; G=''; Y=''; R=''; C=''; D=''; N=''
fi

echo -e "${B}=== Lingtu status @ $(date '+%H:%M:%S') ===${N}"

# Fetch once to minimise latency
SESS=$(curl -s --max-time 2 "$GW/api/v1/session" 2>/dev/null || echo '{}')
HEALTH=$(curl -s --max-time 2 "$GW/api/v1/health" 2>/dev/null || echo '{}')
STATE=$(curl -s --max-time 2 "$GW/api/v1/state" 2>/dev/null || echo '{}')
PATH_JSON=$(curl -s --max-time 2 "$GW/api/v1/path" 2>/dev/null || echo '{}')

# -- Helper: Python one-liner that reads a piped JSON blob and extracts fields --
pjson() { echo "$1" | python3 -c "
import sys,json,math
try:
    d=json.load(sys.stdin)
except Exception:
    print('-'); sys.exit(0)
$2
" 2>/dev/null || echo '-'; }

# [1] Session
MODE=$(pjson "$SESS" 'print(d.get("mode","?"))')
MAP=$(pjson "$SESS" 'print(d.get("active_map") or "-")')
CAN_M=$(pjson "$SESS" 'print(d.get("can_start_mapping",False))')
case "$MODE" in
    mapping)    MC="${G}mapping${N}" ;;
    exploring)  MC="${G}exploring${N}" ;;
    navigating) MC="${C}navigating${N}" ;;
    idle)       MC="${D}idle${N}" ;;
    *)          MC="${R}$MODE${N}" ;;
esac
printf "${B}[1] Session${N}   mode=$MC  map=${C}%s${N}  can_map=%s\n" "$MAP" "$CAN_M"

# [2] SLAM
HZ=$(pjson "$HEALTH" 'print(d["sensors"]["slam"].get("hz","?"))')
LIVE_PTS=$(pjson "$HEALTH" 'print(d.get("map_points",0))')
LOC=$(pjson "$STATE" 'print(d.get("dialogue",{}).get("localization","-"))')
LOC_CAUSE=$(pjson "$STATE" 'print(d.get("dialogue",{}).get("loc_root_cause","") or "")')
if awk "BEGIN{exit !($HZ < 5)}" 2>/dev/null; then HZC="${R}${HZ}${N}"
elif awk "BEGIN{exit !($HZ < 20)}" 2>/dev/null; then HZC="${Y}${HZ}${N}"
else HZC="${G}${HZ}${N}"; fi
case "$LOC" in
    GOOD|OK) LOCC="${G}${LOC}${N}" ;;
    DEGRADED) LOCC="${Y}${LOC}${N}${D} (${LOC_CAUSE})${N}" ;;
    LOST) LOCC="${R}${LOC}${N}${D} (${LOC_CAUSE})${N}" ;;
    *) LOCC="${LOC}" ;;
esac
printf "${B}[2] SLAM${N}      hz=${HZC}Hz  live_pts=%s  loc=%b\n" "$LIVE_PTS" "$LOCC"

# [3] Robot pose
RX=$(pjson "$STATE" 'print(round(float(d["odometry"].get("x",0)),2))')
RY=$(pjson "$STATE" 'print(round(float(d["odometry"].get("y",0)),2))')
RZ=$(pjson "$STATE" 'print(round(float(d["odometry"].get("z",0)),2))')
YAW=$(pjson "$STATE" 'print(round(math.degrees(float(d["odometry"].get("yaw",0))),1))')
VX=$(pjson "$STATE" 'print(round(float(d["odometry"].get("vx",0)),2))')
WZ=$(pjson "$STATE" 'print(round(float(d["odometry"].get("wz",0)),2))')
# Detect absurd odom (SLAM divergence)
ABSURD=$(pjson "$STATE" 'o=d["odometry"]; print(1 if max(abs(float(o.get("x",0))),abs(float(o.get("y",0))))>10000 else 0)')
if [ "$ABSURD" = "1" ]; then
    printf "${B}[3] Robot${N}     ${R}ODOM DIVERGED${N} x=$RX y=$RY — SLAM 需重启\n"
else
    printf "${B}[3] Robot${N}     xy=(%s,%s) z=%s yaw=%s°  v=%sm/s w=%srad/s\n" "$RX" "$RY" "$RZ" "$YAW" "$VX" "$WZ"
fi

# [4] Mission
MSTATE=$(pjson "$STATE" 'print(d.get("mission",{}).get("state","?"))')
WP_IDX=$(pjson "$STATE" 'print(d.get("mission",{}).get("wp_index",0))')
WP_TOT=$(pjson "$STATE" 'print(d.get("mission",{}).get("wp_total",0))')
REPLAN=$(pjson "$HEALTH" 'print(d["sensors"]["navigation"].get("replan_count",0))')
SPEED=$(pjson "$STATE" 'print(d.get("mission",{}).get("speed_scale",1.0))')
DEG=$(pjson "$STATE" 'print(d.get("mission",{}).get("degeneracy","-"))')
case "$MSTATE" in
    EXECUTING|PLANNING) MSC="${G}${MSTATE}${N}" ;;
    SUCCEEDED) MSC="${G}${MSTATE}${N}" ;;
    FAILED|ABORTED) MSC="${R}${MSTATE}${N}" ;;
    IDLE|STANDBY) MSC="${D}${MSTATE}${N}" ;;
    *) MSC="${Y}${MSTATE}${N}" ;;
esac
case "$DEG" in
    NONE|OK) DEGC="${G}${DEG}${N}" ;;
    WARN) DEGC="${Y}${DEG}${N}" ;;
    CRITICAL) DEGC="${R}${DEG}${N}" ;;
    *) DEGC="${DEG}" ;;
esac
printf "${B}[4] Mission${N}   state=%b  wp=%s/%s  replan=%s  speed=%s  deg=%b\n" "$MSC" "$WP_IDX" "$WP_TOT" "$REPLAN" "$SPEED" "$DEGC"

# [5] Path
PATH_PTS=$(pjson "$PATH_JSON" 'p=d.get("path",[]); print(len(p) if isinstance(p,list) else 0)')
if [ "$PATH_PTS" = "0" ] || [ "$PATH_PTS" = "-" ]; then
    printf "${B}[5] Path${N}      ${D}(no active plan)${N}\n"
else
    # Total length + dist-to-goal
    PATH_STATS=$(echo "$PATH_JSON" | python3 -c "
import sys,json,math
try:
    d=json.load(sys.stdin)
    pts=d.get('path',[])
    robot=d.get('robot',{}) or {}
    if not pts:
        print('- - -'); sys.exit(0)
    L=0.0
    for i in range(1,len(pts)):
        a,b=pts[i-1],pts[i]
        L+=math.hypot(a.get('x',0)-b.get('x',0), a.get('y',0)-b.get('y',0))
    # distance robot → first waypoint + first → last
    if len(pts) > 0:
        r_to_first=math.hypot(robot.get('x',0)-pts[0].get('x',0), robot.get('y',0)-pts[0].get('y',0))
        r_to_last =math.hypot(robot.get('x',0)-pts[-1].get('x',0), robot.get('y',0)-pts[-1].get('y',0))
    else:
        r_to_first=r_to_last=0
    print(f'{L:.1f} {r_to_first:.2f} {r_to_last:.2f}')
except Exception:
    print('- - -')
" 2>/dev/null)
    read PLEN DN DGOAL <<< "$PATH_STATS"
    printf "${B}[5] Path${N}      pts=%s  len=%sm  dist_to_next=%sm  dist_to_goal=%sm\n" "$PATH_PTS" "$PLEN" "$DN" "$DGOAL"
fi

# [6] Teleop
TELE=$(pjson "$STATE" 'print(d.get("teleop",{}).get("active",False))')
TELE_N=$(pjson "$STATE" 'print(d.get("teleop",{}).get("clients",0))')
SAFETY=$(pjson "$STATE" 'print(d.get("safety",{}).get("level","-"))')
case "$SAFETY" in
    0) SC="${G}OK${N}" ;;
    1) SC="${Y}WARN${N}" ;;
    2|3) SC="${R}STOP${N}" ;;
    *) SC="${SAFETY}" ;;
esac
printf "${B}[6] Ctrl${N}      teleop=%s(%s)  safety=%b\n" "$TELE" "$TELE_N" "$SC"

# [7] Latest map
LATEST=$(ls -dt "$MAP_DIR"/*/ 2>/dev/null | head -1)
if [ -n "$LATEST" ]; then
    NAME=$(basename "$LATEST")
    MTIME=$(stat -c '%y' "$LATEST" 2>/dev/null | cut -d' ' -f2 | cut -d. -f1)
    PCD_SIZE="-"; BAK_SIZE="-"; PATCHES="-"; DUFO_INFO=""
    [ -f "$LATEST/map.pcd" ] && PCD_SIZE=$(du -h "$LATEST/map.pcd" | cut -f1)
    [ -f "$LATEST/map.pcd.predufo" ] && BAK_SIZE=$(du -h "$LATEST/map.pcd.predufo" | cut -f1)
    [ -d "$LATEST/patches" ] && PATCHES=$(ls "$LATEST/patches" 2>/dev/null | wc -l)
    if [ -f "$LATEST/map.pcd" ] && [ -f "$LATEST/map.pcd.predufo" ]; then
        CLEAN=$(stat -c %s "$LATEST/map.pcd" 2>/dev/null | awk '{printf "%d", $1/16}')
        DIRTY=$(stat -c %s "$LATEST/map.pcd.predufo" 2>/dev/null | awk '{printf "%d", $1/16}')
        DROP=$((DIRTY - CLEAN))
        if [ "$DIRTY" -gt 0 ]; then
            PCT=$(awk -v d=$DROP -v t=$DIRTY 'BEGIN{printf "%.1f", 100*d/t}')
            DUFO_INFO="${G}dufo -${DROP}pts (${PCT}%)${N}"
        fi
    fi
    printf "${B}[7] Map${N}       %s  ${D}@%s${N}  map.pcd=%s  predufo=%s  patches=%s  %b\n" "$NAME" "$MTIME" "$PCD_SIZE" "$BAK_SIZE" "$PATCHES" "$DUFO_INFO"
else
    printf "${B}[7] Map${N}       ${D}(no maps yet)${N}\n"
fi

# [8] Log (recent dynamic_filter or error lines)
LOG=$(journalctl -u lingtu.service --since '10 min ago' --no-pager 2>/dev/null \
    | grep -iE 'dynamic_filter|dufomap|\[ERROR\]' \
    | grep -vE 'VectorMemory|sentence_trans|webrtc.*dispose' \
    | tail -4)
if [ -n "$LOG" ]; then
    printf "${B}[8] Log${N}       ${D}(recent dynamic_filter + errors)${N}\n"
    echo "$LOG" | sed "s/^/             /"
fi
