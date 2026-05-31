#!/bin/bash
# ============================================================
# lingtu.sh — MapPilot 统一启动入口
#
# 用法:
#   ./lingtu.sh map     建图模式（手柄遥控 + SLAM 建图）
#   ./lingtu.sh save    保存地图（自动生成 .pcd + .pickle）
#   ./lingtu.sh nav     导航模式（自动加载最新地图）
#   ./lingtu.sh status  检查系统状态（话题、地图、TF）
# ============================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LINGTU_PYTHON="${LINGTU_PYTHON:-python3}"
LINGTU_CLI="$WORKSPACE_DIR/lingtu.py"
TOMOGRAM_DIR="$WORKSPACE_DIR/src/global_planning/PCT_planner/rsc/tomogram"
PCD_DIR="$WORKSPACE_DIR/src/global_planning/PCT_planner/rsc/pcd"
MAP_DIR="$WORKSPACE_DIR/maps"

_run_lingtu() {
    cd "$WORKSPACE_DIR"
    exec "$LINGTU_PYTHON" "$LINGTU_CLI" "$@"
}

# ── 通用初始化 ───────────────────────────────────────────────
_init() {
    cd "$WORKSPACE_DIR"
    if [ ! -f "install/setup.bash" ]; then
        echo -e "${RED}错误: 未找到 install/setup.bash，请先 make build${NC}"
        exit 1
    fi
    source install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE_DIR/config/fastdds_no_shm.xml"
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
}

# ── 自动检测最新地图 ──────────────────────────────────────────
_find_latest_map() {
    local LATEST_PICKLE
    LATEST_PICKLE=$(ls -t "$TOMOGRAM_DIR"/*.pickle 2>/dev/null | head -1)
    if [ -n "$LATEST_PICKLE" ]; then
        echo "${LATEST_PICKLE%.pickle}"
        return
    fi
    local LATEST_PCD
    LATEST_PCD=$(ls -t "$PCD_DIR"/*.pcd 2>/dev/null | head -1)
    if [ -n "$LATEST_PCD" ]; then
        echo "${LATEST_PCD%.pcd}"
        return
    fi
    echo ""
}

# ── 子命令: map ───────────────────────────────────────────────
cmd_map() {
    echo -e "${BOLD}${BLUE}▶ 建图模式${NC}"
    echo -e "  委托当前 Module-first CLI: ${GREEN}python lingtu.py map${NC}"
    echo ""
    _run_lingtu map
}

# ── 子命令: save ──────────────────────────────────────────────
cmd_save() {
    echo -e "${BOLD}${BLUE}▶ 保存地图${NC}"
    _init
    mkdir -p "$MAP_DIR" "$PCD_DIR" "$TOMOGRAM_DIR"

    local TIMESTAMP MAP_NAME
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    MAP_NAME="map_${TIMESTAMP}"
    echo -e "  地图名称: ${GREEN}${MAP_NAME}${NC}"
    echo ""

    # Step 1: 保存 PCD
    echo -e "${GREEN}[1/3] 调用 SLAM 保存服务...${NC}"
    # 服务查找顺序：
    #   /nav/save_map       — slam_fastlio2.launch.py remap 后的标准接口（首选）
    #   /pgo/save_maps      — PGO 节点直接暴露（有回环优化时更准）
    #   /fastlio2/save_map  — namespace 下的 Fast-LIO2 原始服务
    local SVC
    if ros2 service list 2>/dev/null | grep -q "^/nav/save_map$"; then
        SVC="/nav/save_map"
    elif ros2 service list 2>/dev/null | grep -q "^/pgo/save_maps$"; then
        SVC="/pgo/save_maps"
    elif ros2 service list 2>/dev/null | grep -q "^/fastlio2/save_map$"; then
        SVC="/fastlio2/save_map"
    else
        echo -e "${RED}错误: 未找到地图保存服务${NC}"
        echo -e "${YELLOW}已查找: /nav/save_map, /pgo/save_maps, /fastlio2/save_map${NC}"
        echo -e "${YELLOW}请确认 ./lingtu.sh map 的节点仍在运行${NC}"
        exit 1
    fi
    ros2 service call "$SVC" interface/srv/SaveMaps \
        "{file_path: '$MAP_DIR/$MAP_NAME', save_patches: false}"
    echo -e "${GREEN}✓ PCD 已保存: $MAP_DIR/${MAP_NAME}.pcd${NC}"

    # Step 2: 复制到 PCT 工作目录
    echo -e "${GREEN}[2/3] 复制到规划器工作目录...${NC}"
    if [ ! -f "$MAP_DIR/${MAP_NAME}.pcd" ]; then
        echo -e "${RED}错误: PCD 文件未生成，请检查 SLAM 服务输出${NC}"
        exit 1
    fi
    cp "$MAP_DIR/${MAP_NAME}.pcd" "$PCD_DIR/${MAP_NAME}.pcd"
    ln -sf "$PCD_DIR/${MAP_NAME}.pcd" "$PCD_DIR/latest.pcd"
    echo -e "${GREEN}✓ 已复制到 $PCD_DIR/${NC}"

    # Step 3: 生成 Tomogram
    echo -e "${GREEN}[3/3] 生成 PCT Tomogram（2~5 分钟）...${NC}"
    cd "$WORKSPACE_DIR/src/global_planning/PCT_planner/tomography/scripts"
    if python3 tomography.py --scene "$MAP_NAME"; then
        ln -sf "$TOMOGRAM_DIR/${MAP_NAME}.pickle" "$TOMOGRAM_DIR/latest.pickle"
        echo ""
        echo -e "${BOLD}${GREEN}✓ 地图保存完成！${NC}"
        echo -e "  PCD:      ${BLUE}$PCD_DIR/${MAP_NAME}.pcd${NC}"
        echo -e "  Tomogram: ${BLUE}$TOMOGRAM_DIR/${MAP_NAME}.pickle${NC}"
        echo ""
        echo -e "${YELLOW}下一步: ${GREEN}./lingtu.sh nav${NC}"
    else
        echo -e "${RED}错误: Tomogram 生成失败${NC}"
        exit 1
    fi
}

# ── 子命令: nav ───────────────────────────────────────────────
cmd_nav() {
    echo -e "${BOLD}${BLUE}▶ 导航模式${NC}"
    _init

    local MAP_PATH
    MAP_PATH=$(_find_latest_map)
    if [ -z "$MAP_PATH" ]; then
        echo -e "${RED}错误: 未找到地图文件${NC}"
        echo -e "${YELLOW}请先运行: ${GREEN}./lingtu.sh map${YELLOW} + ${GREEN}./lingtu.sh save${NC}"
        exit 1
    fi

    local MAP_BASE
    MAP_BASE=$(basename "$MAP_PATH")
    echo -e "  使用地图: ${GREEN}${MAP_BASE}${NC}"
    if [[ "$MAP_PATH" == *.pickle ]]; then
        echo -e "  ${YELLOW}注意: 未找到 .pickle，将从 .pcd 实时构建 Tomogram（较慢）${NC}"
    fi
    echo ""

    if [ -f "${MAP_PATH}.pickle" ]; then
        _run_lingtu nav --tomogram "${MAP_PATH}.pickle"
    fi
    export NAV_MAP_PATH="$MAP_PATH"
    echo -e "  委托当前 Module-first CLI: ${GREEN}python lingtu.py nav${NC}"
    _run_lingtu nav
}

# ── 子命令: status ─────────────────────────────────────────────
cmd_status() {
    echo -e "${BOLD}${BLUE}▶ 系统状态检查${NC}"
    echo -e "  委托当前 Module-first CLI: ${GREEN}python lingtu.py status${NC}"
    echo ""
    _run_lingtu status
}

# ── 帮助 ──────────────────────────────────────────────────────
cmd_help() {
    echo -e "${BOLD}lingtu.sh — MapPilot 统一启动入口${NC}"
    echo ""
    echo -e "  ${GREEN}./lingtu.sh map${NC}     兼容入口，委托 python lingtu.py map"
    echo -e "  ${GREEN}./lingtu.sh save${NC}    保存地图（PCD + Tomogram 全自动）"
    echo -e "  ${GREEN}./lingtu.sh nav${NC}     兼容入口，委托 python lingtu.py nav"
    echo -e "  ${GREEN}./lingtu.sh status${NC}  兼容入口，委托 python lingtu.py status"
    echo ""
    echo -e "${YELLOW}典型流程:${NC}"
    echo -e "  1. ${GREEN}./lingtu.sh map${NC}   遥控机器人建图"
    echo -e "  2. Ctrl+C 停止建图"
    echo -e "  3. ${GREEN}./lingtu.sh save${NC}  保存地图（等待 2~5 分钟）"
    echo -e "  4. ${GREEN}./lingtu.sh nav${NC}   启动自主导航"
    echo ""
}

# ── 入口 ──────────────────────────────────────────────────────
case "${1:-help}" in
    map)    cmd_map    ;;
    save)   cmd_save   ;;
    nav)    cmd_nav    ;;
    status) cmd_status ;;
    *)      cmd_help   ;;
esac
