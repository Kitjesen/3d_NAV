#!/usr/bin/env bash
# S100P 一键部署脚本
# 用法: ssh sunrise 'bash ~/data/SLAM/navigation/scripts/deploy_s100p.sh'
# 或本地: ssh sunrise@192.168.66.190 'bash -s' < scripts/deploy_s100p.sh

set -euo pipefail

REPO=~/data/SLAM/navigation
LOG=/tmp/deploy_s100p.log

echo "═══════════════════════════════════════════"
echo "  LingTu S100P 部署"
echo "═══════════════════════════════════════════"

# 1. 拉最新代码
echo "[1/5] 拉取代码..."
cd "$REPO"
git fetch origin main
git reset --hard origin/main
echo "  ✓ $(git log --oneline -1)"

# 2. Dashboard 构建
echo "[2/5] 构建 Dashboard..."
if command -v node &>/dev/null; then
    cd "$REPO/web"
    npm install --silent 2>/dev/null
    npm run build 2>/dev/null
    echo "  ✓ Dashboard built"
else
    echo "  ⚠ Node.js 未安装，跳过 Dashboard 构建"
fi

# 3. 停止旧进程
echo "[3/5] 停止旧进程..."
cd "$REPO"
if [ -f .lingtu/run.json ]; then
    OLD_PID=$(python3 -c "import json; print(json.load(open('.lingtu/run.json')).get('pid',''))" 2>/dev/null || echo "")
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
        kill "$OLD_PID"
        sleep 3
        kill -0 "$OLD_PID" 2>/dev/null && kill -9 "$OLD_PID" 2>/dev/null
        echo "  ✓ 旧进程 $OLD_PID 已停止"
    else
        echo "  ✓ 无旧进程运行"
    fi
    rm -f .lingtu/run.json .lingtu/run.pid
else
    echo "  ✓ 无旧进程运行"
fi

# 4. 启动新进程
echo "[4/5] 启动 lingtu nav..."
source /opt/ros/humble/setup.bash
if [ -f "$REPO/install/setup.bash" ]; then
    source "$REPO/install/setup.bash"
fi
cd "$REPO"
nohup python3 lingtu.py nav --daemon > "$LOG" 2>&1 &
sleep 5

# 5. 验证
echo "[5/5] 验证..."
if [ -f .lingtu/run.json ]; then
    PID=$(python3 -c "import json; print(json.load(open('.lingtu/run.json')).get('pid',''))" 2>/dev/null || echo "")
    if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; then
        PROFILE=$(python3 -c "import json; print(json.load(open('.lingtu/run.json')).get('profile','?'))" 2>/dev/null || echo "?")
        echo "  ✓ PID $PID 运行中 (profile: $PROFILE)"
        echo "  ✓ Dashboard: http://$(hostname -I | awk '{print $1}'):5050"
        echo "  ✓ 日志: $(python3 -c "import json; print(json.load(open('.lingtu/run.json')).get('log_dir','?'))" 2>/dev/null)"
    else
        echo "  ✗ 启动失败，查看日志:"
        tail -10 "$LOG"
        exit 1
    fi
else
    echo "  ✗ run.json 未生成，查看日志:"
    tail -10 "$LOG"
    exit 1
fi

echo ""
echo "═══════════════════════════════════════════"
echo "  部署完成 ✓"
echo "═══════════════════════════════════════════"
