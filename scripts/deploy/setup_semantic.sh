#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# NaviMind 语义导航 — 一键环境配置脚本
#
# 用法:
#   bash scripts/setup_semantic.sh          # 完整安装
#   bash scripts/setup_semantic.sh --check  # 仅检查
#
# 在 Jetson Orin NX 或 x86 Ubuntu 22.04 上运行。
# ═══════════════════════════════════════════════════════════════

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

ok()   { echo -e "${GREEN}[OK]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
fail() { echo -e "${RED}[FAIL]${NC} $1"; }

CHECK_ONLY=false
[[ "$1" == "--check" ]] && CHECK_ONLY=true

echo "═══════════════════════════════════════════════════"
echo "  NaviMind 语义导航 环境配置"
echo "═══════════════════════════════════════════════════"
echo ""

# ── 1. Python 依赖 ──
echo "── [1/5] Python 依赖 ──"
DEPS=(numpy opencv-python ultralytics open-clip-torch openai jieba scipy)
MISSING=()
for dep in "${DEPS[@]}"; do
    pkg=$(echo "$dep" | sed 's/-/_/g')
    if python3 -c "import $pkg" 2>/dev/null; then
        ok "$dep"
    else
        fail "$dep — 未安装"
        MISSING+=("$dep")
    fi
done

if [ ${#MISSING[@]} -gt 0 ] && [ "$CHECK_ONLY" = false ]; then
    echo ""
    echo "安装缺失依赖..."
    pip3 install "${MISSING[@]}"
    ok "依赖安装完成"
fi
echo ""

# ── 2. YOLO-World 模型 ──
echo "── [2/5] YOLO-World 模型 ──"
if python3 -c "
from ultralytics import YOLO
import os
cache = os.path.expanduser('~/.ultralytics')
# Check if any yolov8 world model exists
found = False
for root, dirs, files in os.walk(cache):
    for f in files:
        if 'yolov8' in f and 'world' in f:
            found = True
            break
if not found:
    # Also check current directory
    for f in os.listdir('.'):
        if 'yolov8' in f and 'world' in f:
            found = True
            break
exit(0 if found else 1)
" 2>/dev/null; then
    ok "YOLO-World 权重已缓存"
else
    warn "YOLO-World 权重未缓存 (首次运行会自动下载 ~50MB)"
    if [ "$CHECK_ONLY" = false ]; then
        echo "预下载 YOLO-World..."
        python3 -c "
from ultralytics import YOLO
model = YOLO('yolov8l-worldv2')
print('YOLO-World yolov8l-worldv2 下载完成')
"
        ok "YOLO-World 下载完成"
    fi
fi
echo ""

# ── 3. CLIP 模型 ──
echo "── [3/5] CLIP 模型 (ViT-B/32) ──"
if python3 -c "
import open_clip
model, _, preprocess = open_clip.create_model_and_transforms('ViT-B-32', pretrained='openai')
print('CLIP ViT-B/32 loaded')
" 2>/dev/null; then
    ok "CLIP ViT-B/32 权重已缓存"
else
    warn "CLIP 权重未缓存 (首次运行会自动下载 ~150MB)"
    if [ "$CHECK_ONLY" = false ]; then
        echo "预下载 CLIP..."
        python3 -c "
import open_clip
model, _, preprocess = open_clip.create_model_and_transforms('ViT-B-32', pretrained='openai')
print('CLIP ViT-B/32 下载完成')
"
        ok "CLIP 下载完成"
    fi
fi
echo ""

# ── 4. LLM API Key ──
echo "── [4/5] LLM API Key ──"
if [ -n "$MOONSHOT_API_KEY" ]; then
    ok "MOONSHOT_API_KEY 已设置 (Kimi)"
elif [ -n "$OPENAI_API_KEY" ]; then
    ok "OPENAI_API_KEY 已设置 (OpenAI)"
elif [ -n "$DASHSCOPE_API_KEY" ]; then
    ok "DASHSCOPE_API_KEY 已设置 (Qwen)"
else
    fail "未找到 LLM API Key"
    echo "  请设置以下之一:"
    echo "    export MOONSHOT_API_KEY=sk-xxx    # 推荐: Kimi (国内直连)"
    echo "    export OPENAI_API_KEY=sk-xxx      # OpenAI"
    echo "    export DASHSCOPE_API_KEY=sk-xxx   # 阿里 Qwen"
fi
echo ""

# ── 5. ROS2 包 ──
echo "── [5/5] ROS2 包结构 ──"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_DIR="$(dirname "$SCRIPT_DIR")"

for pkg in semantic_perception semantic_planner; do
    res="$NAV_DIR/src/$pkg/resource/$pkg"
    if [ -f "$res" ]; then
        ok "$pkg/resource/ 存在"
    else
        fail "$pkg/resource/ 缺失"
        if [ "$CHECK_ONLY" = false ]; then
            mkdir -p "$(dirname "$res")"
            touch "$res"
            ok "已创建 $res"
        fi
    fi
done

# 检查是否已 colcon build
if [ -d "$NAV_DIR/install/semantic_perception" ]; then
    ok "semantic_perception 已编译"
else
    warn "semantic_perception 未编译 — 需要运行 colcon build"
fi
if [ -d "$NAV_DIR/install/semantic_planner" ]; then
    ok "semantic_planner 已编译"
else
    warn "semantic_planner 未编译 — 需要运行 colcon build"
fi
echo ""

# ── 汇总 ──
echo "═══════════════════════════════════════════════════"
echo "  配置检查完成"
echo ""
echo "  启动步骤:"
echo "    1. source install/setup.bash"
echo "    2. export MOONSHOT_API_KEY=sk-xxx"
echo "    3. ros2 launch navigation_run.launch.py enable_semantic:=true"
echo ""
echo "  或单独测试:"
echo "    ros2 launch launch/subsystems/semantic.launch.py"
echo "═══════════════════════════════════════════════════"
