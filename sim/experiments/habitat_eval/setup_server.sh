#!/bin/bash
# ============================================================
# NaviMind Habitat 评测环境安装脚本
# 目标服务器: 8×RTX 3090, Ubuntu, Python 3.10
# 用法: bash setup_server.sh
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
VENV_DIR="$PROJECT_ROOT/.venv_habitat"
DATA_DIR="$PROJECT_ROOT/data"

echo "============================================"
echo "  NaviMind Habitat 评测环境安装"
echo "  项目根目录: $PROJECT_ROOT"
echo "============================================"

# ── Step 0: 系统依赖 ──
echo "[0/6] 检查系统依赖..."
if ! command -v python3 &>/dev/null; then
    echo "错误: python3 未安装"
    exit 1
fi

PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "  Python 版本: $PYTHON_VERSION"

# 确保 venv 可用
if ! python3 -c "import venv" 2>/dev/null; then
    echo "  安装 python3-venv..."
    sudo apt-get update -qq && sudo apt-get install -y "python${PYTHON_VERSION}-venv"
fi

# ── Step 1: 创建虚拟环境 ──
echo "[1/6] 创建虚拟环境: $VENV_DIR"
if [ -d "$VENV_DIR" ]; then
    echo "  虚拟环境已存在, 跳过创建"
else
    python3 -m venv "$VENV_DIR"
fi
source "$VENV_DIR/bin/activate"
pip install --upgrade pip setuptools wheel

# ── Step 2: 安装 PyTorch (CUDA 11.8) ──
echo "[2/6] 安装 PyTorch..."
if python3 -c "import torch; print(f'PyTorch {torch.__version__} CUDA={torch.cuda.is_available()}')" 2>/dev/null; then
    echo "  PyTorch 已安装"
else
    pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
fi

# ── Step 3: 安装 habitat-sim (headless, 无 GUI) ──
echo "[3/6] 安装 habitat-sim (headless)..."
if python3 -c "import habitat_sim; print(f'habitat-sim {habitat_sim.__version__}')" 2>/dev/null; then
    echo "  habitat-sim 已安装"
else
    # habitat-sim headless with bullet physics
    pip install habitat-sim-headless \
        -f https://aihabitat.org/habitat-sim/pkg/latest/index.html
fi

# ── Step 4: 安装 habitat-lab ──
echo "[4/6] 安装 habitat-lab..."
if python3 -c "import habitat; print(f'habitat-lab {habitat.__version__}')" 2>/dev/null; then
    echo "  habitat-lab 已安装"
else
    pip install habitat-lab habitat-baselines \
        -f https://aihabitat.org/habitat-lab/pkg/latest/index.html
fi

# ── Step 5: 安装项目 Python 依赖 ──
echo "[5/6] 安装项目依赖..."
pip install numpy pyyaml omegaconf tqdm

# ── Step 6: 下载 HM3D 数据集 ──
echo "[6/6] 下载 HM3D 数据集..."
mkdir -p "$DATA_DIR/scene_datasets" "$DATA_DIR/datasets/objectnav/hm3d/v2"

# HM3D ObjectNav episodes (minival for quick test)
EPISODES_DIR="$DATA_DIR/datasets/objectnav/hm3d/v2"
if [ -f "$EPISODES_DIR/val/val.json.gz" ]; then
    echo "  HM3D ObjectNav episodes 已存在"
else
    echo "  下载 HM3D ObjectNav episodes..."
    echo "  注意: 需要 Matterport 许可证, 请按以下步骤操作:"
    echo "  1. 访问 https://matterport.com/habitat-matterport-3d-research-dataset"
    echo "  2. 接受许可条款, 获取下载凭证"
    echo "  3. 运行: python -m habitat_sim.utils.datasets_download --uids hm3d_minival_v0.2 --data-path $DATA_DIR"
    echo ""
    echo "  尝试自动下载 (如果凭证已配置)..."
    python3 -m habitat_sim.utils.datasets_download \
        --uids hm3d_minival_v0.2 \
        --data-path "$DATA_DIR" 2>/dev/null || {
        echo "  自动下载失败, 请手动下载 (见上方说明)"
    }
fi

# ── 验证安装 ──
echo ""
echo "============================================"
echo "  环境验证"
echo "============================================"

python3 -c "
import sys
print(f'Python:       {sys.version}')

import torch
print(f'PyTorch:      {torch.__version__}')
print(f'CUDA:         {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'GPU count:    {torch.cuda.device_count()}')
    for i in range(torch.cuda.device_count()):
        print(f'  GPU {i}: {torch.cuda.get_device_name(i)}')

import habitat_sim
print(f'habitat-sim:  {habitat_sim.__version__}')

import habitat
print(f'habitat-lab:  OK')

import numpy as np
print(f'NumPy:        {np.__version__}')

print()
print('所有依赖安装成功!')
"

echo ""
echo "============================================"
echo "  安装完成！"
echo "============================================"
echo ""
echo "激活环境:  source $VENV_DIR/bin/activate"
echo "快速测试:  python experiments/habitat_eval/eval_objectnav.py --max-episodes 5 --verbose"
echo "完整评测:  python experiments/habitat_eval/eval_objectnav.py"
echo ""
