#!/bin/bash
# ============================================================
# install_semantic_deps.sh — 安装语义导航 Python 依赖 (pip-only)
#
# Usage:
#   bash scripts/install_semantic_deps.sh
#
# 不再需要 clone 任何仓库!
# 所有依赖通过 pip 安装, 适合 Jetson Orin NX 部署。
#
# 方案对比 (为什么不 clone):
#   ┌──────────────┬───────────────┬──────────────────────┐
#   │ 论文          │ 原始依赖       │ 我们的替代           │
#   ├──────────────┼───────────────┼──────────────────────┤
#   │ GroundingDINO │ clone + 编译   │ ultralytics (YOLO-World) │
#   │ ConceptGraphs │ clone + SAM    │ instance_tracker.py  │
#   │ LOVON        │ clone          │ laplacian_filter.py  │
#   │ SG-Nav       │ clone          │ prompt_templates.py  │
#   │ VLMnav       │ 云端 API       │ GPT-4o Vision        │
#   └──────────────┴───────────────┴──────────────────────┘
#
# ============================================================

set -euo pipefail

echo ""
echo "=========================================="
echo " 语义导航依赖安装 (Semantic Nav)"
echo "=========================================="
echo ""

# ── 1. 检测器: YOLO-World (替代 GroundingDINO) ──
echo "[1/5] ultralytics (YOLO-World, CVPR 2024)"
echo "      - 开放词汇检测, ~50MB 权重"
echo "      - Jetson 上 10+ FPS, 支持 TensorRT"
pip install ultralytics

# ── 2. CLIP: 跨模态匹配 (LOVON/ConceptGraphs 方案) ──
echo ""
echo "[2/5] open-clip-torch (CLIP 特征编码)"
echo "      - 图像-文本跨模态匹配"
echo "      - ViT-B/32: ~0.4GB VRAM"
pip install open-clip-torch

# ── 3. Cloud LLM 客户端 ──
echo ""
echo "[3/5] LLM 客户端 (OpenAI / Anthropic / Qwen)"
pip install openai anthropic dashscope

# ── 4. ROS2 图像处理 ──
echo ""
echo "[4/5] cv-bridge + opencv"
pip install opencv-python-headless

# ── 5. 核心依赖 ──
echo ""
echo "[5/5] 核心库"
pip install numpy scipy

echo ""
echo "=========================================="
echo " 安装完成!"
echo "=========================================="
echo ""
echo " 已安装:"
echo "   - ultralytics    → YOLO-World 检测器 (默认)"
echo "   - open-clip-torch → CLIP 特征匹配"
echo "   - openai         → GPT-4o / GPT-4o Vision"
echo "   - anthropic      → Claude 备用"
echo "   - dashscope      → Qwen 备用"
echo ""
echo " YOLO-World 模型权重会在首次运行时自动下载。"
echo ""
echo " ⚡ Jetson 加速 (可选):"
echo "   perception_node 参数 yolo_world.tensorrt: true"
echo "   首次会用几分钟编译 TensorRT engine, 之后推理更快。"
echo ""
echo " 环境变量 (必须设置):"
echo "   export OPENAI_API_KEY='sk-...'"
echo "   export ANTHROPIC_API_KEY='sk-ant-...'   # 可选备用"
echo "   export DASHSCOPE_API_KEY='sk-...'        # 可选备用"
echo ""
