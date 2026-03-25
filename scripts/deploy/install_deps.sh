#!/bin/bash
# 安装语义导航系统的Python依赖

set -e  # 遇到错误立即退出

echo "=========================================="
echo "安装语义导航系统依赖"
echo "=========================================="

# 检测Python版本
python_version=$(python3 --version 2>&1 | awk '{print $2}')
echo "Python版本: $python_version"

# 核心依赖
echo ""
echo "[1/7] 安装目标检测依赖..."
pip install ultralytics -q

echo "[2/7] 安装语义编码依赖..."
pip install open-clip-torch -q

echo "[3/7] 安装LLM客户端依赖..."
pip install openai anthropic dashscope -q

echo "[4/7] 安装中文分词依赖..."
pip install jieba -q

echo "[5/7] 安装图像处理依赖..."
pip install opencv-python-headless -q

echo "[6/7] 安装数值计算依赖..."
pip install numpy scipy -q

echo "[7/7] 安装测试框架..."
pip install pytest pytest-cov -q

echo ""
echo "=========================================="
echo "依赖安装完成！"
echo "=========================================="

# 验证安装
echo ""
echo "验证安装..."
python3 -c "import ultralytics; print('✓ ultralytics')"
python3 -c "import open_clip; print('✓ open-clip-torch')"
python3 -c "import openai; print('✓ openai')"
python3 -c "import anthropic; print('✓ anthropic')"
python3 -c "import dashscope; print('✓ dashscope')"
python3 -c "import jieba; print('✓ jieba')"
python3 -c "import cv2; print('✓ opencv-python')"
python3 -c "import numpy; print('✓ numpy')"
python3 -c "import scipy; print('✓ scipy')"
python3 -c "import pytest; print('✓ pytest')"

echo ""
echo "所有依赖验证通过！"
