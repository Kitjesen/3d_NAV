#!/bin/bash
# train_milo.sh — GPU 服务器端 MILo 训练脚本
#
# 用法 (在 GPU 服务器上):
#   # 1. 首次安装
#   bash train_milo.sh install
#
#   # 2. 解压 sunrise 传来的数据
#   tar xzf /root/milo_capture.tar.gz -C /root/MILo/data/
#
#   # 3. 训练
#   bash train_milo.sh train /root/MILo/data/milo_capture
#
#   # 4. 提取 mesh
#   bash train_milo.sh mesh /root/MILo/data/milo_capture
#
#   训练输出: /root/MILo/output/milo_capture/
#   Mesh 文件: /root/MILo/output/milo_capture/mesh_learnable_sdf.ply

set -e

MILO_DIR="/root/MILo"
CONDA_ENV="milo"

case "${1}" in
  install)
    echo "=== 安装 MILo ==="
    cd /root
    if [ ! -d "$MILO_DIR" ]; then
      git clone https://github.com/Anttwo/MILo.git --recursive
    fi
    cd "$MILO_DIR"

    # 创建 conda 环境
    conda create -n "$CONDA_ENV" python=3.9 -y || true
    eval "$(conda shell.bash hook)"
    conda activate "$CONDA_ENV"

    # 安装依赖
    python install.py
    echo "=== 安装完成 ==="
    ;;

  train)
    DATA_DIR="${2:?用法: bash train_milo.sh train <数据目录>}"
    SCENE_NAME=$(basename "$DATA_DIR")
    OUTPUT_DIR="$MILO_DIR/output/$SCENE_NAME"

    echo "=== 开始训练 ==="
    echo "  数据: $DATA_DIR"
    echo "  输出: $OUTPUT_DIR"
    echo "  预计时间: 25-50 分钟"

    cd "$MILO_DIR"
    eval "$(conda shell.bash hook)"
    conda activate "$CONDA_ENV"

    python train.py \
      -s "$DATA_DIR" \
      -m "$OUTPUT_DIR" \
      --imp_metric outdoor \
      --rasterizer radegs \
      --data_device cpu \
      --log_interval 200

    echo "=== 训练完成 ==="
    echo "  模型: $OUTPUT_DIR"
    ;;

  mesh)
    DATA_DIR="${2:?用法: bash train_milo.sh mesh <数据目录>}"
    SCENE_NAME=$(basename "$DATA_DIR")
    OUTPUT_DIR="$MILO_DIR/output/$SCENE_NAME"

    echo "=== 提取 Mesh ==="
    cd "$MILO_DIR"
    eval "$(conda shell.bash hook)"
    conda activate "$CONDA_ENV"

    python mesh_extract_sdf.py \
      -s "$DATA_DIR" \
      -m "$OUTPUT_DIR" \
      --rasterizer radegs

    MESH_FILE="$OUTPUT_DIR/mesh_learnable_sdf.ply"
    if [ -f "$MESH_FILE" ]; then
      SIZE=$(du -h "$MESH_FILE" | cut -f1)
      echo "=== Mesh 提取完成 ==="
      echo "  文件: $MESH_FILE"
      echo "  大小: $SIZE"
      echo ""
      echo "  下载到本地:"
      echo "    scp -P 34418 root@connect.westd.seetacloud.com:$MESH_FILE ./"
    else
      echo "错误: Mesh 文件未生成"
      exit 1
    fi
    ;;

  *)
    echo "用法: bash train_milo.sh <install|train|mesh> [数据目录]"
    echo ""
    echo "  install              首次安装 MILo + 依赖"
    echo "  train <数据目录>     训练 (25-50min on RTX 3090)"
    echo "  mesh  <数据目录>     从训练结果提取 mesh"
    ;;
esac
