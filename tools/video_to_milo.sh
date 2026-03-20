#!/bin/bash
# video_to_milo.sh — 手机视频 → MILo 三维重建 一键脚本
#
# 用法:
#   1. 手机拍一段视频（绕物体/场景走一圈，30-60秒）
#   2. 传到服务器:
#      scp -P 34418 my_video.mp4 root@connect.westd.seetacloud.com:/root/MILo/milo/
#   3. SSH 到服务器运行:
#      bash video_to_milo.sh my_video.mp4
#
# 拍摄技巧:
#   - 绕物体/场景缓慢走一圈（360度最佳）
#   - 保持稳定，不要太快
#   - 避免纯白墙面（COLMAP 需要纹理特征）
#   - 30-60秒，不要太长（帧太多会很慢）
#   - 光照均匀，避免强烈阴影变化

set -e

VIDEO="${1:?用法: bash video_to_milo.sh <视频文件> [场景名]}"
SCENE_NAME="${2:-$(basename "${VIDEO%.*}")}"
WORK_DIR="/root/MILo/milo/data/${SCENE_NAME}"
OUTPUT_DIR="/root/MILo/milo/output/${SCENE_NAME}"

export PATH=/root/miniconda3/bin:/usr/local/cuda/bin:$PATH
source /root/miniconda3/etc/profile.d/conda.sh
conda activate milo
export CUDA_HOME=/usr/local/cuda
TORCH_LIB=$(python -c "import torch,os;print(os.path.join(os.path.dirname(torch.__file__),'lib'))")
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$TORCH_LIB:$LD_LIBRARY_PATH

echo "================================================"
echo "  MILo 三维重建 Pipeline"
echo "  视频: ${VIDEO}"
echo "  场景: ${SCENE_NAME}"
echo "  工作目录: ${WORK_DIR}"
echo "================================================"

# ── Step 1: 提取视频帧 ──
echo ""
echo "[1/4] 提取视频帧..."
mkdir -p "${WORK_DIR}/images"

# 每秒提取 2 帧（-vf fps=2），输出 JPEG
ffmpeg -i "${VIDEO}" -vf "fps=2" -q:v 2 "${WORK_DIR}/images/frame_%04d.jpg" -y 2>&1 | tail -3

NUM_FRAMES=$(ls "${WORK_DIR}/images/"*.jpg 2>/dev/null | wc -l)
echo "  提取了 ${NUM_FRAMES} 帧"

if [ "$NUM_FRAMES" -lt 10 ]; then
    echo "  错误: 帧数太少（<10），视频可能太短或格式不支持"
    exit 1
fi

if [ "$NUM_FRAMES" -gt 300 ]; then
    echo "  警告: 帧数较多（${NUM_FRAMES}），COLMAP 会比较慢"
fi

# ── Step 2: COLMAP 特征提取 + 匹配 + 稀疏重建 ──
echo ""
echo "[2/4] COLMAP 稀疏重建..."
mkdir -p "${WORK_DIR}/sparse"

# 安装 COLMAP（如果还没装）
if ! command -v colmap &>/dev/null; then
    echo "  安装 COLMAP..."
    apt-get update -qq && apt-get install -y -qq colmap 2>&1 | tail -2
fi

DB_PATH="${WORK_DIR}/database.db"
rm -f "${DB_PATH}"

echo "  特征提取..."
colmap feature_extractor \
    --database_path "${DB_PATH}" \
    --image_path "${WORK_DIR}/images" \
    --ImageReader.single_camera 1 \
    --ImageReader.camera_model PINHOLE \
    --SiftExtraction.use_gpu 1 2>&1 | tail -5

echo "  特征匹配..."
colmap exhaustive_matcher \
    --database_path "${DB_PATH}" \
    --SiftMatching.use_gpu 1 2>&1 | tail -5

echo "  稀疏重建..."
mkdir -p "${WORK_DIR}/sparse/0"
colmap mapper \
    --database_path "${DB_PATH}" \
    --image_path "${WORK_DIR}/images" \
    --output_path "${WORK_DIR}/sparse" 2>&1 | tail -10

# 检查重建结果
if [ ! -f "${WORK_DIR}/sparse/0/cameras.bin" ]; then
    echo "  错误: COLMAP 重建失败，可能纹理不够或运动太快"
    exit 1
fi

# 转换为文本格式（MILo 需要）
colmap model_converter \
    --input_path "${WORK_DIR}/sparse/0" \
    --output_path "${WORK_DIR}/sparse/0" \
    --output_type TXT 2>&1 | tail -3

NUM_REGISTERED=$(grep -c "^[0-9]" "${WORK_DIR}/sparse/0/images.txt" 2>/dev/null || echo 0)
# images.txt 每张图占两行
NUM_REGISTERED=$((NUM_REGISTERED / 2))
echo "  COLMAP 注册了 ${NUM_REGISTERED}/${NUM_FRAMES} 帧"

# ── Step 3: MILo 训练 ──
echo ""
echo "[3/4] MILo 训练（预计 25-50 分钟）..."
cd /root/MILo/milo

python train.py \
    -s "${WORK_DIR}" \
    -m "${OUTPUT_DIR}" \
    --imp_metric outdoor \
    --rasterizer radegs \
    --data_device cpu \
    --log_interval 200 2>&1 | tail -20

echo "  训练完成"

# ── Step 4: 提取 Mesh ──
echo ""
echo "[4/4] 提取 Mesh..."
python mesh_extract_sdf.py \
    -s "${WORK_DIR}" \
    -m "${OUTPUT_DIR}" \
    --rasterizer radegs 2>&1 | tail -10

MESH_FILE="${OUTPUT_DIR}/mesh_learnable_sdf.ply"
if [ -f "${MESH_FILE}" ]; then
    SIZE=$(du -h "${MESH_FILE}" | cut -f1)
    echo ""
    echo "================================================"
    echo "  重建完成！"
    echo "  Mesh: ${MESH_FILE}"
    echo "  大小: ${SIZE}"
    echo ""
    echo "  下载到本地:"
    echo "    scp -P 34418 root@connect.westd.seetacloud.com:${MESH_FILE} ./"
    echo ""
    echo "  用 MeshLab 或 Blender 打开查看"
    echo "================================================"
else
    echo "  错误: Mesh 文件未生成"
    exit 1
fi
