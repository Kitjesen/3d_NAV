"""
BPU 检测 + BoT-SORT 跟踪验证脚本。

在 S100P 上运行:
  python tools/test_bpu_tracker.py
  python tools/test_bpu_tracker.py --image /tmp/bus.jpg --prompt "person . bus"
  python tools/test_bpu_tracker.py --frames 20 --gmc none   # 固定摄像头优化

测试内容:
  1. BPUDetector + BPUTracker 加载
  2. 单帧检测 + 跟踪 (功能验证)
  3. 连续 10 帧跑同一张图 (track_id 持久性验证)
  4. 每帧耗时统计 + 平均值
"""

import argparse
import os
import sys
import time
from typing import List

import cv2
import numpy as np

# 允许从 repo 根目录直接运行 (不安装包)
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from src.semantic_perception.semantic_perception.bpu_detector import BPUDetector
from src.semantic_perception.semantic_perception.bpu_tracker import BPUTracker, TrackedDetection


# ──────────────────────────────────────────────
#  参数
# ──────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="BPU Tracker 验证脚本")
    p.add_argument("--image", default="/tmp/bus.jpg",
                   help="测试图片路径 (默认 /tmp/bus.jpg)")
    p.add_argument("--prompt", default="person . bus . car . bicycle",
                   help="检测目标标签 ('. ' 分隔)")
    p.add_argument("--frames", type=int, default=10,
                   help="连续重复推理帧数 (验证 track_id 持久性)")
    p.add_argument("--conf", type=float, default=0.25,
                   help="检测置信度阈值")
    p.add_argument("--tracker", default="botsort", choices=["botsort", "bytetrack"],
                   help="跟踪器类型")
    p.add_argument("--gmc", default="sparseOptFlow",
                   choices=["sparseOptFlow", "orb", "sift", "ecc", "none"],
                   help="全局运动补偿方法 (固定摄像头建议 none)")
    p.add_argument("--model", default=None,
                   help="BPU .hbm 模型路径 (不指定则自动搜索)")
    return p.parse_args()


# ──────────────────────────────────────────────
#  工具函数
# ──────────────────────────────────────────────
def load_test_image(path: str) -> np.ndarray:
    """加载测试图片，若不存在则生成合成图。"""
    if os.path.exists(path):
        img = cv2.imread(path)
        if img is not None:
            print(f"[load] 图片: {path}  shape={img.shape}")
            return img
        print(f"[warn] cv2.imread 失败: {path}, 使用合成图")
    else:
        print(f"[warn] 文件不存在: {path}, 使用合成图")

    # 合成 640×480 彩色图（含矩形模拟目标）
    img = np.random.randint(80, 180, (480, 640, 3), dtype=np.uint8)
    # 画几个矩形模拟行人/车辆
    rects = [(50, 80, 180, 350), (220, 100, 320, 400), (400, 90, 560, 380)]
    for (x1, y1, x2, y2) in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), (200, 100, 50), -1)
    print(f"[load] 合成图 shape={img.shape}")
    return img


def print_tracked(tracked: List[TrackedDetection], frame_idx: int) -> None:
    print(f"  帧 {frame_idx:02d}: {len(tracked)} 个目标")
    for t in tracked:
        x1, y1, x2, y2 = t.bbox.astype(int)
        mask_info = f" mask={t.mask.sum()}px" if t.mask is not None else ""
        print(f"    ID={t.track_id:3d}  {t.label:<16s}  "
              f"conf={t.score:.2f}  bbox=[{x1},{y1},{x2},{y2}]{mask_info}")


def check_id_persistence(results_per_frame: list) -> None:
    """检查 track_id 跨帧稳定性。"""
    print("\n[持久性检查]")
    # 收集所有帧中出现的 track_id
    all_ids: set = set()
    for tracked in results_per_frame:
        for t in tracked:
            all_ids.add(t.track_id)

    if not all_ids:
        print("  无有效 track，跳过持久性检查")
        return

    # 统计每个 ID 出现的帧数
    n_frames = len(results_per_frame)
    id_counts = {tid: 0 for tid in all_ids}
    for tracked in results_per_frame:
        seen = {t.track_id for t in tracked}
        for tid in seen:
            id_counts[tid] += 1

    stable = {tid: cnt for tid, cnt in id_counts.items() if cnt >= n_frames // 2}
    print(f"  总 track ID 数: {len(all_ids)}")
    print(f"  稳定 ID (出现 ≥{n_frames//2} 帧): {len(stable)}")
    for tid, cnt in sorted(stable.items()):
        pct = cnt / n_frames * 100
        print(f"    ID={tid}  出现 {cnt}/{n_frames} 帧 ({pct:.0f}%)")

    if stable:
        print("  [PASS] track_id 持久性验证通过")
    else:
        print("  [WARN] 无稳定 track_id，可能是目标置信度太低或模型未加载")


# ──────────────────────────────────────────────
#  主流程
# ──────────────────────────────────────────────
def main():
    args = parse_args()

    print("=" * 60)
    print("BPU Tracker 验证")
    print(f"  tracker  : {args.tracker}")
    print(f"  gmc      : {args.gmc}")
    print(f"  prompt   : {args.prompt}")
    print(f"  frames   : {args.frames}")
    print(f"  conf_thr : {args.conf}")
    print("=" * 60)

    # ── 1. 加载模型 ──────────────────────────────
    print("\n[1] 加载 BPUDetector ...")
    t0 = time.perf_counter()
    detector = BPUDetector(confidence=args.conf, model_path=args.model)
    try:
        detector.load_model()
    except FileNotFoundError as e:
        print(f"  [ERROR] BPU 模型加载失败: {e}")
        print("  请确认 .hbm 模型在 /home/sunrise/models/ 下，或用 --model 指定路径")
        sys.exit(1)
    load_ms = (time.perf_counter() - t0) * 1000.0
    print(f"  模型加载耗时: {load_ms:.0f}ms  has_seg={detector.has_seg}")

    # ── 2. 创建 BPUTracker ─────────────────────
    print("\n[2] 创建 BPUTracker ...")
    tracker = BPUTracker(
        bpu_detector=detector,
        tracker_type=args.tracker,
        frame_rate=30,
        gmc_method=args.gmc,
        track_high_thresh=args.conf,
        new_track_thresh=args.conf,
    )
    print(f"  跟踪器类型: {args.tracker}")

    # ── 3. 加载测试图片 ────────────────────────
    print("\n[3] 加载测试图片 ...")
    frame = load_test_image(args.image)

    # ── 4. 单帧热身 (BPU 首帧往往较慢) ──────────
    print("\n[4] 热身推理 (首帧, 不计入统计) ...")
    tracker.track(frame, args.prompt)
    tracker.reset()
    print(f"  热身完成  {tracker.timing_summary()}")

    # ── 5. 连续 N 帧推理 ──────────────────────
    print(f"\n[5] 连续 {args.frames} 帧推理 ...")
    detect_times: List[float] = []
    track_times: List[float] = []
    results_per_frame: List[List[TrackedDetection]] = []

    for i in range(args.frames):
        # 对同一张图加轻微噪声模拟帧差 (让 GMC 和 Kalman 有变化量)
        noise = np.random.randint(-3, 4, frame.shape, dtype=np.int16)
        noisy = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        tracked = tracker.track(noisy, args.prompt)
        detect_times.append(tracker.last_detect_ms)
        track_times.append(tracker.last_track_ms)
        results_per_frame.append(tracked)

        print_tracked(tracked, i)

    # ── 6. 耗时统计 ───────────────────────────
    print("\n[6] 耗时统计")
    det_arr = np.array(detect_times)
    trk_arr = np.array(track_times)
    tot_arr = det_arr + trk_arr

    def _stats(arr: np.ndarray, name: str):
        print(f"  {name:10s}: avg={arr.mean():.1f}ms  "
              f"min={arr.min():.1f}ms  max={arr.max():.1f}ms  "
              f"p95={np.percentile(arr, 95):.1f}ms")

    _stats(det_arr, "detect")
    _stats(trk_arr, "track")
    _stats(tot_arr, "total")
    print(f"  推算 FPS: {1000.0 / tot_arr.mean():.1f}")

    # ── 7. track_id 持久性验证 ─────────────────
    check_id_persistence(results_per_frame)

    # ── 8. 清理 ──────────────────────────────
    detector.shutdown()
    print("\n[完成] BPU Tracker 验证结束")


if __name__ == "__main__":
    main()
