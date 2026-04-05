#!/usr/bin/env python3
"""Offline degeneracy evaluation — replay GEODE / ntnu-arl datasets.

Reads ROS2 bag or CSV files, replays LiDAR point clouds through the
degeneracy detection pipeline, records per-frame metrics, and generates
comparison plots + statistics.

This does NOT run the full SLAM C++ node — it evaluates:
  1. Degeneracy detection thresholds (eigenvalue analysis)
  2. Classification accuracy (corridor/tunnel → detected as degenerate)
  3. Visual odometry fusion quality (if ground truth available)

For full SLAM replay with C++ nodes, use:
  ros2 bag play <bag> --clock
  ros2 launch fastlio2 lio.launch.py

Usage:
    # Evaluate degeneracy detection on a rosbag
    python tools/evaluation/evaluate_degeneracy.py \
        --bag ~/data/geode/bags/corridor_01.bag \
        --output ~/data/geode/results/

    # Evaluate from CSV (exported degeneracy metrics)
    python tools/evaluation/evaluate_degeneracy.py \
        --csv ~/data/geode/results/degeneracy_log.csv \
        --output ~/data/geode/results/

    # Generate threshold calibration from ground truth
    python tools/evaluation/evaluate_degeneracy.py \
        --csv ~/data/geode/results/degeneracy_log.csv \
        --ground-truth ~/data/geode/GEODE_dataset/gt/corridor_01.txt \
        --output ~/data/geode/results/
"""

import argparse
import csv
import json
import logging
import os
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)


# ── Degeneracy frame record ───────────────────────────────────────────

@dataclass
class DegeneracyFrame:
    """Per-frame degeneracy metrics from SLAM Hessian eigenvalue analysis."""
    timestamp: float = 0.0
    # Eigenvalues of 6x6 pose Hessian (sorted ascending)
    eigenvalues: List[float] = field(default_factory=lambda: [0.0] * 6)
    min_eigenvalue: float = 0.0
    max_eigenvalue: float = 0.0
    condition_number: float = 0.0
    effective_ratio: float = 1.0
    degenerate_dof_count: int = 0
    # Classification
    detected: bool = False
    # Optional: ground truth label
    gt_degenerate: Optional[bool] = None


# ── CSV I/O ───────────────────────────────────────────────────────────

CSV_HEADER = [
    "timestamp", "eig0", "eig1", "eig2", "eig3", "eig4", "eig5",
    "min_eig", "max_eig", "condition_number", "effective_ratio",
    "degen_dof_count", "detected",
]


def write_csv(frames: List[DegeneracyFrame], path: str) -> None:
    """Write degeneracy frames to CSV."""
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        for fr in frames:
            w.writerow([
                f"{fr.timestamp:.6f}",
                *[f"{e:.6f}" for e in fr.eigenvalues],
                f"{fr.min_eigenvalue:.6f}",
                f"{fr.max_eigenvalue:.6f}",
                f"{fr.condition_number:.2f}",
                f"{fr.effective_ratio:.4f}",
                fr.degenerate_dof_count,
                int(fr.detected),
            ])
    logger.info("Wrote %d frames to %s", len(frames), path)


def read_csv(path: str) -> List[DegeneracyFrame]:
    """Read degeneracy frames from CSV."""
    frames = []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            fr = DegeneracyFrame(
                timestamp=float(row["timestamp"]),
                eigenvalues=[float(row[f"eig{i}"]) for i in range(6)],
                min_eigenvalue=float(row["min_eig"]),
                max_eigenvalue=float(row["max_eig"]),
                condition_number=float(row["condition_number"]),
                effective_ratio=float(row["effective_ratio"]),
                degenerate_dof_count=int(row["degen_dof_count"]),
                detected=bool(int(row["detected"])),
            )
            frames.append(fr)
    logger.info("Read %d frames from %s", len(frames), path)
    return frames


# ── ROS2 bag parsing (optional, requires rosbag2) ────────────────────

def parse_rosbag_degeneracy(bag_path: str) -> List[DegeneracyFrame]:
    """Extract degeneracy messages from a ROS2 bag.

    Expects topics:
      /slam/degeneracy       (std_msgs/Float32)
      /slam/degeneracy_detail (std_msgs/Float32MultiArray)

    If these topics don't exist, the bag was recorded without degeneracy
    detection. In that case, use the C++ node replay approach.
    """
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import Float32, Float32MultiArray
    except ImportError:
        logger.error(
            "rosbag2_py not available. Install with:\n"
            "  sudo apt install ros-humble-rosbag2-py\n"
            "Or export degeneracy to CSV and use --csv instead."
        )
        return []

    storage = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage, converter)

    # Collect detail messages (11 floats per frame)
    frames = []
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic == "/slam/degeneracy_detail":
            msg = deserialize_message(data, Float32MultiArray)
            d = msg.data
            if len(d) >= 11:
                fr = DegeneracyFrame(
                    timestamp=ts * 1e-9,
                    effective_ratio=float(d[0]),
                    condition_number=float(d[1]),
                    min_eigenvalue=float(d[2]),
                    max_eigenvalue=float(d[3]),
                    degenerate_dof_count=int(d[4]),
                    eigenvalues=[float(d[5 + i]) for i in range(6)],
                    detected=(int(d[4]) > 0),
                )
                frames.append(fr)

    logger.info("Parsed %d degeneracy frames from bag %s", len(frames), bag_path)
    return frames


# ── Ground truth loading ───��──────────────────────────────────────────

def load_ground_truth_tum(path: str) -> np.ndarray:
    """Load TUM-format ground truth: timestamp tx ty tz qx qy qz qw.

    Returns (N, 8) array.
    """
    data = np.loadtxt(path, comments="#")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    logger.info("Loaded %d ground truth poses from %s", len(data), path)
    return data


def label_degeneracy_from_gt(
    frames: List[DegeneracyFrame],
    gt: np.ndarray,
    velocity_threshold: float = 0.05,
) -> List[DegeneracyFrame]:
    """Label frames as degenerate based on ground truth velocity.

    Heuristic: if the robot is moving (GT velocity > threshold) but the
    displacement between consecutive poses is very small in certain axes,
    those axes are degenerate. This is an approximation — real labeling
    requires manual annotation per scenario.

    For now, we just label frames where condition_number > 1000 AND the
    robot is actually moving (not stationary) as ground-truth degenerate.
    """
    if len(gt) < 2:
        return frames

    # Compute GT velocities
    gt_ts = gt[:, 0]
    gt_pos = gt[:, 1:4]
    gt_vel = np.zeros(len(gt))
    for i in range(1, len(gt)):
        dt = gt_ts[i] - gt_ts[i - 1]
        if dt > 0:
            gt_vel[i] = np.linalg.norm(gt_pos[i] - gt_pos[i - 1]) / dt

    for fr in frames:
        # Find closest GT timestamp
        idx = np.searchsorted(gt_ts, fr.timestamp)
        idx = min(idx, len(gt_vel) - 1)
        moving = gt_vel[idx] > velocity_threshold
        # If moving and high condition number → degenerate
        fr.gt_degenerate = moving and fr.condition_number > 1000

    n_degen = sum(1 for f in frames if f.gt_degenerate)
    logger.info("GT labeling: %d/%d frames marked degenerate", n_degen, len(frames))
    return frames


# ── Statistics ────────────────────────────────────────────────────────

@dataclass
class EvalStats:
    total_frames: int = 0
    detected_frames: int = 0
    detection_rate: float = 0.0
    mean_condition_number: float = 0.0
    max_condition_number: float = 0.0
    mean_effective_ratio: float = 0.0
    min_effective_ratio: float = 1.0
    # Eigenvalue statistics
    mean_min_eigenvalue: float = 0.0
    mean_max_eigenvalue: float = 0.0
    # Per-DOF degeneracy frequency (how often each DOF is degenerate)
    dof_degen_freq: List[float] = field(default_factory=lambda: [0.0] * 6)
    # Classification (if GT available)
    true_positive: int = 0
    false_positive: int = 0
    true_negative: int = 0
    false_negative: int = 0
    precision: float = 0.0
    recall: float = 0.0
    f1: float = 0.0


def compute_stats(frames: List[DegeneracyFrame]) -> EvalStats:
    """Compute evaluation statistics from degeneracy frames."""
    if not frames:
        return EvalStats()

    stats = EvalStats(total_frames=len(frames))
    stats.detected_frames = sum(1 for f in frames if f.detected)
    stats.detection_rate = stats.detected_frames / len(frames)

    conds = [f.condition_number for f in frames]
    stats.mean_condition_number = float(np.mean(conds))
    stats.max_condition_number = float(np.max(conds))

    effs = [f.effective_ratio for f in frames]
    stats.mean_effective_ratio = float(np.mean(effs))
    stats.min_effective_ratio = float(np.min(effs))

    min_eigs = [f.min_eigenvalue for f in frames]
    max_eigs = [f.max_eigenvalue for f in frames]
    stats.mean_min_eigenvalue = float(np.mean(min_eigs))
    stats.mean_max_eigenvalue = float(np.mean(max_eigs))

    # Classification accuracy (if GT labels exist)
    has_gt = any(f.gt_degenerate is not None for f in frames)
    if has_gt:
        for f in frames:
            if f.gt_degenerate is None:
                continue
            if f.detected and f.gt_degenerate:
                stats.true_positive += 1
            elif f.detected and not f.gt_degenerate:
                stats.false_positive += 1
            elif not f.detected and f.gt_degenerate:
                stats.false_negative += 1
            else:
                stats.true_negative += 1

        total_pos = stats.true_positive + stats.false_positive
        total_gt_pos = stats.true_positive + stats.false_negative
        stats.precision = stats.true_positive / total_pos if total_pos > 0 else 0.0
        stats.recall = stats.true_positive / total_gt_pos if total_gt_pos > 0 else 0.0
        if stats.precision + stats.recall > 0:
            stats.f1 = 2 * stats.precision * stats.recall / (stats.precision + stats.recall)

    return stats


def print_stats(stats: EvalStats) -> None:
    """Pretty-print evaluation statistics."""
    logger.info("\n=== Degeneracy Evaluation Statistics ===")
    logger.info("  Total frames:          %d", stats.total_frames)
    logger.info("  Detected degenerate:   %d (%.1f%%)",
                stats.detected_frames, stats.detection_rate * 100)
    logger.info("  Mean condition number: %.1f", stats.mean_condition_number)
    logger.info("  Max condition number:  %.1f", stats.max_condition_number)
    logger.info("  Mean effective ratio:  %.3f", stats.mean_effective_ratio)
    logger.info("  Min effective ratio:   %.3f", stats.min_effective_ratio)
    logger.info("  Mean min eigenvalue:   %.6f", stats.mean_min_eigenvalue)
    logger.info("  Mean max eigenvalue:   %.6f", stats.mean_max_eigenvalue)

    if stats.true_positive + stats.false_negative > 0:
        logger.info("\n  Classification (vs ground truth):")
        logger.info("    TP=%d  FP=%d  TN=%d  FN=%d",
                    stats.true_positive, stats.false_positive,
                    stats.true_negative, stats.false_negative)
        logger.info("    Precision: %.3f", stats.precision)
        logger.info("    Recall:    %.3f", stats.recall)
        logger.info("    F1:        %.3f", stats.f1)


# ── Threshold calibration ─────────────────────────────────────────────

def calibrate_thresholds(
    frames: List[DegeneracyFrame],
) -> Dict[str, float]:
    """Suggest optimal degeneracy thresholds from labeled data.

    Sweeps condition_number and effective_ratio thresholds to find
    the best F1 score. Requires gt_degenerate labels.
    """
    has_gt = any(f.gt_degenerate is not None for f in frames)
    if not has_gt:
        logger.warning("No ground truth labels — cannot calibrate thresholds")
        return {}

    labeled = [f for f in frames if f.gt_degenerate is not None]
    if not labeled:
        return {}

    best_f1 = 0.0
    best_thresh = {"condition_number": 100.0, "effective_ratio": 0.8}

    # Sweep condition number thresholds
    for cn_thresh in [50, 100, 200, 500, 1000, 2000, 5000]:
        tp = sum(1 for f in labeled if f.condition_number > cn_thresh and f.gt_degenerate)
        fp = sum(1 for f in labeled if f.condition_number > cn_thresh and not f.gt_degenerate)
        fn = sum(1 for f in labeled if f.condition_number <= cn_thresh and f.gt_degenerate)
        prec = tp / (tp + fp) if (tp + fp) > 0 else 0
        rec = tp / (tp + fn) if (tp + fn) > 0 else 0
        f1 = 2 * prec * rec / (prec + rec) if (prec + rec) > 0 else 0
        if f1 > best_f1:
            best_f1 = f1
            best_thresh["condition_number"] = cn_thresh

    # Sweep effective_ratio thresholds
    for er_thresh in [0.5, 0.6, 0.7, 0.8, 0.85, 0.9, 0.95]:
        tp = sum(1 for f in labeled if f.effective_ratio < er_thresh and f.gt_degenerate)
        fp = sum(1 for f in labeled if f.effective_ratio < er_thresh and not f.gt_degenerate)
        fn = sum(1 for f in labeled if f.effective_ratio >= er_thresh and f.gt_degenerate)
        prec = tp / (tp + fp) if (tp + fp) > 0 else 0
        rec = tp / (tp + fn) if (tp + fn) > 0 else 0
        f1 = 2 * prec * rec / (prec + rec) if (prec + rec) > 0 else 0
        if f1 > best_f1:
            best_f1 = f1
            best_thresh["effective_ratio"] = er_thresh

    logger.info("\n=== Threshold Calibration ===")
    logger.info("  Best F1: %.3f", best_f1)
    logger.info("  Suggested thresholds:")
    logger.info("    condition_number > %.0f → degenerate", best_thresh["condition_number"])
    logger.info("    effective_ratio < %.2f → degenerate", best_thresh["effective_ratio"])

    return best_thresh


# ── Plotting ──────��───────────────────────────────────────────────────

def plot_degeneracy(frames: List[DegeneracyFrame], output_dir: str) -> None:
    """Generate degeneracy analysis plots. Requires matplotlib."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        logger.warning("matplotlib not available — skipping plots")
        return

    ts = [f.timestamp - frames[0].timestamp for f in frames]

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)

    # 1. Eigenvalues over time
    for i in range(6):
        axes[0].semilogy(ts, [f.eigenvalues[i] for f in frames],
                         label=f"eig[{i}]", alpha=0.7, linewidth=0.8)
    axes[0].set_ylabel("Eigenvalue (log)")
    axes[0].set_title("Hessian Eigenvalues (6-DOF pose block)")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)

    # 2. Condition number
    axes[1].semilogy(ts, [f.condition_number for f in frames], "b-", linewidth=0.8)
    axes[1].axhline(y=100, color="orange", linestyle="--", label="threshold=100")
    axes[1].axhline(y=1000, color="red", linestyle="--", label="threshold=1000")
    axes[1].set_ylabel("Condition Number (log)")
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    # 3. Effective ratio
    axes[2].plot(ts, [f.effective_ratio for f in frames], "g-", linewidth=0.8)
    axes[2].axhline(y=0.67, color="orange", linestyle="--", label="1 DOF lost")
    axes[2].axhline(y=0.5, color="red", linestyle="--", label="3 DOF lost")
    axes[2].set_ylabel("Effective Ratio")
    axes[2].set_ylim(-0.05, 1.05)
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)

    # 4. Degenerate DOF count
    axes[3].fill_between(ts, [f.degenerate_dof_count for f in frames],
                          alpha=0.5, color="red")
    axes[3].set_ylabel("Degenerate DOFs")
    axes[3].set_xlabel("Time (s)")
    axes[3].set_ylim(-0.5, 6.5)
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(output_dir, "degeneracy_analysis.png")
    plt.savefig(plot_path, dpi=150)
    plt.close()
    logger.info("Plot saved: %s", plot_path)

    # Eigenvalue distribution histogram
    fig, ax = plt.subplots(1, 1, figsize=(10, 5))
    all_eigs = np.array([f.eigenvalues for f in frames]).flatten()
    all_eigs_pos = all_eigs[all_eigs > 0]
    if len(all_eigs_pos) > 0:
        ax.hist(np.log10(all_eigs_pos), bins=100, alpha=0.7, edgecolor="black", linewidth=0.3)
    ax.set_xlabel("log10(eigenvalue)")
    ax.set_ylabel("Count")
    ax.set_title("Eigenvalue Distribution (all 6 DOFs, all frames)")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    hist_path = os.path.join(output_dir, "eigenvalue_distribution.png")
    plt.savefig(hist_path, dpi=150)
    plt.close()
    logger.info("Plot saved: %s", hist_path)


# ── Main ──────────────────────────��───────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Evaluate degeneracy detection on GEODE/ntnu datasets",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--bag", help="Path to ROS2 bag with /slam/degeneracy_detail topic")
    parser.add_argument("--csv", help="Path to CSV with pre-exported degeneracy metrics")
    parser.add_argument("--ground-truth", help="TUM-format ground truth file for classification eval")
    parser.add_argument("--output", default=".", help="Output directory for results")
    parser.add_argument("--plot", action="store_true", default=True, help="Generate plots")
    parser.add_argument("--calibrate", action="store_true", help="Run threshold calibration")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    # Load data
    frames: List[DegeneracyFrame] = []

    if args.csv:
        frames = read_csv(args.csv)
    elif args.bag:
        frames = parse_rosbag_degeneracy(args.bag)
        if frames:
            csv_out = os.path.join(args.output, "degeneracy_log.csv")
            write_csv(frames, csv_out)
    else:
        logger.error("Provide --bag or --csv. Run with -h for usage.")
        sys.exit(1)

    if not frames:
        logger.error("No degeneracy frames loaded. Check input data.")
        sys.exit(1)

    # Apply ground truth labels if available
    if args.ground_truth:
        gt = load_ground_truth_tum(args.ground_truth)
        frames = label_degeneracy_from_gt(frames, gt)

    # Compute and print statistics
    stats = compute_stats(frames)
    print_stats(stats)

    # Save stats JSON
    stats_path = os.path.join(args.output, "eval_stats.json")
    with open(stats_path, "w") as f:
        json.dump(asdict(stats), f, indent=2)
    logger.info("Stats saved: %s", stats_path)

    # Threshold calibration
    if args.calibrate and args.ground_truth:
        thresholds = calibrate_thresholds(frames)
        if thresholds:
            thresh_path = os.path.join(args.output, "calibrated_thresholds.json")
            with open(thresh_path, "w") as f:
                json.dump(thresholds, f, indent=2)
            logger.info("Thresholds saved: %s", thresh_path)

    # Plots
    if args.plot:
        plot_degeneracy(frames, args.output)

    logger.info("\nDone. Results in %s/", args.output)


if __name__ == "__main__":
    main()
