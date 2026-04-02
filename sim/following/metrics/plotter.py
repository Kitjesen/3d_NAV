"""Comparison plots for following benchmark results.

Generates matplotlib figures comparing multiple controllers/perception
pipelines across trajectory types.
"""
from __future__ import annotations

import os
from typing import Dict

import numpy as np


def plot_distance_comparison(
    results: Dict[str, dict],
    target_distance: float = 1.5,
    output_path: str = "distance_comparison.png",
    title: str = "Following Distance Over Time",
) -> str:
    """Time-series overlay: distance to person for each controller."""
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 5))

    for label, ts in results.items():
        t = ts["timestamp"] - ts["timestamp"][0]
        ax.plot(t, ts["following_distance"], label=label, linewidth=1.5)

    ax.axhline(y=target_distance, color="k", linestyle="--",
               alpha=0.5, label=f"Target ({target_distance}m)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance to Person (m)")
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, max(5, target_distance * 3))

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def plot_trajectory_2d(
    results: Dict[str, dict],
    output_path: str = "trajectory_2d.png",
    title: str = "Robot & Person Trajectories (Top-Down)",
) -> str:
    """2D overhead view of robot and person trajectories."""
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 8))

    # Person trajectory (same for all runs)
    first = next(iter(results.values()))
    ax.plot(first["person_x"], first["person_y"],
            "k-", linewidth=2, label="Person", zorder=10)
    ax.plot(first["person_x"][0], first["person_y"][0],
            "ko", markersize=10, zorder=11)

    colors = ["#2196F3", "#F44336", "#4CAF50", "#FF9800", "#9C27B0"]
    for i, (label, ts) in enumerate(results.items()):
        c = colors[i % len(colors)]
        ax.plot(ts["robot_x"], ts["robot_y"],
                color=c, linewidth=1.5, label=f"Robot ({label})")
        ax.plot(ts["robot_x"][0], ts["robot_y"][0],
                "o", color=c, markersize=8)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def plot_command_smoothness(
    results: Dict[str, dict],
    output_path: str = "command_smoothness.png",
    title: str = "Command Smoothness (Jerk)",
) -> str:
    """Box plot of command jerk for each controller."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    labels = list(results.keys())

    for ax, (metric, name) in zip(axes, [
        ("cmd_vx", "Forward Vel (m/s)"),
        ("cmd_vy", "Lateral Vel (m/s)"),
        ("cmd_dyaw", "Yaw Rate (rad/s)"),
    ]):
        data = []
        for label in labels:
            ts = results[label]
            jerk = np.abs(np.diff(ts[metric]))
            data.append(jerk)
        ax.boxplot(data, labels=labels)
        ax.set_title(f"{name} Jerk")
        ax.set_ylabel("|Δcmd|")
        ax.grid(True, alpha=0.3)

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def plot_summary_radar(
    summaries: Dict[str, dict],
    output_path: str = "radar_comparison.png",
    title: str = "Controller Comparison",
) -> str:
    """Radar chart of aggregate metrics per controller."""
    import matplotlib.pyplot as plt

    metrics = [
        "mean_distance_error", "mean_lateral_error",
        "mean_heading_error", "smoothness", "tracking_ratio",
    ]
    metric_labels = [
        "Dist Error", "Lateral Error",
        "Heading Error", "Smoothness", "Tracking %",
    ]

    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(projection="polar"))
    angles = np.linspace(0, 2 * np.pi, len(metrics), endpoint=False).tolist()
    angles += angles[:1]

    colors = ["#2196F3", "#F44336", "#4CAF50", "#FF9800", "#9C27B0"]
    for i, (label, summary) in enumerate(summaries.items()):
        values = []
        for m in metrics:
            v = summary.get(m, 0)
            # Normalize: lower error = better (invert for radar)
            if "error" in m:
                v = max(0, 1.0 - min(v, 1.0))  # 0=bad, 1=good
            values.append(v)
        values += values[:1]
        c = colors[i % len(colors)]
        ax.plot(angles, values, "o-", color=c, linewidth=2, label=label)
        ax.fill(angles, values, color=c, alpha=0.1)

    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(metric_labels)
    ax.set_ylim(0, 1.1)
    ax.set_title(title, pad=20)
    ax.legend(loc="upper right", bbox_to_anchor=(1.3, 1.1))

    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return output_path
