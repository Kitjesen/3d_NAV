#!/usr/bin/env python3
"""Render a Gazebo frontier exploration trace to an MP4 top-down video."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np


def _points(values: list[Any] | None) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for item in values or []:
        try:
            points.append((float(item[0]), float(item[1])))
        except Exception:
            continue
    return points


def _frontier_points(values: list[dict[str, Any]] | None) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for item in values or []:
        try:
            points.append((float(item["cx"]), float(item["cy"])))
        except Exception:
            continue
    return points


def _bounds(samples: list[dict[str, Any]]) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for sample in samples:
        for point in (
            _points([sample.get("pose")])
            + _points([sample.get("goal")] if sample.get("goal") else [])
            + _points(sample.get("known_points"))
            + _points(sample.get("global_path"))
            + _points(sample.get("local_path"))
            + _frontier_points(sample.get("frontiers"))
        ):
            xs.append(point[0])
            ys.append(point[1])
    if not xs:
        return -2.0, 2.0, -2.0, 2.0
    margin = 0.8
    return min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin


def _draw_polyline(
    frame: np.ndarray,
    points: list[tuple[float, float]],
    project,
    color: tuple[int, int, int],
    thickness: int,
) -> None:
    if len(points) < 2:
        return
    arr = np.array([project(point) for point in points], dtype=np.int32)
    cv2.polylines(frame, [arr], isClosed=False, color=color, thickness=thickness, lineType=cv2.LINE_AA)


def _put(frame: np.ndarray, text: str, x: int, y: int, scale: float = 0.58) -> None:
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (20, 20, 20), 3, cv2.LINE_AA)
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (245, 245, 245), 1, cv2.LINE_AA)


def render(trace_path: Path, output_path: Path, *, width: int, height: int, fps: float) -> dict[str, Any]:
    payload = json.loads(trace_path.read_text(encoding="utf-8"))
    samples = list(payload.get("samples") or [])
    if not samples:
        raise RuntimeError(f"trace has no samples: {trace_path}")

    min_x, max_x, min_y, max_y = _bounds(samples)
    map_w = max(max_x - min_x, 0.1)
    map_h = max(max_y - min_y, 0.1)
    pad_l, pad_r, pad_t, pad_b = 70, 330, 55, 70
    sx = (width - pad_l - pad_r) / map_w
    sy = (height - pad_t - pad_b) / map_h
    scale = min(sx, sy)

    def project(point: tuple[float, float]) -> tuple[int, int]:
        x, y = point
        px = pad_l + int((x - min_x) * scale)
        py = height - pad_b - int((y - min_y) * scale)
        return px, py

    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"failed to open video writer: {output_path}")

    trail: list[tuple[float, float]] = []
    try:
        for sample in samples:
            frame = np.full((height, width, 3), (34, 37, 42), dtype=np.uint8)
            map_x0, map_y0 = project((min_x, max_y))
            map_x1, map_y1 = project((max_x, min_y))
            cv2.rectangle(frame, (map_x0, map_y0), (map_x1, map_y1), (52, 56, 64), -1)
            cv2.rectangle(frame, (map_x0, map_y0), (map_x1, map_y1), (160, 165, 175), 1)

            for x in np.arange(math.floor(min_x), math.ceil(max_x) + 1):
                p0 = project((float(x), min_y))
                p1 = project((float(x), max_y))
                cv2.line(frame, p0, p1, (67, 71, 80), 1)
            for y in np.arange(math.floor(min_y), math.ceil(max_y) + 1):
                p0 = project((min_x, float(y)))
                p1 = project((max_x, float(y)))
                cv2.line(frame, p0, p1, (67, 71, 80), 1)

            for point in _points(sample.get("known_points")):
                cv2.circle(frame, project(point), 2, (94, 130, 255), -1, lineType=cv2.LINE_AA)

            frontiers = _frontier_points(sample.get("frontiers"))
            for point in frontiers:
                cv2.circle(frame, project(point), 7, (0, 190, 255), 2, lineType=cv2.LINE_AA)

            global_path = _points(sample.get("global_path"))
            local_path = _points(sample.get("local_path"))
            _draw_polyline(frame, global_path, project, (70, 220, 90), 3)
            _draw_polyline(frame, local_path, project, (255, 190, 70), 3)

            pose_values = sample.get("pose") or [0.0, 0.0, 0.0]
            pose = (float(pose_values[0]), float(pose_values[1]))
            yaw = float(pose_values[2]) if len(pose_values) > 2 else 0.0
            trail.append(pose)
            _draw_polyline(frame, trail, project, (245, 245, 245), 2)

            goal_values = sample.get("goal")
            if goal_values:
                goal = (float(goal_values[0]), float(goal_values[1]))
                cv2.circle(frame, project(goal), 9, (80, 80, 255), -1, lineType=cv2.LINE_AA)

            robot_px = project(pose)
            cv2.circle(frame, robot_px, 9, (255, 230, 80), -1, lineType=cv2.LINE_AA)
            heading = project((pose[0] + 0.32 * math.cos(yaw), pose[1] + 0.32 * math.sin(yaw)))
            cv2.line(frame, robot_px, heading, (255, 255, 255), 2, lineType=cv2.LINE_AA)

            panel_x = width - pad_r + 20
            cv2.rectangle(frame, (width - pad_r, 0), (width, height), (26, 28, 32), -1)
            _put(frame, "LingTu Gazebo Frontier Gate", 24, 34, 0.72)
            _put(frame, f"t={float(sample.get('t', 0.0)):.1f}s", panel_x, 44)
            _put(frame, f"known cells: {int(sample.get('known_cells', 0))}", panel_x, 82)
            _put(frame, f"area: {float(sample.get('explored_area_m2', 0.0)):.2f} m2", panel_x, 120)
            _put(frame, f"frontiers: {len(frontiers)}", panel_x, 158)
            cmd = sample.get("cmd_vel") or [0.0, 0.0, 0.0]
            _put(frame, f"cmd vx: {float(cmd[0]):+.3f}", panel_x, 196)
            _put(frame, f"cmd wz: {float(cmd[2]):+.3f}", panel_x, 234)
            _put(frame, f"pose: {pose[0]:+.2f}, {pose[1]:+.2f}", panel_x, 272)
            _put(frame, "blue: explored/known", panel_x, 334, 0.5)
            _put(frame, "orange: frontier", panel_x, 364, 0.5)
            _put(frame, "green: global path", panel_x, 394, 0.5)
            _put(frame, "amber: local path", panel_x, 424, 0.5)
            _put(frame, "white: odom trail", panel_x, 454, 0.5)
            writer.write(frame)
    finally:
        writer.release()

    result = payload.get("result") or {}
    return {
        "schema_version": "lingtu.gazebo_frontier_video.v1",
        "trace": str(trace_path),
        "video": str(output_path),
        "frames": len(samples),
        "fps": fps,
        "duration_sec": round(len(samples) / fps, 3),
        "frontier_ok": result.get("ok"),
        "odom_delta_m": result.get("odom_delta_m"),
        "known_cells_delta": result.get("known_cells_delta"),
        "explored_area_delta_m2": result.get("explored_area_delta_m2"),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--summary-out", default="")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=8.0)
    args = parser.parse_args()

    summary = render(
        Path(args.trace),
        Path(args.output),
        width=args.width,
        height=args.height,
        fps=args.fps,
    )
    if args.summary_out:
        path = Path(args.summary_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
