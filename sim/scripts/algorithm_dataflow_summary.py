#!/usr/bin/env python3
"""Summarize one live navigation run as a compact dataflow verdict.

This intentionally does not run a scenario. It reads an existing live-run
``report.json`` and answers the first question we should ask during debugging:
did the real data move through the algorithm chain, and where did it stop?
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _count(mapping: dict[str, Any], key: str) -> int:
    try:
        return int(mapping.get(key) or 0)
    except (TypeError, ValueError):
        return 0


def _subcheck_ok(report: dict[str, Any], key: str) -> bool:
    value = report.get(key)
    return isinstance(value, dict) and value.get("ok") is True


def _video_exists(report: dict[str, Any], *, base_dir: Path | None = None) -> bool:
    video = report.get("video")
    if not isinstance(video, dict):
        return False
    if video.get("exists") is True:
        return True
    raw_path = str(video.get("path") or "").strip()
    if not raw_path:
        return False
    path = Path(raw_path)
    if not path.is_absolute() and base_dir is not None:
        path = base_dir / path
    return path.is_file()


def _checkpoint_ok(inspection: dict[str, Any]) -> bool:
    successful = _count(inspection, "successful_navigation_goal_count")
    required = _count(inspection, "min_required_checkpoints")
    if required <= 0:
        required = _count(inspection, "goal_count") or 1
    return successful >= required


def summarize_report(
    report: dict[str, Any],
    *,
    report_path: Path | None = None,
    require_video_file: bool = False,
) -> dict[str, Any]:
    outputs = report.get("outputs") if isinstance(report.get("outputs"), dict) else {}
    inspection = (
        report.get("lingtu_inspection")
        if isinstance(report.get("lingtu_inspection"), dict)
        else {}
    )
    base_dir = report_path.parent if report_path is not None else None

    edges: list[dict[str, Any]] = [
        {
            "id": "raw_lidar_to_fastlio",
            "ok": _count(outputs, "fastlio2_cloud_registered") > 0
            or _count(outputs, "fastlio2_cloud_map") > 0,
            "evidence": {
                "fastlio2_cloud_registered": _count(outputs, "fastlio2_cloud_registered"),
                "fastlio2_cloud_map": _count(outputs, "fastlio2_cloud_map"),
            },
        },
        {
            "id": "fastlio_odometry_to_navigation",
            "ok": _count(outputs, "fastlio2_odometry") > 0
            and _count(outputs, "nav_odometry") > 0,
            "evidence": {
                "fastlio2_odometry": _count(outputs, "fastlio2_odometry"),
                "nav_odometry": _count(outputs, "nav_odometry"),
            },
        },
        {
            "id": "fastlio_map_cloud_to_navigation",
            "ok": (
                _count(outputs, "fastlio2_cloud_map") > 0
                or _count(outputs, "fastlio2_cloud_registered") > 0
            )
            and (
                _count(outputs, "nav_map_cloud") > 0
                or _count(outputs, "nav_registered_cloud") > 0
            ),
            "evidence": {
                "nav_map_cloud": _count(outputs, "nav_map_cloud"),
                "nav_registered_cloud": _count(outputs, "nav_registered_cloud"),
            },
        },
        {
            "id": "global_planner_to_local_planner",
            "ok": _count(inspection, "global_path_count") > 0
            and _count(inspection, "local_path_count") > 0,
            "evidence": {
                "global_path_count": _count(inspection, "global_path_count"),
                "local_path_count": _count(inspection, "local_path_count"),
            },
        },
        {
            "id": "path_follower_to_cmd_vel",
            "ok": _count(outputs, "nav_cmd_vel_nonzero") > 0,
            "evidence": {
                "nav_cmd_vel": _count(outputs, "nav_cmd_vel"),
                "nav_cmd_vel_nonzero": _count(outputs, "nav_cmd_vel_nonzero"),
            },
        },
        {
            "id": "fastlio_motion_consistency",
            "ok": _subcheck_ok(report, "fastlio2_motion_consistency"),
            "evidence": report.get("fastlio2_motion_consistency") or {},
        },
        {
            "id": "fastlio_z_consistency",
            "ok": _subcheck_ok(report, "fastlio2_z_consistency"),
            "evidence": report.get("fastlio2_z_consistency") or {},
        },
        {
            "id": "fastlio_yaw_consistency",
            "ok": _subcheck_ok(report, "fastlio2_yaw_consistency"),
            "evidence": report.get("fastlio2_yaw_consistency") or {},
        },
        {
            "id": "inspection_checkpoints",
            "ok": _checkpoint_ok(inspection),
            "evidence": {
                "successful_navigation_goal_count": _count(
                    inspection, "successful_navigation_goal_count"
                ),
                "min_required_checkpoints": _count(inspection, "min_required_checkpoints"),
                "goal_count": _count(inspection, "goal_count"),
                "patrol_state": inspection.get("patrol_state"),
            },
        },
    ]

    if require_video_file:
        edges.append(
            {
                "id": "video_evidence",
                "ok": _video_exists(report, base_dir=base_dir),
                "evidence": report.get("video") or {},
            }
        )

    failed = [edge for edge in edges if edge["ok"] is not True]
    primary = failed[0]["id"] if failed else ""
    return {
        "ok": not failed,
        "primary_blocker": primary,
        "flow": edges,
        "remaining_gaps": list(report.get("remaining_gaps") or []),
        "source_report_ok": report.get("ok"),
        "source_report": str(report_path) if report_path is not None else "",
    }


def resolve_report_path(value: str) -> Path:
    path = Path(value)
    if path.is_dir():
        return path / "report.json"
    if path.name == "latest.txt":
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.startswith("latest_run_dir="):
                return Path(line.split("=", 1)[1]) / "report.json"
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", help="Path to report.json, a run dir, or latest.txt")
    parser.add_argument("--json-out", default="")
    parser.add_argument("--require-video-file", action="store_true")
    args = parser.parse_args(argv)

    report_path = resolve_report_path(args.report)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    summary = summarize_report(
        report,
        report_path=report_path,
        require_video_file=args.require_video_file,
    )
    text = json.dumps(summary, indent=2, sort_keys=True)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
