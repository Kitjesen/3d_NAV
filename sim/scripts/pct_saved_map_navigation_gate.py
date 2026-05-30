#!/usr/bin/env python3
"""Validate PCT saved-map navigation after runtime relocalization.

This gate composes the Phase 3/4 product evidence:

same-source tomogram + saved map -> relocalization LOCKED -> PCT preview ->
native localPlanner/pathFollower -> MuJoCo closed-loop motion.

It is intentionally simulation-only and delegates motion evidence to
``native_pct_mujoco_gate.py`` instead of duplicating the local planner harness.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from drivers.sim.mujoco_scene_metadata import write_scene_obstacle_metadata  # noqa: E402

DEFAULT_SAVED_MAP_GOAL = [9.0, 11.5]


def _latest(patterns: tuple[str, ...]) -> Path | None:
    for pattern in patterns:
        existing = [path for path in ROOT.glob(pattern) if path.exists()]
        if existing:
            return max(existing, key=lambda path: path.stat().st_mtime)
    return None


def _resolve_tomogram_from_relocalize_report(report: dict[str, Any]) -> Path | None:
    map_value = str(report.get("map_pcd") or "").strip()
    if not map_value:
        return None
    map_path = Path(map_value).expanduser()
    if not map_path.is_absolute():
        map_path = (ROOT / map_path).resolve()
    tomogram = map_path.parent / "tomogram.pickle"
    return tomogram if tomogram.exists() else None


def _resolve_tomogram(
    path: Path | None,
    *,
    relocalize_report: dict[str, Any] | None = None,
) -> Path:
    if path is not None:
        return path
    if relocalize_report is not None:
        same_source = _resolve_tomogram_from_relocalize_report(relocalize_report)
        if same_source is not None:
            return same_source
    latest = _latest(
        (
            "artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live*/**/same_source_map/tomogram.pickle",
            "artifacts/server_sim_closure/mujoco_tare_exploration*/**/same_source_map/tomogram.pickle",
            "artifacts/server_sim_closure/cli_explore_endpoint_mujoco_live*/**/same_source_map/tomogram.pickle",
            "artifacts/server_sim_closure/mujoco_fastlio2_live*/**/same_source_map/tomogram.pickle",
            "artifacts/server_sim_closure/**/same_source_map/tomogram.pickle",
        )
    )
    if latest is None:
        raise FileNotFoundError("no same-source tomogram.pickle found")
    return latest


def _resolve_relocalize_report(path: Path | None) -> Path:
    if path is not None:
        return path
    latest = _latest(
        (
            "artifacts/server_sim_closure/saved_map_relocalize*/report.json",
            "artifacts/saved_map_relocalize*/report.json",
        )
    )
    if latest is None:
        raise FileNotFoundError("no saved-map relocalization report found")
    return latest


def _metadata_for_tomogram(tomogram: Path) -> Path | None:
    candidate = tomogram.parent / "metadata.json"
    return candidate if candidate.exists() else None


def _scene_for_tomogram(tomogram: Path, explicit: Path | None) -> Path:
    if explicit is not None:
        return explicit
    metadata = _metadata_for_tomogram(tomogram)
    if metadata is not None:
        try:
            payload = json.loads(metadata.read_text(encoding="utf-8"))
            world = Path(str(payload.get("world") or ""))
            if world.exists():
                return world
        except Exception:
            pass
    return ROOT / "sim/worlds/industrial_park_scene.xml"


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _validate_relocalization(report: dict[str, Any]) -> tuple[bool, list[str], dict[str, Any]]:
    blockers: list[str] = []
    service = report.get("service") or {}
    localizer = report.get("localizer") or {}
    if report.get("ok") is not True:
        blockers.append("relocalization report.ok is not true")
    if report.get("runtime_relocalization_validated") is not True:
        blockers.append("runtime_relocalization_validated is not true")
    if service.get("success") is not True:
        blockers.append("/nav/relocalize service did not succeed")
    if str(localizer.get("latest_health_state") or "").upper() != "LOCKED":
        blockers.append("localizer latest_health_state is not LOCKED")
    if int(localizer.get("saved_map_cloud_points_latest") or 0) < 1000:
        blockers.append("saved_map_cloud_points_latest below threshold")
    return not blockers, blockers, {
        "service": service,
        "latest_health_state": localizer.get("latest_health_state"),
        "latest_health": localizer.get("latest_health"),
        "saved_map_cloud_points_latest": localizer.get("saved_map_cloud_points_latest"),
        "map_to_odom_xy_m": localizer.get("map_to_odom_xy_m"),
        "map_to_odom_z_abs_m": localizer.get("map_to_odom_z_abs_m"),
    }


def _run_plan_preview(args: argparse.Namespace, *, tomogram: Path, out_path: Path) -> dict[str, Any]:
    cmd = [
        sys.executable,
        str(ROOT / "scripts/plan_preview.py"),
        "--planner",
        "pct",
        "--tomogram",
        str(tomogram),
        "--map-root",
        str(tomogram.parent),
        "--internal-only",
        "--strict",
        "--compact",
        "--max-endpoint-z-error-m",
        str(args.max_endpoint_z_error_m),
        "--timeout",
        str(args.preview_timeout_s),
    ]
    if args.start:
        cmd.extend(["--start", *[str(v) for v in args.start]])
    if args.goal:
        if not args.start:
            cmd.append("--use-last-pose")
        cmd.extend(["--goal", *[str(v) for v in args.goal]])
    started = time.time()
    proc = subprocess.run(
        cmd,
        cwd=str(ROOT),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=max(5.0, float(args.preview_timeout_s) + 10.0),
        check=False,
    )
    stdout = proc.stdout.strip()
    parsed: dict[str, Any] = {}
    for line in reversed(stdout.splitlines()):
        if line.strip().startswith("{"):
            parsed = json.loads(line)
            break
    parsed.setdefault("command", cmd)
    parsed.setdefault("returncode", proc.returncode)
    parsed.setdefault("stderr_tail", proc.stderr[-2000:])
    parsed.setdefault("wall_s", round(time.time() - started, 3))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(parsed, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return parsed


def _point_xyz(point: dict[str, Any]) -> list[float]:
    return [float(point.get("x", 0.0)), float(point.get("y", 0.0)), float(point.get("z", 0.0))]


def _select_preview_case(preview: dict[str, Any]) -> dict[str, Any]:
    for case in preview.get("cases") or []:
        body = case.get("preview") or {}
        if case.get("feasible") is True and body.get("selected_planner") == "pct":
            path = body.get("path") or []
            if len(path) >= 2:
                return case
    raise RuntimeError("no feasible PCT preview case with a non-empty path")


def _build_source_report(
    *,
    preview: dict[str, Any],
    case: dict[str, Any],
    tomogram: Path,
    scene_xml: Path,
    map_metadata: Path | None,
    obstacle_metadata: Path,
    output: Path,
    route_name: str,
) -> dict[str, Any]:
    body = case.get("preview") or {}
    path = [_point_xyz(point) for point in body.get("path") or []]
    start = path[0]
    goal = path[-1]
    pct_runtime = preview.get("pct_runtime_libs") or {}
    source = {
        "schema_version": "lingtu.pct_saved_map_navigation_source.v1",
        "validation_level": "saved_map_relocalized_pct_plan_preview",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "route": route_name,
        "cases": [
            {
                "route": route_name,
                "passed": True,
                "assets": {
                    "scene_xml": str(scene_xml),
                    "tomogram": str(tomogram),
                    "metadata": str(obstacle_metadata),
                    "map_metadata": str(map_metadata or ""),
                    "start": start,
                    "goal": goal,
                },
                "selection": {
                    "primary_planner": "pct",
                    "selected_planner": "pct",
                    "fallback_used": False,
                    "selected_route_ok": True,
                    "policy": "saved_map_relocalized_pct_no_fallback",
                },
                "path_safety": body.get("path_safety") or {"ok": True},
                "planning": [
                    {
                        "planner": "pct",
                        "backend_class": case.get("backend_class") or body.get("backend_class") or "_PCTBackend",
                        "native_backend_used": bool(pct_runtime.get("ok", True)),
                        "native_runtime": pct_runtime or {"ok": True},
                        "plan_ms": body.get("plan_ms"),
                        "route_ok": True,
                        "path_safety": body.get("path_safety") or {"ok": True},
                        "start": start,
                        "goal": goal,
                        "path": path,
                    }
                ],
            }
        ],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(source, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return source


def _run_native_gate(args: argparse.Namespace, *, source_report: Path, out_path: Path) -> dict[str, Any]:
    cmd = [
        sys.executable,
        str(ROOT / "sim/scripts/native_pct_mujoco_gate.py"),
        "--source-report",
        str(source_report),
        "--route",
        str(args.route_name),
        "--planner",
        "pct",
        "--artifact-dir",
        str(out_path.parent),
        "--json-out",
        str(out_path),
        "--ros-domain-id",
        str(args.ros_domain_id),
        "--timeout-s",
        str(args.timeout_s),
        "--sim-vehicle",
        str(args.sim_vehicle),
        "--min-route-progress-ratio",
        str(args.min_route_progress_ratio),
        "--strict",
    ]
    if args.video_out:
        cmd.extend(
            [
                "--video-out",
                str(args.video_out),
                "--video-layout",
                str(args.video_layout),
                "--video-width",
                str(args.video_width),
                "--video-height",
                str(args.video_height),
                "--video-fps",
                str(args.video_fps),
            ]
        )
    proc = subprocess.run(
        cmd,
        cwd=str(ROOT),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=max(float(args.timeout_s) + 90.0, 120.0),
        check=False,
    )
    if out_path.exists():
        report = _load_json(out_path)
    else:
        report = {"ok": False}
    report["command"] = cmd
    report["process_returncode"] = proc.returncode
    report["stdout_tail"] = proc.stdout[-2000:]
    report["stderr_tail"] = proc.stderr[-2000:]
    out_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def run_gate(args: argparse.Namespace) -> dict[str, Any]:
    run_dir = args.run_dir
    run_dir.mkdir(parents=True, exist_ok=True)
    relocalize_report_path = _resolve_relocalize_report(args.relocalize_report)
    relocalize = _load_json(relocalize_report_path)
    tomogram = _resolve_tomogram(args.tomogram, relocalize_report=relocalize)
    scene_xml = _scene_for_tomogram(tomogram, args.scene_xml)
    map_metadata = _metadata_for_tomogram(tomogram)

    report: dict[str, Any] = {
        "schema_version": "lingtu.pct_saved_map_navigation_gate.v1",
        "validation_level": "saved_map_relocalized_pct_navigation",
        "ok": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "tomogram": str(tomogram),
        "scene_xml": str(scene_xml),
        "map_metadata": str(map_metadata or ""),
        "relocalize_report": str(relocalize_report_path),
        "run_dir": str(run_dir),
        "blockers": [],
    }
    try:
        reloc_ok, reloc_blockers, reloc_summary = _validate_relocalization(relocalize)
        report["relocalization"] = {
            "ok": reloc_ok,
            "blockers": reloc_blockers,
            **reloc_summary,
        }

        obstacle_metadata_path = run_dir / "scene_obstacles.json"
        obstacle_metadata = write_scene_obstacle_metadata(
            scene_xml=scene_xml,
            source_map_metadata=map_metadata,
            output=obstacle_metadata_path,
        )
        report["scene_obstacle_metadata"] = {
            "path": str(obstacle_metadata_path),
            "obstacle_count": obstacle_metadata.get("obstacle_count"),
        }

        preview_path = run_dir / "pct_plan_preview.json"
        preview = _run_plan_preview(args, tomogram=tomogram, out_path=preview_path)
        case = _select_preview_case(preview)
        report["plan_preview"] = {
            "ok": bool(preview.get("ok")),
            "path": str(preview_path),
            "selected_case": case.get("name"),
            "selected_planner": (case.get("preview") or {}).get("selected_planner"),
            "fallback_reason": (case.get("preview") or {}).get("fallback_reason"),
            "path_count": len((case.get("preview") or {}).get("path") or []),
            "pct_runtime_libs": preview.get("pct_runtime_libs"),
        }

        source_report_path = run_dir / "pct_saved_map_source_report.json"
        _build_source_report(
            preview=preview,
            case=case,
            tomogram=tomogram,
            scene_xml=scene_xml,
            map_metadata=map_metadata,
            obstacle_metadata=obstacle_metadata_path,
            output=source_report_path,
            route_name=args.route_name,
        )
        report["source_report"] = str(source_report_path)

        native_report_path = run_dir / "native_pct/report.json"
        native = _run_native_gate(args, source_report=source_report_path, out_path=native_report_path)
        report["native_gate"] = native

        blockers = list(reloc_blockers)
        if not reloc_ok:
            blockers.append("saved-map relocalization prerequisite failed")
        if preview.get("ok") is not True:
            blockers.append("PCT plan preview report.ok is not true")
        if report["plan_preview"]["selected_planner"] != "pct":
            blockers.append("PCT preview selected planner is not pct")
        if report["plan_preview"]["fallback_reason"]:
            blockers.append("PCT preview reported fallback_reason")
        if native.get("ok") is not True:
            blockers.append("native PCT MuJoCo closed-loop gate failed")
        if native.get("selected_planner") != "pct":
            blockers.append("native selected_planner is not pct")
        if native.get("fallback_used") is not False:
            blockers.append("native fallback_used is not false")
        if native.get("reached_goal") is not True:
            blockers.append("native gate did not reach goal")
        report["blockers"] = blockers
        report["ok"] = not blockers
    except Exception as exc:
        report["ok"] = False
        report["error"] = str(exc)
        report["blockers"] = [str(exc)]

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tomogram", type=Path, default=None)
    parser.add_argument("--relocalize-report", type=Path, default=None)
    parser.add_argument("--scene-xml", type=Path, default=None)
    parser.add_argument("--run-dir", type=Path, default=ROOT / "artifacts/server_sim_closure/pct_saved_map_navigation")
    parser.add_argument("--json-out", type=Path, default=ROOT / "artifacts/server_sim_closure/pct_saved_map_navigation/report.json")
    parser.add_argument("--route-name", default="saved_map_internal")
    parser.add_argument("--ros-domain-id", default=os.environ.get("ROS_DOMAIN_ID", "161"))
    parser.add_argument("--preview-timeout-s", type=float, default=45.0)
    parser.add_argument("--max-endpoint-z-error-m", type=float, default=1.0)
    parser.add_argument("--timeout-s", type=float, default=120.0)
    parser.add_argument("--min-route-progress-ratio", type=float, default=0.90)
    parser.add_argument("--sim-vehicle", choices=("quadruped", "omni_cart"), default="omni_cart")
    parser.add_argument("--start", nargs="+", type=float, default=None)
    parser.add_argument("--goal", nargs="+", type=float, default=DEFAULT_SAVED_MAP_GOAL)
    parser.add_argument("--video-out", type=Path, default=None)
    parser.add_argument("--video-layout", choices=("observer", "evidence", "scene_overlay"), default="scene_overlay")
    parser.add_argument("--video-width", type=int, default=1280)
    parser.add_argument("--video-height", type=int, default=720)
    parser.add_argument("--video-fps", type=float, default=20.0)
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = run_gate(args)
    print(json.dumps(report, indent=2, sort_keys=True))
    if args.strict and not report.get("ok"):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
