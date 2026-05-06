#!/usr/bin/env python3
"""Offline LingTu plan preview for real tomograms.

This tool is intentionally non-motion:
- it does not start LingTu or systemd services
- it does not call Gateway endpoints
- it does not instantiate a robot driver
- it does not publish cmd_vel or goals

It loads a tomogram pickle, injects synthetic odometry into NavigationModule,
calls preview_plan(), prints JSON evidence, and exits.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import pickle
import platform
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


SCRIPT_PATH = Path(__file__).resolve()
REPO_ROOT = SCRIPT_PATH.parents[1]
for _path in (REPO_ROOT, REPO_ROOT / "src"):
    text = str(_path)
    if text not in sys.path:
        sys.path.insert(0, text)


PCT_LIB_NAMES = [
    "libcommon_smoothing.so",
    "liba_star_search.so",
    "libmap_manager.so",
    "libgpmp_optimizer.so",
    "libele_planner_lib.so",
    "a_star.cpython-310-aarch64-linux-gnu.so",
    "ele_planner.cpython-310-aarch64-linux-gnu.so",
    "py_map_manager.cpython-310-aarch64-linux-gnu.so",
    "traj_opt.cpython-310-aarch64-linux-gnu.so",
]


@dataclass(frozen=True)
class LastPose:
    x: float
    y: float
    yaw: float

    @property
    def start(self) -> list[float]:
        return [self.x, self.y, 0.0]


@dataclass(frozen=True)
class PreviewCase:
    name: str
    start: list[float]
    goal: list[float]
    source: str


class TomogramInfo:
    def __init__(
        self,
        *,
        path: Path,
        raw: dict[str, Any],
        grid: np.ndarray,
        resolution: float,
        origin: np.ndarray,
        obstacle_thr: float,
    ) -> None:
        self.path = path
        self.raw = raw
        self.grid = np.asarray(grid, dtype=np.float32)
        self.resolution = float(resolution)
        self.origin = np.asarray(origin[:2], dtype=np.float64)
        self.obstacle_thr = float(obstacle_thr)
        self.shape = tuple(int(v) for v in self.grid.shape)
        if self.grid.ndim != 2:
            raise ValueError(f"ground traversability grid must be 2D, got {self.grid.shape}")
        h, w = self.shape
        self.max_xy = self.origin + np.array([(w - 1) * self.resolution, (h - 1) * self.resolution])
        self.free_mask = np.isfinite(self.grid) & (self.grid < self.obstacle_thr)

    def in_bounds(self, point: list[float] | tuple[float, ...] | np.ndarray) -> bool:
        arr = np.asarray(point, dtype=float).reshape(-1)
        if arr.size < 2 or not np.all(np.isfinite(arr[:2])):
            return False
        return bool(
            self.origin[0] <= arr[0] <= self.max_xy[0]
            and self.origin[1] <= arr[1] <= self.max_xy[1]
        )

    def world_for_cell(self, row: int, col: int, z: float = 0.0) -> list[float]:
        return [
            float(self.origin[0] + col * self.resolution),
            float(self.origin[1] + row * self.resolution),
            float(z),
        ]

    def nearest_free(self, x: float, y: float, z: float = 0.0) -> list[float] | None:
        rows, cols = np.where(self.free_mask)
        if rows.size == 0:
            return None
        xs = self.origin[0] + cols * self.resolution
        ys = self.origin[1] + rows * self.resolution
        dist2 = (xs - float(x)) ** 2 + (ys - float(y)) ** 2
        idx = int(np.argmin(dist2))
        return self.world_for_cell(int(rows[idx]), int(cols[idx]), z)

    def far_free_from(self, point: list[float], z: float = 0.0) -> list[float] | None:
        rows, cols = np.where(self.free_mask)
        if rows.size == 0:
            return None
        xs = self.origin[0] + cols * self.resolution
        ys = self.origin[1] + rows * self.resolution
        dist2 = (xs - float(point[0])) ** 2 + (ys - float(point[1])) ** 2
        idx = int(np.argmax(dist2))
        return self.world_for_cell(int(rows[idx]), int(cols[idx]), z)

    def to_dict(self) -> dict[str, Any]:
        raw_data = self.raw.get("data")
        raw_shape = list(np.asarray(raw_data).shape) if raw_data is not None else None
        return {
            "path": str(self.path),
            "exists": self.path.exists(),
            "size_bytes": self.path.stat().st_size if self.path.exists() else 0,
            "raw_shape": raw_shape,
            "ground_shape": list(self.shape),
            "resolution": round_float(self.resolution),
            "origin": point2(self.origin),
            "max_xy": point2(self.max_xy),
            "free_cells": int(np.count_nonzero(self.free_mask)),
            "total_cells": int(self.free_mask.size),
            "obstacle_thr": round_float(self.obstacle_thr),
        }


def round_float(value: Any, digits: int = 6) -> float | None:
    try:
        num = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(num):
        return None
    return round(num, digits)


def point3(point: Any) -> list[float]:
    arr = np.asarray(point, dtype=float).reshape(-1)
    if arr.size < 3:
        arr = np.pad(arr[:2], (0, 3 - arr.size), constant_values=0.0)
    return [float(round_float(v) or 0.0) for v in arr[:3]]


def point2(point: Any) -> list[float]:
    arr = np.asarray(point, dtype=float).reshape(-1)
    return [float(round_float(v) or 0.0) for v in arr[:2]]


def resolve_map_root(explicit: str | None = None) -> Path:
    if explicit:
        return Path(explicit).expanduser()
    env_root = os.environ.get("NAV_MAP_DIR")
    if env_root:
        return Path(env_root).expanduser()
    home = Path.home()
    for candidate in (
        home / "data" / "nova" / "maps",
        home / "data" / "lingtu" / "maps",
        home / "data" / "inovxio" / "data" / "maps",
    ):
        if candidate.exists():
            return candidate
    return home / "data" / "nova" / "maps"


def resolve_tomogram(map_root: Path, explicit: str | None = None) -> Path:
    if explicit:
        return Path(explicit).expanduser()
    active = map_root / "active"
    return active / "tomogram.pickle"


def load_tomogram_info(path: Path, obstacle_thr: float = 49.9) -> TomogramInfo:
    with path.open("rb") as fh:
        raw = pickle.load(fh)
    if not isinstance(raw, dict):
        raise ValueError(f"unexpected tomogram format in {path}: expected dict")

    resolution = float(raw.get("resolution", 0.2))
    tomo_data = raw.get("data")
    origin: np.ndarray
    if tomo_data is not None:
        arr = np.asarray(tomo_data)
        if arr.ndim != 4 or arr.shape[0] < 1 or arr.shape[1] < 1:
            raise ValueError(f"unexpected tomogram data shape in {path}: {arr.shape}")
        grid = np.asarray(arr[0, 0], dtype=np.float32)
        center = np.asarray(raw.get("center", [0, 0])[:2], dtype=np.float64)
        h, w = grid.shape
        origin = center - np.array([w * resolution / 2.0, h * resolution / 2.0])
    else:
        grid_obj = raw.get("grid", raw.get("traversability"))
        if grid_obj is None:
            raise ValueError(f"tomogram has no data/grid/traversability: {path}")
        grid = np.asarray(grid_obj, dtype=np.float32)
        origin = np.asarray(raw.get("origin", [0, 0])[:2], dtype=np.float64)

    return TomogramInfo(
        path=path,
        raw=raw,
        grid=grid,
        resolution=resolution,
        origin=origin,
        obstacle_thr=obstacle_thr,
    )


def read_last_pose(map_root: Path) -> LastPose | None:
    for path in (map_root / "active" / "last_pose.txt", map_root / "last_pose.txt"):
        if not path.exists():
            continue
        parts = path.read_text(encoding="utf-8", errors="ignore").strip().split()
        if len(parts) < 2:
            continue
        try:
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2]) if len(parts) >= 3 else 0.0
        except ValueError:
            continue
        if all(math.isfinite(v) for v in (x, y, yaw)):
            return LastPose(x=x, y=y, yaw=yaw)
    return None


def parse_xyz(values: list[str] | None, label: str) -> list[float] | None:
    if values is None:
        return None
    if len(values) not in (2, 3):
        raise SystemExit(f"--{label} expects X Y [Z], got {len(values)} values")
    try:
        nums = [float(v) for v in values]
    except ValueError as exc:
        raise SystemExit(f"--{label} expects numeric values") from exc
    if not all(math.isfinite(v) for v in nums):
        raise SystemExit(f"--{label} values must be finite")
    if len(nums) == 2:
        nums.append(0.0)
    return nums


def choose_internal_route(info: TomogramInfo) -> PreviewCase | None:
    h, w = info.shape
    start_hint = info.world_for_cell(max(0, min(h - 1, int(round(h * 0.45)))), max(0, min(w - 1, int(round(w * 0.20)))))
    goal_hint = info.world_for_cell(max(0, min(h - 1, int(round(h * 0.55)))), max(0, min(w - 1, int(round(w * 0.80)))))
    start = info.nearest_free(start_hint[0], start_hint[1])
    goal = info.nearest_free(goal_hint[0], goal_hint[1])
    if start is None or goal is None:
        return None
    if xy_distance(start, goal) < max(2.0, 10.0 * info.resolution):
        far_goal = info.far_free_from(start)
        if far_goal is not None:
            goal = far_goal
    return PreviewCase(
        name="internal_free_route",
        start=point3(start),
        goal=point3(goal),
        source="tomogram_free_cells",
    )


def choose_near_free_goal(info: TomogramInfo, start: list[float]) -> list[float] | None:
    nearest = info.nearest_free(start[0], start[1], start[2] if len(start) > 2 else 0.0)
    if nearest is None:
        return None
    if xy_distance(start, nearest) >= max(1.0, 5.0 * info.resolution):
        return point3(nearest)
    far = info.far_free_from(start, start[2] if len(start) > 2 else 0.0)
    return point3(far) if far is not None else point3(nearest)


def build_preview_cases(
    info: TomogramInfo,
    *,
    explicit_start: list[float] | None = None,
    explicit_goal: list[float] | None = None,
    last_pose: LastPose | None = None,
    use_last_pose: bool = False,
    internal_only: bool = False,
    last_pose_only: bool = False,
) -> list[PreviewCase]:
    cases: list[PreviewCase] = []

    if explicit_goal is not None:
        if explicit_start is not None:
            start = explicit_start
            source = "explicit_start_goal"
        elif use_last_pose and last_pose is not None:
            start = last_pose.start
            source = "last_pose_to_explicit_goal"
        else:
            raise SystemExit("--goal requires --start, or --use-last-pose with active last_pose.txt")
        return [PreviewCase("requested", point3(start), point3(explicit_goal), source)]

    if explicit_start is not None:
        goal = choose_near_free_goal(info, explicit_start)
        if goal is None:
            return []
        return [PreviewCase("start_to_near_free", point3(explicit_start), goal, "explicit_start_auto_goal")]

    if not internal_only and last_pose is not None:
        goal = choose_near_free_goal(info, last_pose.start)
        if goal is not None:
            cases.append(
                PreviewCase(
                    "last_pose_to_near_free",
                    point3(last_pose.start),
                    point3(goal),
                    "active_last_pose_auto_goal",
                )
            )

    if not last_pose_only:
        internal = choose_internal_route(info)
        if internal is not None:
            cases.append(internal)

    return cases


def xy_distance(a: Any, b: Any) -> float:
    av = np.asarray(a, dtype=float).reshape(-1)
    bv = np.asarray(b, dtype=float).reshape(-1)
    if av.size < 2 or bv.size < 2:
        return float("nan")
    return float(np.linalg.norm(bv[:2] - av[:2]))


def path_segments(path: list[dict[str, Any]]) -> list[float]:
    segments: list[float] = []
    for prev, curr in zip(path, path[1:]):
        dist = xy_distance([prev.get("x"), prev.get("y")], [curr.get("x"), curr.get("y")])
        if math.isfinite(dist):
            segments.append(round(dist, 6))
    return segments


def tail_lines(text: str, max_lines: int = 40) -> list[str]:
    lines = [line for line in text.splitlines() if line.strip()]
    if len(lines) <= max_lines:
        return lines
    return lines[-max_lines:]


def extract_first_json_object(text: str) -> tuple[dict[str, Any] | None, str]:
    for index, line in enumerate(text.splitlines()):
        stripped = line.strip()
        if not stripped.startswith("{"):
            continue
        try:
            parsed = json.loads(stripped)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            remaining = "\n".join(text.splitlines()[:index] + text.splitlines()[index + 1 :])
            return parsed, remaining
    return None, text


def call_with_captured_fds(fn: Any) -> tuple[Any, str]:
    """Run fn while capturing Python and native stdout/stderr writes."""

    sys.stdout.flush()
    sys.stderr.flush()
    import tempfile

    with tempfile.TemporaryFile(mode="w+b") as capture:
        old_stdout = os.dup(1)
        old_stderr = os.dup(2)
        try:
            os.dup2(capture.fileno(), 1)
            os.dup2(capture.fileno(), 2)
            result = fn()
            sys.stdout.flush()
            sys.stderr.flush()
        finally:
            os.dup2(old_stdout, 1)
            os.dup2(old_stderr, 2)
            os.close(old_stdout)
            os.close(old_stderr)
        capture.seek(0)
        text = capture.read().decode("utf-8", errors="replace")
    return result, text


def pct_runtime_lib_report(repo_root: Path = REPO_ROOT, machine: str | None = None) -> dict[str, Any]:
    arch = (machine or platform.machine()).lower()
    canonical = {"amd64": "x86_64", "x86_64": "x86_64", "arm64": "aarch64", "aarch64": "aarch64"}.get(arch, arch)
    lib_root = repo_root / "src" / "global_planning" / "PCT_planner" / "planner" / "lib"
    candidates = [lib_root / canonical, lib_root]
    chosen = next((path for path in candidates if path.exists() and any(path.glob("*.so"))), lib_root)
    required = PCT_LIB_NAMES if canonical == "aarch64" else []
    missing = [name for name in required if not (chosen / name).exists()]
    return {
        "machine": arch,
        "canonical_arch": canonical,
        "lib_dir": str(chosen),
        "required": required,
        "missing": missing,
        "ok": not missing,
    }


def make_odom(start: list[float], planning_frame: str) -> Any:
    from core.msgs.geometry import Pose
    from core.msgs.nav import Odometry

    return Odometry(pose=Pose(float(start[0]), float(start[1]), float(start[2])), frame_id=planning_frame)


def run_navigation_preview_inprocess(
    *,
    tomogram_path: Path,
    case: PreviewCase,
    planner: str,
    planning_frame: str,
    obstacle_thr: float,
    timeout_s: float,
    downsample_dist: float,
) -> dict[str, Any]:
    from nav.navigation_module import NavigationModule

    nav = NavigationModule(
        planner=planner,
        tomogram=str(tomogram_path),
        obstacle_thr=obstacle_thr,
        planning_frame_id=planning_frame,
        preview_timeout=timeout_s,
        downsample_dist=downsample_dist,
        enable_ros2_bridge=False,
        allow_direct_goal_fallback=False,
    )
    t0 = time.time()
    try:
        def _preview() -> dict[str, Any]:
            nav.setup()
            nav._on_odom(make_odom(case.start, planning_frame))
            return nav.preview_plan(case.goal[0], case.goal[1], case.goal[2])

        preview, native_output = call_with_captured_fds(_preview)
    finally:
        try:
            nav._preview_executor.shutdown(wait=True, cancel_futures=True)
        except Exception:
            pass

    backend = getattr(nav._planner_svc, "_backend", None)
    backend_available = bool(
        getattr(backend, "available", False)
        if hasattr(backend, "available")
        else (nav._planner_svc.is_ready and nav._planner_svc.has_map)
    )
    preview_path = preview.get("path") or []
    first = preview_path[0] if preview_path else None
    last = preview_path[-1] if preview_path else None
    return {
        "planner": planner,
        "backend_class": type(backend).__name__ if backend is not None else None,
        "backend_available": backend_available,
        "backend_load_error": getattr(backend, "_load_error", "") if backend is not None else "",
        "has_map": bool(nav._planner_svc.has_map),
        "wall_ms": round((time.time() - t0) * 1000.0, 3),
        "preview": preview,
        "first_point": first,
        "last_point": last,
        "segments_m": path_segments(preview_path),
        "native_output_tail": tail_lines(native_output),
    }


def join_output(*parts: Any) -> str:
    text_parts: list[str] = []
    for part in parts:
        if not part:
            continue
        if isinstance(part, bytes):
            text_parts.append(part.decode("utf-8", errors="replace"))
        else:
            text_parts.append(str(part))
    return "\n".join(text_parts)


def subprocess_failure_result(
    *,
    planner: str,
    reason: str,
    error: str,
    wall_ms: float,
    output: str = "",
) -> dict[str, Any]:
    return {
        "planner": planner,
        "backend_class": None,
        "backend_available": False,
        "backend_load_error": "",
        "has_map": False,
        "wall_ms": round(wall_ms, 3),
        "preview": {
            "ok": True,
            "feasible": False,
            "reasons": [reason],
            "error": error,
        },
        "first_point": None,
        "last_point": None,
        "segments_m": [],
        "native_output_tail": tail_lines(output),
    }


def append_native_output(result: dict[str, Any], output: str) -> dict[str, Any]:
    if output:
        result.setdefault("native_output_tail", [])
        result["native_output_tail"].extend(tail_lines(output))
        result["native_output_tail"] = result["native_output_tail"][-40:]
    return result


def run_navigation_preview(
    *,
    tomogram_path: Path,
    case: PreviewCase,
    planner: str,
    planning_frame: str,
    obstacle_thr: float,
    timeout_s: float,
    downsample_dist: float,
) -> dict[str, Any]:
    payload = {
        "case": {
            "name": case.name,
            "start": case.start,
            "goal": case.goal,
            "source": case.source,
        },
        "tomogram": str(tomogram_path),
        "planner": planner,
        "planning_frame": planning_frame,
        "obstacle_thr": obstacle_thr,
        "timeout_s": timeout_s,
        "downsample_dist": downsample_dist,
    }
    encoded = base64.urlsafe_b64encode(
        json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")
    ).decode("ascii")
    cmd = [sys.executable, str(SCRIPT_PATH), "--_child-preview", encoded]
    wall_timeout = max(float(timeout_s) + 3.0, 5.0)
    t0 = time.time()
    try:
        proc = subprocess.run(
            cmd,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=wall_timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        return subprocess_failure_result(
            planner=planner,
            reason="planning_timeout",
            error=f"plan preview subprocess timed out after {wall_timeout:.1f}s",
            wall_ms=(time.time() - t0) * 1000.0,
            output=join_output(exc.stdout, exc.stderr),
        )

    stdout = proc.stdout or ""
    stderr = proc.stderr or ""
    if proc.returncode != 0:
        return subprocess_failure_result(
            planner=planner,
            reason="planning_subprocess_failed",
            error=f"plan preview subprocess exited {proc.returncode}",
            wall_ms=(time.time() - t0) * 1000.0,
            output=join_output(stdout, stderr),
        )

    result, unparsed_stdout = extract_first_json_object(stdout)
    if result is None:
        return subprocess_failure_result(
            planner=planner,
            reason="planning_subprocess_invalid_json",
            error="plan preview subprocess did not emit a JSON object",
            wall_ms=(time.time() - t0) * 1000.0,
            output=join_output(stdout, stderr),
        )
    result["wall_ms"] = round((time.time() - t0) * 1000.0, 3)
    return append_native_output(result, join_output(unparsed_stdout, stderr))


def child_preview(encoded_payload: str) -> int:
    try:
        payload = json.loads(
            base64.urlsafe_b64decode(encoded_payload.encode("ascii")).decode("utf-8")
        )
        case_raw = payload["case"]
        case = PreviewCase(
            name=str(case_raw["name"]),
            start=point3(case_raw["start"]),
            goal=point3(case_raw["goal"]),
            source=str(case_raw.get("source") or "child"),
        )
        result = run_navigation_preview_inprocess(
            tomogram_path=Path(payload["tomogram"]).expanduser(),
            case=case,
            planner=str(payload["planner"]),
            planning_frame=str(payload["planning_frame"]),
            obstacle_thr=float(payload["obstacle_thr"]),
            timeout_s=float(payload["timeout_s"]),
            downsample_dist=float(payload["downsample_dist"]),
        )
        print(json.dumps(result, allow_nan=False, separators=(",", ":"), sort_keys=True))
        return 0
    except Exception as exc:
        print(
            json.dumps(
                {
                    "preview": {
                        "ok": True,
                        "feasible": False,
                        "reasons": ["planning_child_exception"],
                        "error": str(exc),
                    },
                    "backend_available": False,
                    "backend_class": None,
                    "backend_load_error": "",
                    "has_map": False,
                    "native_output_tail": [],
                },
                allow_nan=False,
                separators=(",", ":"),
                sort_keys=True,
            )
        )
        return 0


def summarize_case(
    info: TomogramInfo,
    case: PreviewCase,
    *,
    tomogram_path: Path,
    planner: str,
    planning_frame: str,
    obstacle_thr: float,
    timeout_s: float,
    downsample_dist: float,
    allow_out_of_bounds: bool,
    pct_runtime_libs: dict[str, Any],
) -> dict[str, Any]:
    start_in_bounds = info.in_bounds(case.start)
    goal_in_bounds = info.in_bounds(case.goal)
    result = {
        "name": case.name,
        "source": case.source,
        "start": point3(case.start),
        "goal": point3(case.goal),
        "start_in_bounds": start_in_bounds,
        "goal_in_bounds": goal_in_bounds,
        "ok": False,
        "feasible": False,
        "skipped": False,
        "reasons": [],
        "error": None,
    }
    if not allow_out_of_bounds and (not start_in_bounds or not goal_in_bounds):
        reasons = []
        if not start_in_bounds:
            reasons.append("start_out_of_bounds")
        if not goal_in_bounds:
            reasons.append("goal_out_of_bounds")
        result.update(
            {
                "skipped": True,
                "reasons": reasons,
                "error": ",".join(reasons),
            }
        )
        return result
    if planner == "pct" and not bool(pct_runtime_libs.get("ok", False)):
        result.update(
            {
                "skipped": True,
                "reasons": ["pct_runtime_libs_missing"],
                "error": ",".join(pct_runtime_libs.get("missing") or ["pct_runtime_libs_missing"]),
                "backend_available": False,
                "backend_class": "_PCTBackend",
                "backend_load_error": "PCT runtime libraries are missing",
            }
        )
        return result

    try:
        preview = run_navigation_preview(
            tomogram_path=tomogram_path,
            case=case,
            planner=planner,
            planning_frame=planning_frame,
            obstacle_thr=obstacle_thr,
            timeout_s=timeout_s,
            downsample_dist=downsample_dist,
        )
        result.update(preview)
        body = preview.get("preview") or {}
        result["ok"] = bool(body.get("ok"))
        result["feasible"] = bool(body.get("feasible"))
    except Exception as exc:
        result["error"] = str(exc)
    return result


def case_failures(
    cases: list[dict[str, Any]],
    *,
    strict: bool,
    allow_out_of_bounds: bool,
) -> list[str]:
    failures: list[str] = []
    for case in cases:
        name = case.get("name")
        if case.get("skipped"):
            reasons = case.get("reasons") or ["skipped"]
            failures.extend(f"{name}:{reason}" for reason in reasons)
            continue
        if not case.get("backend_available"):
            failures.append(f"{name}:backend_unavailable")
        if not case.get("feasible"):
            failures.append(f"{name}:infeasible")
        if strict and not allow_out_of_bounds:
            if not case.get("start_in_bounds"):
                failures.append(f"{name}:start_out_of_bounds")
            if not case.get("goal_in_bounds"):
                failures.append(f"{name}:goal_out_of_bounds")
    return failures


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Offline non-motion plan preview against an active tomogram."
    )
    parser.add_argument("--tomogram", help="Path to tomogram.pickle. Defaults to <map-root>/active/tomogram.pickle.")
    parser.add_argument("--map-root", help="Map root. Defaults to NAV_MAP_DIR, then ~/data/nova/maps.")
    parser.add_argument("--start", nargs="+", metavar="XYZ", help="Explicit start as X Y [Z].")
    parser.add_argument("--goal", nargs="+", metavar="XYZ", help="Explicit goal as X Y [Z].")
    parser.add_argument("--use-last-pose", action="store_true", help="Use active last_pose.txt as start when --goal is provided.")
    parser.add_argument("--internal-only", action="store_true", help="Only run an internal free-cell route, skipping last_pose.")
    parser.add_argument("--last-pose-only", action="store_true", help="Only run last_pose to nearest free route.")
    parser.add_argument("--planner", choices=("pct", "astar"), default="pct")
    parser.add_argument("--planning-frame", default="odom")
    parser.add_argument("--obstacle-thr", type=float, default=49.9)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--downsample-dist", type=float, default=2.0)
    parser.add_argument("--strict", action="store_true", help="Fail if any case is infeasible, backend unavailable, or endpoints are out of bounds.")
    parser.add_argument("--allow-out-of-bounds", action="store_true", help="Do not make --strict fail on out-of-bounds endpoints.")
    parser.add_argument("--compact", action="store_true", help="Print compact JSON.")
    parser.add_argument("--_child-preview", help=argparse.SUPPRESS)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    if args._child_preview:
        return child_preview(args._child_preview)
    map_root = resolve_map_root(args.map_root)
    tomogram_path = resolve_tomogram(map_root, args.tomogram)
    pct_libs = pct_runtime_lib_report()

    report: dict[str, Any] = {
        "schema_version": 1,
        "non_motion": True,
        "service_not_started": True,
        "gateway_used": False,
        "driver_used": False,
        "cmd_vel_sent": False,
        "goal_sent": False,
        "map_root": str(map_root),
        "planning_frame": args.planning_frame,
        "planner": args.planner,
        "strict": bool(args.strict),
        "pct_runtime_libs": pct_libs,
        "tomogram": None,
        "last_pose": None,
        "last_pose_in_bounds": None,
        "cases": [],
        "ok": False,
        "error": None,
    }

    try:
        info = load_tomogram_info(tomogram_path, obstacle_thr=args.obstacle_thr)
        report["tomogram"] = info.to_dict()
        last_pose = read_last_pose(map_root)
        if last_pose is not None:
            report["last_pose"] = {
                "x": round_float(last_pose.x),
                "y": round_float(last_pose.y),
                "yaw": round_float(last_pose.yaw),
                "start": point3(last_pose.start),
            }
            report["last_pose_in_bounds"] = info.in_bounds(last_pose.start)

        cases = build_preview_cases(
            info,
            explicit_start=parse_xyz(args.start, "start"),
            explicit_goal=parse_xyz(args.goal, "goal"),
            last_pose=last_pose,
            use_last_pose=bool(args.use_last_pose),
            internal_only=bool(args.internal_only),
            last_pose_only=bool(args.last_pose_only),
        )
        if not cases:
            raise RuntimeError("no preview cases could be created from the tomogram")

        report["cases"] = [
            summarize_case(
                info,
                case,
                tomogram_path=tomogram_path,
                planner=args.planner,
                planning_frame=args.planning_frame,
                obstacle_thr=args.obstacle_thr,
                timeout_s=args.timeout,
                downsample_dist=args.downsample_dist,
                allow_out_of_bounds=bool(args.allow_out_of_bounds),
                pct_runtime_libs=pct_libs,
            )
            for case in cases
        ]

        failures = case_failures(
            report["cases"],
            strict=bool(args.strict),
            allow_out_of_bounds=bool(args.allow_out_of_bounds),
        )
        report["ok"] = not failures
        if failures:
            report["error"] = ";".join(failures)
    except Exception as exc:
        report["ok"] = False
        report["error"] = str(exc)

    if args.compact:
        print(json.dumps(report, allow_nan=False, separators=(",", ":"), sort_keys=True))
    else:
        print(json.dumps(report, allow_nan=False, indent=2, sort_keys=True))

    if args.strict and not report["ok"]:
        return 1
    return 0 if report["error"] is None or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
