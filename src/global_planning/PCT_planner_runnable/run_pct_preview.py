"""CLI smoke test for the original PCT planner runtime.

Example:
    python -m global_planning.PCT_planner_runnable.run_pct_preview \
        --tomogram artifacts/scene/tomogram.pickle \
        --start 2 3 0 --goal 18 11 0 --json
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

from .runtime import load_tomogram_planner


def _distance(path: np.ndarray) -> float:
    if path.ndim != 2 or len(path) < 2:
        return 0.0
    deltas = np.diff(path[:, :2], axis=0)
    return float(np.sum(np.linalg.norm(deltas, axis=1)))


def _as_point(values: list[float]) -> np.ndarray:
    if len(values) != 3:
        raise ValueError("point must contain exactly x y z")
    return np.asarray(values, dtype=np.float64)


def run_preview(args: argparse.Namespace) -> dict[str, Any]:
    start = _as_point(args.start)
    goal = _as_point(args.goal)
    planner, paths = load_tomogram_planner(
        args.tomogram,
        repo_root=args.repo_root,
        obstacle_thr=args.obstacle_thr,
    )
    result = planner.plan(start[:2], goal[:2], float(start[2]), float(goal[2]))
    if result is None or len(result) == 0:
        return {
            "ok": False,
            "error": "pct returned no path",
            "runtime": {
                "lib_dir": str(paths.lib_dir),
                "arch": paths.canonical_arch,
                "python": paths.python_tag,
            },
        }

    arr = np.asarray(result, dtype=np.float64)
    finite = bool(np.all(np.isfinite(arr)))
    report = {
        "ok": bool(finite and arr.ndim == 2 and arr.shape[1] >= 3),
        "planner": "pct",
        "path_count": int(len(arr)),
        "path_distance_m": _distance(arr),
        "start": start.tolist(),
        "goal": goal.tolist(),
        "first": arr[0, :3].tolist(),
        "last": arr[-1, :3].tolist(),
        "goal_error_m": float(math.dist(arr[-1, :2], goal[:2])),
        "runtime": {
            "lib_dir": str(paths.lib_dir),
            "arch": paths.canonical_arch,
            "python": paths.python_tag,
        },
    }
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run original PCT planner on a tomogram.")
    parser.add_argument("--tomogram", required=True, help="Path to tomogram.pickle")
    parser.add_argument("--start", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--goal", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--obstacle-thr", type=float, default=49.9)
    parser.add_argument("--repo-root", default=None)
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON only")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    native_log = ""
    try:
        if args.json:
            capture = io.StringIO()
            with contextlib.redirect_stdout(capture), contextlib.redirect_stderr(capture):
                report = run_preview(args)
            native_log = capture.getvalue()
            if native_log:
                report["native_log"] = native_log[-8000:]
        else:
            report = run_preview(args)
    except Exception as exc:
        report = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
        if not args.json:
            print(report["error"], file=sys.stderr)
        else:
            if native_log:
                report["native_log"] = native_log[-8000:]
            print(json.dumps(report, ensure_ascii=False, indent=2))
        return 2

    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
