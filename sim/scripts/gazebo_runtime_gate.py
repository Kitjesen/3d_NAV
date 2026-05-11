#!/usr/bin/env python3
"""Start the ROS-native Gazebo runtime and verify the LingTu TF/topic contract."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]


def _terminate_process_tree(proc: subprocess.Popen[Any], timeout_s: float = 8.0) -> None:
    if proc.poll() is not None:
        return
    if os.name == "posix":
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            return
    else:
        proc.terminate()
    try:
        proc.wait(timeout=timeout_s)
        return
    except subprocess.TimeoutExpired:
        pass
    if os.name == "posix":
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
    else:
        proc.kill()
    try:
        proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        if os.name != "posix":
            proc.kill()


def _load_report(path: Path, schema_version: str = "lingtu.gazebo_runtime_smoke.v1") -> dict[str, Any]:
    if not path.exists():
        return {
            "schema_version": schema_version,
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "errors": [f"smoke report not written: {path}"],
        }
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data, dict):
        return data
    return {
        "schema_version": "lingtu.gazebo_runtime_smoke.v1",
        "ok": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "errors": ["smoke report was not a JSON object"],
    }


def run_gate(args: argparse.Namespace) -> dict[str, Any]:
    out_path = Path(args.json_out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    launch_log = Path(args.launch_log)
    launch_log.parent.mkdir(parents=True, exist_ok=True)
    smoke_tmp = out_path.with_suffix(".smoke.tmp.json")
    nav_tmp = out_path.with_suffix(".nav.tmp.json")

    launch_cmd = [
        "ros2",
        "launch",
        str(ROOT / "launch" / "gazebo_simulation.launch.py"),
        f"headless:={'true' if args.headless else 'false'}",
        f"use_bridge:={'true' if args.use_bridge else 'false'}",
        f"spawn_robot:={'true' if args.spawn_robot else 'false'}",
    ]
    smoke_cmd = [
        sys.executable,
        str(ROOT / "tests" / "integration" / "tf_contract_smoke.py"),
        "--timeout-sec",
        str(args.smoke_timeout_sec),
        "--min-samples",
        str(args.min_samples),
        "--require-sensors",
        "--json",
        "--json-out",
        str(smoke_tmp),
    ]
    if args.require_camera:
        smoke_cmd.append("--require-camera")
    nav_launch_cmd = [
        "ros2",
        "launch",
        str(ROOT / "tests" / "planning" / "sim_navigation.launch.py"),
        "use_sim_robot:=false",
        "use_terrain_passthrough:=true",
        f"goal_x:={args.nav_goal_x}",
        f"goal_y:={args.nav_goal_y}",
        f"goal_z:={args.nav_goal_z}",
    ]
    nav_smoke_cmd = [
        sys.executable,
        str(ROOT / "tests" / "integration" / "gazebo_nav_loop_smoke.py"),
        "--timeout-sec",
        str(args.nav_timeout_sec),
        "--goal-x",
        str(args.nav_goal_x),
        "--goal-y",
        str(args.nav_goal_y),
        "--goal-z",
        str(args.nav_goal_z),
        "--json",
        "--json-out",
        str(nav_tmp),
    ]

    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(
        [str(ROOT / "src"), str(ROOT), env.get("PYTHONPATH", "")]
    ).strip(os.pathsep)

    with launch_log.open("wb") as log:
        proc = subprocess.Popen(
            launch_cmd,
            cwd=str(ROOT),
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=(os.name == "posix"),
        )
        nav_proc: subprocess.Popen[Any] | None = None
        try:
            time.sleep(args.warmup_sec)
            smoke = subprocess.run(
                smoke_cmd,
                cwd=str(ROOT),
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=args.smoke_timeout_sec + 10.0,
            )
            report = _load_report(smoke_tmp)
            errors = list(report.get("errors") or [])
            if proc.poll() is not None:
                errors.append(f"gazebo launch exited early with code {proc.returncode}")
            if smoke.returncode != 0 and report.get("ok") is not True:
                errors.append(f"tf_contract_smoke exited with code {smoke.returncode}")
            report["ok"] = bool(report.get("ok")) and not errors
            report["errors"] = errors
            report["gate"] = {
                "name": "gazebo_runtime",
                "launch_cmd": launch_cmd,
                "smoke_cmd": smoke_cmd,
                "launch_log": str(launch_log),
                "smoke_returncode": smoke.returncode,
                "launch_returncode_before_cleanup": proc.poll(),
                "simulation_only": True,
                "real_robot_motion": False,
                "cmd_vel_sent_to_hardware": False,
            }
            if smoke.stdout:
                report["gate"]["smoke_stdout_tail"] = smoke.stdout[-2000:]
            if smoke.stderr:
                report["gate"]["smoke_stderr_tail"] = smoke.stderr[-2000:]
            if args.check_nav_loop and report["ok"]:
                nav_proc = subprocess.Popen(
                    nav_launch_cmd,
                    cwd=str(ROOT),
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=(os.name == "posix"),
                )
                time.sleep(args.nav_warmup_sec)
                nav_smoke = subprocess.run(
                    nav_smoke_cmd,
                    cwd=str(ROOT),
                    env=env,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=args.nav_timeout_sec + 10.0,
                )
                nav_report = _load_report(nav_tmp, "lingtu.gazebo_nav_loop.v1")
                nav_errors = list(nav_report.get("errors") or [])
                if nav_proc.poll() is not None:
                    nav_errors.append(
                        f"navigation launch exited early with code {nav_proc.returncode}"
                    )
                if nav_smoke.returncode != 0 and nav_report.get("ok") is not True:
                    nav_errors.append(
                        f"gazebo_nav_loop_smoke exited with code {nav_smoke.returncode}"
                    )
                nav_report["ok"] = bool(nav_report.get("ok")) and not nav_errors
                nav_report["errors"] = nav_errors
                if nav_smoke.stdout:
                    nav_report["stdout_tail"] = nav_smoke.stdout[-2000:]
                if nav_smoke.stderr:
                    nav_report["stderr_tail"] = nav_smoke.stderr[-2000:]
                report["nav_loop"] = nav_report
                report["ok"] = bool(report["ok"]) and bool(nav_report.get("ok"))
                report["errors"] = list(report.get("errors") or []) + [
                    f"nav_loop: {err}" for err in nav_errors
                ]
                report["gate"]["nav_launch_cmd"] = nav_launch_cmd
                report["gate"]["nav_smoke_cmd"] = nav_smoke_cmd
                report["gate"]["nav_smoke_returncode"] = nav_smoke.returncode
            with out_path.open("w", encoding="utf-8") as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
            return report
        finally:
            if nav_proc is not None:
                _terminate_process_tree(nav_proc)
            _terminate_process_tree(proc)
            if smoke_tmp.exists():
                try:
                    smoke_tmp.unlink()
                except OSError:
                    pass
            if nav_tmp.exists():
                try:
                    nav_tmp.unlink()
                except OSError:
                    pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--json-out",
        default="artifacts/server_sim_closure/gazebo_runtime/report.json",
    )
    parser.add_argument(
        "--launch-log",
        default="artifacts/server_sim_closure/gazebo_runtime/launch.log",
    )
    parser.add_argument("--warmup-sec", type=float, default=12.0)
    parser.add_argument("--smoke-timeout-sec", type=float, default=18.0)
    parser.add_argument("--min-samples", type=int, default=3)
    parser.add_argument("--require-camera", action="store_true")
    parser.add_argument("--check-nav-loop", action="store_true")
    parser.add_argument("--nav-warmup-sec", type=float, default=8.0)
    parser.add_argument("--nav-timeout-sec", type=float, default=30.0)
    parser.add_argument("--nav-goal-x", type=float, default=2.0)
    parser.add_argument("--nav-goal-y", type=float, default=0.0)
    parser.add_argument("--nav-goal-z", type=float, default=0.0)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-bridge", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--spawn-robot", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()

    report = run_gate(args)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report.get("ok") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
