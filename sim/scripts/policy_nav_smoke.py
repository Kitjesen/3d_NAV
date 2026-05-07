#!/usr/bin/env python3
"""Run MuJoCo policy-mode gait and navigation smoke checks.

This script is intentionally simulation-only. It uses the in-process
sim_mujoco driver and never talks to real robot services.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def _rpy_from_xyzw(q: Any) -> tuple[float, float, float]:
    x, y, z, w = [float(v) for v in q]
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def _stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"min": None, "max": None, "avg": None}
    return {
        "min": float(min(values)),
        "max": float(max(values)),
        "avg": float(sum(values) / len(values)),
    }


def _safe_float(value: Any) -> float:
    try:
        return float(value)
    except Exception:
        return 0.0


def _load_policy_metadata(policy_path: str) -> dict[str, Any]:
    if not policy_path:
        return {"path": "", "exists": False}
    path = Path(policy_path)
    meta: dict[str, Any] = {"path": str(path), "exists": path.exists()}
    if not path.exists():
        return meta
    try:
        import hashlib

        meta["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    except Exception as exc:
        meta["sha256_error"] = str(exc)
    try:
        import onnxruntime as ort

        sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        meta["input"] = [
            {"name": inp.name, "shape": list(inp.shape), "type": inp.type}
            for inp in sess.get_inputs()
        ]
        meta["output"] = [
            {"name": out.name, "shape": list(out.shape), "type": out.type}
            for out in sess.get_outputs()
        ]
    except Exception as exc:
        meta["onnx_error"] = str(exc)
    return meta


def run_direct_policy(
    *,
    world: str,
    duration: float,
    linear_x: float,
    angular_z: float,
    policy_path: str = "",
) -> dict[str, Any]:
    from drivers.sim.mujoco_driver_module import MujocoDriverModule
    from sim.engine.core.engine import VelocityCommand

    driver = MujocoDriverModule(
        world=world,
        render=False,
        enable_camera=False,
        drive_mode="policy",
        policy_path=policy_path,
    )
    driver.setup()
    engine = driver._engine
    if engine is None:
        raise RuntimeError("MujocoDriverModule setup failed")

    try:
        start = engine.get_robot_state()
        start_xy = np.array(start.position[:2], dtype=float)
        dt = max(0.001, _safe_float(engine.control_dt))
        steps = max(1, int(duration / dt))
        z_values: list[float] = []
        roll_values: list[float] = []
        pitch_values: list[float] = []
        speed_values: list[float] = []
        finite = True
        ctrl_abs_max = 0.0
        cmd = VelocityCommand(linear_x=linear_x, angular_z=angular_z)

        for _ in range(steps):
            state = engine.step(cmd)
            finite = (
                finite
                and bool(np.isfinite(state.position).all())
                and bool(np.isfinite(state.orientation).all())
                and bool(np.isfinite(state.linear_velocity).all())
            )
            roll, pitch, _ = _rpy_from_xyzw(state.orientation)
            z_values.append(float(state.position[2]))
            roll_values.append(abs(roll))
            pitch_values.append(abs(pitch))
            speed_values.append(float(np.linalg.norm(state.linear_velocity[:2])))
            if engine.data is not None:
                finite = (
                    finite
                    and bool(np.isfinite(engine.data.qpos).all())
                    and bool(np.isfinite(engine.data.qvel).all())
                    and bool(np.isfinite(engine.data.ctrl).all())
                )
                ctrl_abs_max = max(ctrl_abs_max, float(np.max(np.abs(engine.data.ctrl))))

        end = engine.get_robot_state()
        moved = float(np.linalg.norm(np.array(end.position[:2], dtype=float) - start_xy))
        return {
            "mode": "direct_policy",
            "world": world,
            "duration_s": duration,
            "policy_loaded": bool(engine.has_policy),
            "policy_path": str(driver._policy_path),
            "finite": finite,
            "moved_m": moved,
            "z": _stats(z_values),
            "roll_abs": _stats(roll_values),
            "pitch_abs": _stats(pitch_values),
            "speed_xy": _stats(speed_values),
            "ctrl_abs_max": ctrl_abs_max,
            "start": [float(v) for v in start.position[:3]],
            "end": [float(v) for v in end.position[:3]],
        }
    finally:
        if driver._engine is not None:
            driver._engine.close()
            driver._engine = None


def run_full_stack_nav(
    *,
    world: str,
    duration: float,
    goal_distance: float,
    policy_path: str = "",
) -> dict[str, Any]:
    from core.blueprints.full_stack import full_stack_blueprint
    from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3

    system = full_stack_blueprint(
        robot="sim_mujoco",
        world=world,
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        render=False,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        drive_mode="policy",
        policy_path=policy_path,
        waypoint_threshold=0.35,
        downsample_dist=0.5,
        run_startup_checks=False,
    ).build()

    driver = system.get_module("MujocoDriverModule")
    ogm = system.get_module("OccupancyGridModule")
    nav = system.get_module("NavigationModule")
    local_planner = system.get_module("LocalPlannerModule")
    path_follower = system.get_module("PathFollowerModule")
    mux = system.get_module("CmdVelMux")

    seen = {
        "costmap": 0,
        "waypoints": 0,
        "local_path": 0,
        "path_follower_cmd": 0,
        "mux_cmd": 0,
        "direct_fallback": 0,
    }
    odom: list[tuple[float, float, float]] = []
    last_mux: list[tuple[float, float]] = []
    ogm.costmap._add_callback(lambda _: seen.__setitem__("costmap", seen["costmap"] + 1))
    nav.waypoint._add_callback(lambda _: seen.__setitem__("waypoints", seen["waypoints"] + 1))
    nav.adapter_status._add_callback(
        lambda e: seen.__setitem__(
            "direct_fallback",
            seen["direct_fallback"] + (1 if e.get("event") == "direct_goal_fallback" else 0),
        )
    )
    local_planner.local_path._add_callback(
        lambda _: seen.__setitem__("local_path", seen["local_path"] + 1)
    )
    path_follower.cmd_vel._add_callback(
        lambda _: seen.__setitem__("path_follower_cmd", seen["path_follower_cmd"] + 1)
    )
    mux.driver_cmd_vel._add_callback(
        lambda m: (
            seen.__setitem__("mux_cmd", seen["mux_cmd"] + 1),
            last_mux.append((float(m.linear.x), float(m.angular.z))),
        )
    )
    driver.odometry._add_callback(
        lambda m: odom.append(
            (float(m.pose.position.x), float(m.pose.position.y), float(m.pose.position.z))
        )
    )

    system.start()
    try:
        warm_deadline = time.time() + min(10.0, max(3.0, duration * 0.25))
        while time.time() < warm_deadline and (seen["costmap"] == 0 or not odom):
            time.sleep(0.1)
        if not odom:
            raise RuntimeError("No odometry from sim_mujoco policy mode")

        engine = driver._engine
        start = odom[-1]
        goal_x = start[0] + goal_distance
        goal_y = start[1]
        nav.goal_pose._deliver(
            PoseStamped(
                pose=Pose(
                    position=Vector3(goal_x, goal_y, 0.0),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
                frame_id="map",
                ts=time.time(),
            )
        )

        deadline = time.time() + duration
        moved = 0.0
        dist_to_goal = math.hypot(goal_x - start[0], goal_y - start[1])
        z_values: list[float] = []
        finite = True
        while time.time() < deadline:
            time.sleep(0.1)
            if odom:
                x, y, z = odom[-1]
                moved = math.hypot(x - start[0], y - start[1])
                dist_to_goal = math.hypot(goal_x - x, goal_y - y)
                z_values.append(z)
                finite = finite and all(math.isfinite(v) for v in odom[-1])

        return {
            "mode": "full_stack_policy_nav",
            "world": world,
            "duration_s": duration,
            "goal_distance_m": goal_distance,
            "policy_loaded": bool(engine.has_policy) if engine is not None else False,
            "policy_path": str(driver._policy_path),
            "finite": finite,
            "seen": seen,
            "moved_m": moved,
            "dist_to_goal_m": dist_to_goal,
            "z": _stats(z_values),
            "nav_state": str(getattr(nav, "_state", "")),
            "last_mux": list(last_mux[-1]) if last_mux else None,
            "start": [float(v) for v in start[:3]],
            "end": [float(v) for v in odom[-1][:3]] if odom else None,
        }
    finally:
        system.stop()


def _passes_direct(result: dict[str, Any], min_motion: float) -> bool:
    return (
        bool(result.get("policy_loaded"))
        and bool(result.get("finite"))
        and float(result.get("moved_m", 0.0)) >= min_motion
        and (result.get("z", {}).get("min") or 0.0) > 0.35
        and (result.get("z", {}).get("max") or 999.0) < 0.55
        and (result.get("roll_abs", {}).get("max") or 999.0) < 0.35
        and (result.get("pitch_abs", {}).get("max") or 999.0) < 0.35
    )


def _passes_nav(result: dict[str, Any], min_motion: float) -> bool:
    seen = result.get("seen", {})
    return (
        bool(result.get("policy_loaded"))
        and bool(result.get("finite"))
        and int(seen.get("costmap", 0)) > 0
        and int(seen.get("waypoints", 0)) > 0
        and int(seen.get("local_path", 0)) > 0
        and int(seen.get("path_follower_cmd", 0)) > 3
        and int(seen.get("mux_cmd", 0)) > 3
        and int(seen.get("direct_fallback", 0)) == 0
        and float(result.get("moved_m", 0.0)) >= min_motion
        and (result.get("z", {}).get("min") or 0.0) > 0.35
        and (result.get("z", {}).get("max") or 999.0) < 0.55
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="open_field")
    parser.add_argument("--direct-duration", type=float, default=6.0)
    parser.add_argument("--nav-duration", type=float, default=18.0)
    parser.add_argument("--goal-distance", type=float, default=1.0)
    parser.add_argument("--linear-x", type=float, default=0.2)
    parser.add_argument("--angular-z", type=float, default=0.0)
    parser.add_argument(
        "--policy-path",
        default="",
        help="Explicit ONNX policy path. If omitted, MujocoDriverModule searches its default candidates.",
    )
    parser.add_argument("--min-direct-motion", type=float, default=0.20)
    parser.add_argument("--min-nav-motion", type=float, default=0.20)
    parser.add_argument("--direct-only", action="store_true")
    parser.add_argument("--nav-only", action="store_true")
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()

    if args.direct_only and args.nav_only:
        parser.error("--direct-only and --nav-only are mutually exclusive")

    results: dict[str, Any] = {"world": args.world, "checks": []}
    direct_result = None
    nav_result = None
    if not args.nav_only:
        direct_result = run_direct_policy(
            world=args.world,
            duration=args.direct_duration,
            linear_x=args.linear_x,
            angular_z=args.angular_z,
            policy_path=args.policy_path,
        )
        direct_result["passed"] = _passes_direct(direct_result, args.min_direct_motion)
        direct_result["policy"] = _load_policy_metadata(str(direct_result.get("policy_path", "")))
        results["checks"].append(direct_result)

    if not args.direct_only:
        nav_result = run_full_stack_nav(
            world=args.world,
            duration=args.nav_duration,
            goal_distance=args.goal_distance,
            policy_path=args.policy_path,
        )
        nav_result["passed"] = _passes_nav(nav_result, args.min_nav_motion)
        nav_result["policy"] = _load_policy_metadata(str(nav_result.get("policy_path", "")))
        results["checks"].append(nav_result)

    results["passed"] = all(bool(check.get("passed")) for check in results["checks"])
    output = json.dumps(results, indent=2, sort_keys=True)
    print(output)
    if args.json_out:
        Path(args.json_out).write_text(output + "\n", encoding="utf-8")
    return 0 if results["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
