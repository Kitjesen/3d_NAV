#!/usr/bin/env python3
"""Record a simulation-only MuJoCo policy navigation run to MP4.

This script never talks to real robot services. It runs the in-process
sim_mujoco driver through the LingTu full stack, sends one map-frame goal, and
records an offscreen MuJoCo view with navigation telemetry overlaid.
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


def _put_text(frame: np.ndarray, text: str, y: int, color=(255, 255, 255)) -> None:
    import cv2

    cv2.putText(
        frame,
        text,
        (20, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        color,
        2,
        cv2.LINE_AA,
    )


def _draw_inset(
    frame: np.ndarray,
    *,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    trail: list[tuple[float, float]],
) -> None:
    import cv2

    h, w = frame.shape[:2]
    x0, y0 = w - 300, 30
    x1, y1 = w - 20, 250
    cv2.rectangle(frame, (x0, y0), (x1, y1), (20, 20, 20), -1)
    cv2.rectangle(frame, (x0, y0), (x1, y1), (220, 220, 220), 1)
    points = [start_xy, goal_xy, *trail]
    min_x = min(p[0] for p in points) - 0.3
    max_x = max(p[0] for p in points) + 0.3
    min_y = min(p[1] for p in points) - 0.3
    max_y = max(p[1] for p in points) + 0.3
    sx = (x1 - x0 - 30) / max(max_x - min_x, 0.1)
    sy = (y1 - y0 - 30) / max(max_y - min_y, 0.1)
    scale = min(sx, sy)

    def project(p: tuple[float, float]) -> tuple[int, int]:
        px = x0 + 15 + int((p[0] - min_x) * scale)
        py = y1 - 15 - int((p[1] - min_y) * scale)
        return px, py

    if len(trail) >= 2:
        pts = np.array([project(p) for p in trail], dtype=np.int32)
        cv2.polylines(frame, [pts], isClosed=False, color=(0, 220, 255), thickness=2)
    cv2.circle(frame, project(start_xy), 5, (80, 220, 80), -1)
    cv2.circle(frame, project(goal_xy), 6, (80, 80, 255), -1)
    if trail:
        cv2.circle(frame, project(trail[-1]), 6, (255, 220, 0), -1)
    cv2.putText(frame, "top-down trail", (x0 + 12, y0 + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1)


def _render_replay(
    *,
    world: str,
    policy_path: str,
    output: Path,
    snapshots: list[dict[str, Any]],
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    width: int,
    height: int,
    fps: float,
) -> int:
    import cv2
    import mujoco

    from drivers.sim.mujoco_driver_module import MujocoDriverModule

    if not snapshots:
        raise RuntimeError("No navigation snapshots captured; nothing to render")

    driver = MujocoDriverModule(
        world=world,
        render=False,
        enable_camera=False,
        drive_mode="policy",
        policy_path=policy_path,
    )
    driver.setup()
    engine = driver._engine
    renderer = None
    writer = None
    try:
        if engine is None or engine.model is None or engine.data is None:
            raise RuntimeError("Replay MuJoCo engine is not available")
        engine.model.vis.global_.offwidth = max(int(engine.model.vis.global_.offwidth), width)
        engine.model.vis.global_.offheight = max(int(engine.model.vis.global_.offheight), height)
        renderer = mujoco.Renderer(engine.model, height=height, width=width)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.distance = 3.8
        camera.elevation = -55.0
        camera.azimuth = 135.0

        output.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(output),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (width, height),
        )
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open video writer: {output}")

        replay_trail: list[tuple[float, float]] = []
        for snap in snapshots:
            qpos = np.asarray(snap["qpos"], dtype=np.float64)
            n_qpos = min(qpos.size, engine.data.qpos.size)
            engine.data.qpos[:n_qpos] = qpos[:n_qpos]
            engine.data.qvel[:] = 0.0
            mujoco.mj_forward(engine.model, engine.data)

            x = float(snap["x"])
            y = float(snap["y"])
            replay_trail.append((x, y))
            camera.lookat[0] = (x + goal_xy[0]) * 0.5
            camera.lookat[1] = (y + goal_xy[1]) * 0.5
            camera.lookat[2] = 0.45
            renderer.update_scene(engine.data, camera)
            frame = renderer.render().copy()
            cmd = snap["cmd"]
            _put_text(frame, "LingTu policy navigation sim", 36, (255, 255, 255))
            _put_text(
                frame,
                f"state={snap['state']}  t={snap['t']:.1f}s  dist={snap['dist']:.3f}m",
                70,
                (0, 255, 255),
            )
            _put_text(
                frame,
                f"robot=({x:.2f},{y:.2f})  goal=({goal_xy[0]:.2f},{goal_xy[1]:.2f})  cmd=({cmd[0]:.2f},{cmd[1]:.2f})",
                104,
                (230, 230, 230),
            )
            _draw_inset(
                frame,
                start_xy=start_xy,
                goal_xy=goal_xy,
                trail=replay_trail,
            )
            writer.write(frame[:, :, ::-1])
        return len(snapshots)
    finally:
        if writer is not None:
            writer.release()
        if renderer is not None:
            renderer.close()
        driver.stop()


def record_full_stack_nav(
    *,
    world: str,
    policy_path: str,
    output: Path,
    goal_distance: float,
    duration: float,
    fps: float,
    width: int,
    height: int,
    waypoint_threshold: float,
    final_waypoint_threshold: float,
    downsample_dist: float,
    path_goal_tolerance: float,
    path_min_speed: float,
    path_max_speed: float,
    safe_goal_tolerance: float,
    nav_max_angular_z: float,
    success_settle: float,
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
        max_angular_vel=nav_max_angular_z,
        waypoint_threshold=waypoint_threshold,
        final_waypoint_threshold=final_waypoint_threshold,
        downsample_dist=downsample_dist,
        path_follower_goal_tolerance=path_goal_tolerance,
        path_follower_min_speed=path_min_speed,
        path_follower_max_speed=path_max_speed,
        safe_goal_tolerance=safe_goal_tolerance,
        run_startup_checks=False,
    ).build()

    driver = system.get_module("MujocoDriverModule")
    ogm = system.get_module("OccupancyGridModule")
    nav = system.get_module("NavigationModule")
    mux = system.get_module("CmdVelMux")

    seen = {"costmap": 0, "waypoints": 0, "mux_cmd": 0}
    odom: list[tuple[float, float, float]] = []
    last_mux: list[tuple[float, float]] = []
    trail: list[tuple[float, float]] = []
    global_path_last: list[float] | None = None

    ogm.costmap._add_callback(lambda _: seen.__setitem__("costmap", seen["costmap"] + 1))
    nav.waypoint._add_callback(lambda _: seen.__setitem__("waypoints", seen["waypoints"] + 1))

    def _record_global_path(path: Any) -> None:
        nonlocal global_path_last
        points = []
        for p in path:
            try:
                points.append([float(p[0]), float(p[1]), float(p[2] if len(p) > 2 else 0.0)])
            except Exception:
                continue
        if points:
            global_path_last = points[-1]

    nav.global_path._add_callback(_record_global_path)
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

    snapshots: list[dict[str, Any]] = []
    result: dict[str, Any] | None = None
    system.start()
    try:
        warm_deadline = time.time() + min(10.0, max(3.0, duration * 0.25))
        while time.time() < warm_deadline and (seen["costmap"] == 0 or not odom):
            time.sleep(0.1)
        if not odom:
            raise RuntimeError("No odometry from sim_mujoco policy mode")
        engine = driver._engine
        if engine is None or engine.model is None or engine.data is None:
            raise RuntimeError("MuJoCo engine is not available")

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
        next_frame = time.time()
        frame_period = 1.0 / max(fps, 1.0)
        started_at = time.time()
        success_at: float | None = None
        dist_at_success: float | None = None
        success_pose: tuple[float, float, float] | None = None

        while time.time() < deadline:
            now = time.time()
            if odom:
                x, y, z = odom[-1]
                if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) > 0.02:
                    trail.append((x, y))
                dist_to_goal = math.hypot(goal_x - x, goal_y - y)
            else:
                x, y, z = start
                dist_to_goal = math.hypot(goal_x - x, goal_y - y)

            if now >= next_frame:
                state = str(getattr(nav, "_state", ""))
                cmd = last_mux[-1] if last_mux else (0.0, 0.0)
                snapshots.append(
                    {
                        "t": time.time() - started_at,
                        "x": float(x),
                        "y": float(y),
                        "z": float(z),
                        "dist": float(dist_to_goal),
                        "state": state,
                        "cmd": [float(cmd[0]), float(cmd[1])],
                        "qpos": engine.data.qpos.copy().tolist(),
                    }
                )
                next_frame += frame_period

            if str(getattr(nav, "_state", "")) == "SUCCESS":
                if success_at is None:
                    success_at = time.time()
                    dist_at_success = dist_to_goal
                    success_pose = odom[-1] if odom else None
                if time.time() - success_at >= success_settle:
                    break
            time.sleep(0.02)

        elapsed = time.time() - started_at
        end = odom[-1] if odom else start
        result = {
            "output": str(output),
            "frames": len(snapshots),
            "fps": fps,
            "elapsed_s": elapsed,
            "world": world,
            "policy": _load_policy_metadata(policy_path),
            "start": [float(v) for v in start[:3]],
            "goal": [float(goal_x), float(goal_y), 0.0],
            "end": [float(v) for v in end[:3]],
            "success_seen": success_at is not None,
            "dist_at_success_m": dist_at_success,
            "success_end": [float(v) for v in success_pose[:3]] if success_pose else None,
            "dist_to_goal_m": math.hypot(goal_x - end[0], goal_y - end[1]),
            "nav_state": str(getattr(nav, "_state", "")),
            "seen": seen,
            "global_path_last": global_path_last,
        }
    finally:
        system.stop()
    if result is None:
        raise RuntimeError("Navigation run did not produce a result")
    rendered_frames = _render_replay(
        world=world,
        policy_path=policy_path,
        output=output,
        snapshots=snapshots,
        start_xy=(float(result["start"][0]), float(result["start"][1])),
        goal_xy=(float(result["goal"][0]), float(result["goal"][1])),
        width=width,
        height=height,
        fps=fps,
    )
    result["frames"] = rendered_frames
    return result


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="open_field")
    parser.add_argument("--policy", required=True, help="Explicit ONNX policy path")
    parser.add_argument("--output", default="artifacts/policy_nav_video.mp4")
    parser.add_argument("--summary-out", default="")
    parser.add_argument("--duration", type=float, default=18.0)
    parser.add_argument("--goal-distance", type=float, default=1.0)
    parser.add_argument("--fps", type=float, default=25.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--nav-max-angular-z", type=float, default=0.2)
    parser.add_argument("--nav-waypoint-threshold", type=float, default=0.25)
    parser.add_argument("--nav-final-waypoint-threshold", type=float, default=0.06)
    parser.add_argument("--nav-downsample-dist", type=float, default=0.25)
    parser.add_argument("--nav-path-goal-tolerance", type=float, default=0.08)
    parser.add_argument("--nav-path-min-speed", type=float, default=0.05)
    parser.add_argument("--nav-path-max-speed", type=float, default=0.25)
    parser.add_argument("--nav-safe-goal-tolerance", type=float, default=0.0)
    parser.add_argument("--success-settle", type=float, default=0.8)
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    result = record_full_stack_nav(
        world=args.world,
        policy_path=args.policy,
        output=Path(args.output),
        goal_distance=args.goal_distance,
        duration=args.duration,
        fps=args.fps,
        width=args.width,
        height=args.height,
        waypoint_threshold=args.nav_waypoint_threshold,
        final_waypoint_threshold=args.nav_final_waypoint_threshold,
        downsample_dist=args.nav_downsample_dist,
        path_goal_tolerance=args.nav_path_goal_tolerance,
        path_min_speed=args.nav_path_min_speed,
        path_max_speed=args.nav_path_max_speed,
        safe_goal_tolerance=args.nav_safe_goal_tolerance,
        nav_max_angular_z=args.nav_max_angular_z,
        success_settle=args.success_settle,
    )
    output = json.dumps(result, indent=2, sort_keys=True)
    print(output)
    if args.summary_out:
        Path(args.summary_out).write_text(output + "\n", encoding="utf-8")
    return 0 if result.get("success_seen") and Path(result["output"]).exists() else 1


if __name__ == "__main__":
    raise SystemExit(main())
