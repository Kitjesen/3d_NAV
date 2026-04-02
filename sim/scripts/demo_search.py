#!/usr/bin/env python3
"""Demo: person following + search behavior with dual-view video.

Renders side-by-side:
  LEFT:  overhead tracking view (robot + person + room)
  RIGHT: robot first-person camera (what the robot sees)

The person walks, then disappears behind furniture.
The robot follows, loses the person, enters SEARCH mode.

All behavior decisions printed to CLI in real-time.

Usage:
    cd lingtu
    python sim/scripts/demo_search.py
    python sim/scripts/demo_search.py --render-video sim/output/demo_search.mp4
"""
import argparse
import math
import os
import sys
import time
from pathlib import Path
from collections import deque

import numpy as np
import cv2

_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT / "sim"))

import mujoco

from following.interfaces import FollowCommand, PersonState, PerceivedTarget
from following.person.person_model import PersonController, get_person_xml
from following.person.trajectory import WaypointWalk
from following.perception.ground_truth import GroundTruthPerception
from following.controller.pure_pursuit import PurePursuitFollower
from following.behavior import FollowingBehavior, BehaviorConfig

# Reuse RL policy code
from scripts.run_person_following import (
    DOF_IDS, DOF_VEL, DEFAULT_ANGLES, ACTION_SCALE, KP, KD,
    get_obs, pd_control, build_scene_xml,
)

_BRAINSTEM_SIM = _ROOT.parent / "brainstem" / "sim"


class OcclusionPerception:
    """Ground truth + simulated occlusion when person behind wall/furniture.

    Person is invisible when:
      - Behind the bedroom wall (y > 10.0) — wall blocks line of sight
      - Robot is still in the hallway (y < 10.0)
    Once robot also enters the bedroom area, person becomes visible again.
    """

    def __init__(self):
        self._gt = GroundTruthPerception()

    def update(self, engine, person_gt, timestamp):
        if person_gt is None:
            return None
        px, py = person_gt.position[0], person_gt.position[1]
        # Get robot position from engine
        try:
            rx, ry = float(engine._data.qpos[0]), float(engine._data.qpos[1])
        except Exception:
            rx, ry = 0.0, 0.0
        # Person is behind bedroom wall AND robot is still in hallway
        if py > 10.0 and ry < 9.5:
            return None  # occluded by wall!
        return self._gt.update(engine, person_gt, timestamp)


def find_policy():
    for c in [
        _BRAINSTEM_SIM / "model" / "policy_v9_800.onnx",
        _ROOT.parent / "brainstem" / "sim" / "model" / "policy_v9_800.onnx",
    ]:
        if c.exists():
            return str(c)
    raise FileNotFoundError("No ONNX policy found")


def main():
    parser = argparse.ArgumentParser(description="Search behavior demo")
    parser.add_argument("--render-video", default=None)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    sim_dir = _ROOT / "sim"
    robot_xml = sim_dir / "robots" / "nova_dog" / "robot_with_camera.xml"
    scene_xml = sim_dir / "scenes" / "go2_room_nova.xml"
    policy_path = find_policy()

    # Person trajectory: walk through hallway, go through a door, disappear
    person_start = [10.0, 5.0, 0.0]
    trajectory = WaypointWalk(
        waypoints=[
            (10.0, 5.0),   # start in hallway
            (10.0, 7.0),   # walk forward (robot follows)
            (10.0, 9.5),   # approach bedroom wall (robot close)
            (10.0, 11.0),  # cross through door → OCCLUDED
            (8.0, 12.0),   # inside master bedroom → STILL OCCLUDED
            (5.0, 12.0),   # deep in bedroom → robot enters SEARCH
            (3.5, 12.0),   # stay in bedroom
        ],
        speed=0.3,  # slow enough for robot to follow before occlusion
    )

    robot_start = [10.0, 3.0, 0.45]

    print("=" * 70)
    print("  DEMO: Person Following + Search Behavior")
    print("=" * 70)
    print(f"  Person: hallway → door → bedroom (with occlusion)")
    print(f"  Robot:  follow → SEARCH when person disappears")
    print(f"  Video:  dual view (overhead + robot POV)")
    print("=" * 70)

    # Build scene
    merged_xml = build_scene_xml(robot_xml, scene_xml, person_start)
    import tempfile
    tmp = tempfile.NamedTemporaryFile(
        suffix=".xml", delete=False, mode="w", encoding="utf-8",
        dir=str(sim_dir / "robots" / "nova_dog"),
    )
    tmp.write(merged_xml)
    tmp.close()

    model = mujoco.MjModel.from_xml_path(tmp.name)
    model.vis.global_.offwidth = 1920
    model.vis.global_.offheight = 1080
    data = mujoco.MjData(model)

    # Load policy
    import onnxruntime as ort
    sess = ort.InferenceSession(policy_path)
    input_name = sess.get_inputs()[0].name
    history_size = sess.get_inputs()[0].shape[1] // 57

    # Create components
    person_ctrl = PersonController(model, trajectory)
    perception = OcclusionPerception()
    controller = PurePursuitFollower(target_distance=1.5)
    behavior = FollowingBehavior(
        config=BehaviorConfig(search_timeout_s=3.0, explore_timeout_s=12.0),
        controller=controller,
    )

    # Renderers
    overhead_renderer = mujoco.Renderer(model, height=540, width=960)
    pov_renderer = mujoco.Renderer(model, height=540, width=960)
    vid_cam = mujoco.MjvCamera()
    vid_cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    vid_cam.distance = 8.0
    vid_cam.elevation = -75.0
    vid_cam.azimuth = 90.0

    # Hide ceiling
    ceil_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ceiling")
    if ceil_id >= 0:
        model.geom_rgba[ceil_id] = [0, 0, 0, 0]

    # Initialize
    mujoco.mj_resetData(model, data)
    data.qpos[:3] = robot_start
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[DOF_IDS] = DEFAULT_ANGLES
    mujoco.mj_forward(model, data)
    person_state = person_ctrl.reset(data)

    obs_history = deque(maxlen=history_size)
    action = np.zeros(16, dtype=np.float64)
    last_action = np.zeros(16, dtype=np.float64)
    target_q = DEFAULT_ANGLES.copy()
    cmd = FollowCommand()
    decimation = 4
    dt_sim = model.opt.timestep
    dt_ctrl = dt_sim * decimation
    sim_steps = int(args.duration / dt_sim)
    warmup_steps = int(1.0 / dt_sim)
    count = 0
    frames = [] if args.render_video else None
    prev_state = "follow"

    print(f"\n{'Time':>6} {'State':<10} {'Dist':>6} {'Cmd':>24} {'Info'}")
    print("-" * 70)

    from scipy.spatial.transform import Rotation as R

    for _ in range(sim_steps // decimation):
        # PD control
        for _ in range(decimation):
            q = data.qpos[DOF_IDS]
            dq = data.qvel[DOF_VEL]
            tau_leg = pd_control(target_q[:12], q[:12], KP, dq[:12], KD)
            tau_wheel = 1.0 * (target_q[12:] - dq[12:])
            tau = np.clip(np.concatenate([tau_leg, tau_wheel]), -120, 120)
            data.ctrl[:] = tau
            mujoco.mj_step(model, data)
        count += decimation

        if count > warmup_steps:
            robot_pos = data.qpos[:3].copy()
            robot_yaw = R.from_quat(data.qpos[3:7][[1, 2, 3, 0]]).as_euler('xyz')[2]

            person_state = person_ctrl.step(data, dt_ctrl)
            # Pass a simple engine-like object so OcclusionPerception can read robot pos
            class _Eng:
                _data = data
            target = perception.update(_Eng(), person_state, data.time)

            cmd = behavior.update(
                robot_pos[:2], robot_yaw,
                target, person_state, dt_ctrl,
            )
        else:
            cmd = FollowCommand()

        # RL policy
        obs = get_obs(data, cmd.vx, cmd.vy, cmd.dyaw, last_action)
        obs_history.append(obs.copy())
        while len(obs_history) < history_size:
            obs_history.appendleft(obs.copy())
        obs_flat = np.concatenate(list(obs_history)).reshape(1, -1)
        raw_action = sess.run(None, {input_name: obs_flat})[0].squeeze()
        action[:] = raw_action
        if count > warmup_steps:
            target_q = action * ACTION_SCALE + DEFAULT_ANGLES
            last_action = action.copy()
        else:
            target_q = DEFAULT_ANGLES.copy()

        # CLI output on state change or every 1s
        cur_state = behavior.state.value
        rx, ry = float(data.qpos[0]), float(data.qpos[1])
        px, py = float(person_state.position[0]), float(person_state.position[1])
        dist = math.hypot(rx - px, ry - py)
        t_s = count * dt_sim

        state_changed = cur_state != prev_state
        if state_changed or count % int(1.0 / dt_sim) == 0:
            detected = "visible" if (target is not None if count > warmup_steps else False) else "HIDDEN"
            marker = " <<<" if state_changed else ""
            print(
                f"{t_s:6.1f} {cur_state:<10} {dist:6.2f}  "
                f"({cmd.vx:+.2f},{cmd.vy:+.2f},{cmd.dyaw:+.2f})  "
                f"{detected}{marker}"
            )
            prev_state = cur_state

        # Render dual-view frame
        if frames is not None and count % (decimation * 2) == 0:
            # Overhead view
            vid_cam.lookat[0] = (rx + px) / 2
            vid_cam.lookat[1] = (ry + py) / 2
            vid_cam.lookat[2] = 0.5
            overhead_renderer.update_scene(data, vid_cam)
            overhead = overhead_renderer.render().copy()

            # Robot POV
            pov_renderer.update_scene(data, camera="front_camera")
            pov = pov_renderer.render().copy()

            # HUD on overhead
            state_color = {
                "follow": (0, 255, 0), "wait": (255, 255, 0),
                "search": (0, 165, 255), "explore": (255, 0, 255),
                "recover": (0, 0, 255),
            }.get(cur_state, (200, 200, 200))
            cv2.putText(overhead, f"[{cur_state.upper()}] dist={dist:.2f}m",
                        (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2)
            cv2.putText(overhead, f"t={t_s:.1f}s  cmd=({cmd.vx:.2f},{cmd.vy:.2f},{cmd.dyaw:.2f})",
                        (15, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            # HUD on POV
            cv2.putText(pov, "ROBOT POV", (15, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            detected_str = "TARGET: VISIBLE" if (target is not None if count > warmup_steps else False) else "TARGET: LOST"
            det_color = (0, 255, 0) if "VISIBLE" in detected_str else (0, 0, 255)
            cv2.putText(pov, detected_str, (15, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, det_color, 2)

            # Side-by-side
            combined = np.hstack([overhead, pov])
            frames.append(combined)

        if trajectory.is_complete and count > warmup_steps + int(5.0 / dt_sim):
            break

    # Save video
    if frames and args.render_video:
        out_path = args.render_video
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        h, w = frames[0].shape[:2]
        writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), 25, (w, h))
        for f in frames:
            writer.write(f[:, :, ::-1])
        writer.release()
        print(f"\nVideo saved: {out_path} ({len(frames)} frames, {w}x{h})")

    print(f"\n{'='*70}")
    print(f"  Final: robot=({rx:.1f},{ry:.1f}) person=({px:.1f},{py:.1f}) dist={dist:.2f}m")
    print(f"  State: {behavior.state.value}")
    print(f"{'='*70}")
    os.unlink(tmp.name)


if __name__ == "__main__":
    main()
