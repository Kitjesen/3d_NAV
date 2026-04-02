#!/usr/bin/env python3
"""Person-following simulation — MuJoCo verification platform.

Runs a quadruped robot following a scripted moving person in a
residential scene. Validates perception→tracking→control pipeline.

Usage:
    # Phase 1: Ground truth + PD (basic verification)
    python sim/scripts/run_person_following.py

    # Full perception pipeline
    python sim/scripts/run_person_following.py --perception camera_yolo

    # Pure Pursuit controller
    python sim/scripts/run_person_following.py --controller pure_pursuit

    # Custom trajectory
    python sim/scripts/run_person_following.py --trajectory l_turn --speed 0.6

    # Headless (no viewer)
    python sim/scripts/run_person_following.py --headless --duration 30
"""
import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

# Ensure project root is on path
_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT / "sim"))

import mujoco
import mujoco.viewer

from following.interfaces import FollowCommand, PersonState
from following.person.person_model import PersonController, get_person_xml
from following.person.trajectory import (
    StraightWalk, LTurnWalk, UTurnWalk, StopAndGoWalk,
    CircleWalk, Figure8Walk, WaypointWalk, TRAJECTORIES,
)
from following.perception.ground_truth import GroundTruthPerception
from following.controller.pd_follower import PDFollower
from following.controller.pure_pursuit import PurePursuitFollower
from following.behavior import FollowingBehavior, BehaviorConfig


# ---------------------------------------------------------------------------
# RL policy runner (adapted from brainstem walk_ref.py)
# ---------------------------------------------------------------------------

_BRAINSTEM_SIM = Path(__file__).resolve().parent.parent.parent.parent / "brainstem" / "sim"

# Joint indices (Dart order → MuJoCo qpos/qvel indices)
DOF_IDS = [7, 8, 9, 11, 12, 13, 15, 16, 17, 19, 20, 21, 10, 14, 18, 22]
DOF_VEL = [6, 7, 8, 10, 11, 12, 14, 15, 16, 18, 19, 20, 9, 13, 17, 21]

DEFAULT_ANGLES = np.array([
    -0.1, -0.8, 1.8,   # FR hip, thigh, calf
     0.1,  0.8, -1.8,  # FL
     0.1,  0.8, -1.8,  # RR
    -0.1, -0.8, 1.8,   # RL
     0.0,  0.0, 0.0, 0.0,  # 4 wheels
], dtype=np.float64)

ACTION_SCALE = np.array([
    0.125, 0.25, 0.25,  # FR
    0.125, 0.25, 0.25,  # FL
    0.125, 0.25, 0.25,  # RR
    0.125, 0.25, 0.25,  # RL
    5.0, 5.0, 5.0, 5.0, # wheels
], dtype=np.float64)

KP = np.array([70, 100, 120] * 4, dtype=np.float64)
KD = np.array([15, 15, 20] * 4, dtype=np.float64)


def get_obs(data, vx, vy, dyaw, last_action):
    """Build 57-dim observation vector (matches brainstem walk_ref.py)."""
    from scipy.spatial.transform import Rotation as R

    q = data.qpos[DOF_IDS].astype(np.float64) - DEFAULT_ANGLES
    dq = data.qvel[DOF_VEL].astype(np.float64) * 0.05
    q[-4:] = 0.0  # zero wheel position

    imu_quat = data.sensor('orientation').data[[1, 2, 3, 0]].astype(np.float64)
    r_imu = R.from_quat(imu_quat)
    proj = r_imu.apply(np.array([0.0, 0.0, -1.0]), inverse=True)

    gyro_local = data.sensor('angular-velocity').data.astype(np.float64)
    base_quat = data.qpos[3:7][[1, 2, 3, 0]].astype(np.float64)
    r_base = R.from_quat(base_quat)
    gyro_world = r_imu.apply(gyro_local)
    gyro = r_base.apply(gyro_world, inverse=True) * 0.25

    vel_cmd = np.array([vx, vy, dyaw], dtype=np.float64)
    obs = np.concatenate([gyro, proj, vel_cmd, q, dq, last_action])
    return obs.astype(np.float32)


def pd_control(target_q, q, kp, dq, kd):
    """PD torque computation for 12 leg joints."""
    return kp * (target_q - q) - kd * dq


# ---------------------------------------------------------------------------
# Scene builder: merge robot + room + person
# ---------------------------------------------------------------------------

def build_scene_xml(
    robot_xml: Path,
    scene_xml: Path,
    person_start: list[float],
) -> str:
    """Merge robot XML + scene XML + person mocap body into one XML."""
    robot_text = robot_xml.read_text(encoding="utf-8")
    scene_text = scene_xml.read_text(encoding="utf-8")

    # Extract scene's <worldbody> content (between tags)
    import re
    wb_match = re.search(
        r"<worldbody>(.*?)</worldbody>", scene_text, re.DOTALL
    )
    scene_bodies = wb_match.group(1) if wb_match else ""

    # Extract scene's <asset> content
    asset_match = re.search(
        r"<asset>(.*?)</asset>", scene_text, re.DOTALL
    )
    scene_assets = asset_match.group(1) if asset_match else ""

    # Person XML
    person_xml = get_person_xml(person_start)

    # Inject scene assets into robot's asset block (or create one)
    if "<asset>" in robot_text:
        robot_text = robot_text.replace(
            "</asset>", scene_assets + "\n  </asset>", 1
        )
    else:
        robot_text = robot_text.replace(
            "<worldbody>",
            f"<asset>\n{scene_assets}\n  </asset>\n\n  <worldbody>",
            1,
        )

    # Inject scene bodies + person into robot's worldbody
    robot_text = robot_text.replace(
        "</worldbody>",
        f"\n    <!-- === SCENE === -->\n{scene_bodies}\n"
        f"\n    <!-- === PERSON === -->\n    {person_xml}\n"
        f"  </worldbody>",
        1,
    )

    return robot_text


# ---------------------------------------------------------------------------
# Main simulation
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Person-following simulation")
    parser.add_argument("--trajectory", default="straight",
                        choices=list(TRAJECTORIES.keys()),
                        help="Person trajectory type")
    parser.add_argument("--speed", type=float, default=0.8,
                        help="Person walking speed (m/s)")
    parser.add_argument("--perception", default="ground_truth",
                        choices=["ground_truth"],
                        help="Perception pipeline")
    parser.add_argument("--controller", default="pd",
                        choices=["pd", "pure_pursuit"],
                        help="Following controller")
    parser.add_argument("--target-distance", type=float, default=1.5,
                        help="Desired following distance (m)")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Simulation duration (s)")
    parser.add_argument("--headless", action="store_true",
                        help="No GUI viewer")
    parser.add_argument("--scene", default=None,
                        help="Scene XML path (default: go2_room_nova)")
    parser.add_argument("--policy", default=None,
                        help="ONNX policy path")
    parser.add_argument("--render-video", default=None,
                        help="Save video to path")
    args = parser.parse_args()

    # Paths
    sim_dir = Path(__file__).resolve().parent.parent
    robot_xml = sim_dir / "robots" / "nova_dog" / "robot_with_camera.xml"
    scene_xml = Path(args.scene) if args.scene else sim_dir / "scenes" / "go2_room_nova.xml"

    policy_path = args.policy
    if policy_path is None:
        candidates = [
            _BRAINSTEM_SIM / "model" / "policy_v9_800.onnx",
            _ROOT.parent / "brainstem" / "sim" / "model" / "policy_v9_800.onnx",
        ]
        for c in candidates:
            if c.exists():
                policy_path = str(c)
                break
    if policy_path is None:
        print("ERROR: No ONNX policy found. Use --policy <path>")
        sys.exit(1)

    # Robot start position (in the open hallway)
    robot_start = [10.0, 3.0, 0.45]
    # Person starts ahead of robot
    person_start = [10.0, 6.0, 0.0]

    print(f"Scene:      {scene_xml.name}")
    print(f"Robot:      {robot_xml.name}")
    print(f"Policy:     {Path(policy_path).name}")
    print(f"Trajectory: {args.trajectory} @ {args.speed} m/s")
    print(f"Perception: {args.perception}")
    print(f"Controller: {args.controller} (target_dist={args.target_distance}m)")
    print(f"Duration:   {args.duration}s")

    # ── Build merged scene XML ──
    merged_xml = build_scene_xml(robot_xml, scene_xml, person_start)

    # Write to temp file for MuJoCo
    import tempfile
    tmp = tempfile.NamedTemporaryFile(
        suffix=".xml", delete=False, mode="w", encoding="utf-8",
        dir=str(sim_dir / "robots" / "nova_dog"),  # same dir as meshes
    )
    tmp.write(merged_xml)
    tmp.close()

    try:
        model = mujoco.MjModel.from_xml_path(tmp.name)
        # Set offscreen buffer for HD video rendering
        model.vis.global_.offwidth = 1920
        model.vis.global_.offheight = 1080
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"ERROR loading model: {e}")
        os.unlink(tmp.name)
        sys.exit(1)

    # ── Load ONNX policy ──
    import onnxruntime as ort
    sess = ort.InferenceSession(policy_path)
    input_name = sess.get_inputs()[0].name
    obs_dim = 57
    history_size = sess.get_inputs()[0].shape[1] // obs_dim
    print(f"Policy:     obs={obs_dim} x history={history_size} = {obs_dim * history_size}")

    from collections import deque
    obs_history = deque(maxlen=history_size)

    # ── Create trajectory ──
    traj_cls = TRAJECTORIES[args.trajectory]
    if args.trajectory == "straight":
        trajectory = traj_cls(start=tuple(person_start[:2]), speed=args.speed)
    elif args.trajectory == "l_turn":
        trajectory = traj_cls(start=tuple(person_start[:2]), speed=args.speed)
    elif args.trajectory == "u_turn":
        trajectory = traj_cls(start=tuple(person_start[:2]), speed=args.speed)
    elif args.trajectory == "stop_and_go":
        trajectory = traj_cls(start=tuple(person_start[:2]), speed=args.speed)
    elif args.trajectory == "circle":
        trajectory = CircleWalk(center=tuple(person_start[:2]), speed=args.speed)
    elif args.trajectory == "figure8":
        trajectory = Figure8Walk(center=tuple(person_start[:2]), speed=args.speed)
    else:
        trajectory = traj_cls(
            waypoints=[tuple(person_start[:2]), (15, 6), (15, 10)],
            speed=args.speed,
        )

    # ── Create components ──
    person_ctrl = PersonController(model, trajectory)
    perception = GroundTruthPerception()
    if args.controller == "pure_pursuit":
        controller = PurePursuitFollower(target_distance=args.target_distance)
    else:
        controller = PDFollower(target_distance=args.target_distance)
    behavior = FollowingBehavior(
        config=BehaviorConfig(),
        controller=controller,
    )

    # ── Initialize ──
    mujoco.mj_resetData(model, data)
    data.qpos[:3] = robot_start
    data.qpos[3:7] = [1, 0, 0, 0]  # identity quaternion
    data.qpos[DOF_IDS] = DEFAULT_ANGLES
    mujoco.mj_forward(model, data)
    person_state = person_ctrl.reset(data)

    # ── Simulation state ──
    action = np.zeros(16, dtype=np.float64)
    last_action = np.zeros(16, dtype=np.float64)
    target_q = DEFAULT_ANGLES.copy()
    cmd = FollowCommand()
    decimation = 4
    dt_sim = model.opt.timestep
    dt_ctrl = dt_sim * decimation
    sim_steps = int(args.duration / dt_sim)
    count = 0
    warmup_steps = int(1.0 / dt_sim)  # 1 second warmup (stand)

    print(f"\nStarting simulation ({sim_steps} steps, dt={dt_sim*1000:.1f}ms)...")

    # ── Video recording (720p, tracking overhead camera) ──
    frames = [] if args.render_video else None
    renderer = None
    vid_cam = None
    if args.render_video:
        renderer = mujoco.Renderer(model, height=720, width=1280)
        # Create a tracking camera that follows the midpoint of robot & person
        vid_cam = mujoco.MjvCamera()
        vid_cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        vid_cam.distance = 8.0      # close overhead
        vid_cam.elevation = -75.0   # near top-down
        vid_cam.azimuth = 90.0
        # Hide ceiling for overhead view
        ceiling_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ceiling")
        if ceiling_id >= 0:
            model.geom_rgba[ceiling_id] = [0, 0, 0, 0]  # transparent

    # ── Main loop ──
    t_start = time.monotonic()

    def step_fn():
        nonlocal count, action, last_action, target_q, cmd, person_state

        for _ in range(decimation):
            # PD control every physics step
            q = data.qpos[DOF_IDS]
            dq = data.qvel[DOF_VEL]
            tau_leg = pd_control(target_q[:12], q[:12], KP, dq[:12], KD)
            tau_wheel = 1.0 * (target_q[12:] - dq[12:])
            tau = np.clip(
                np.concatenate([tau_leg, tau_wheel]), -120, 120
            )
            data.ctrl[:] = tau
            mujoco.mj_step(model, data)

        count += decimation

        # RL policy inference at control rate
        if count > warmup_steps:
            # Get following command
            robot_pos = data.qpos[:3].copy()
            robot_quat = data.qpos[3:7].copy()
            from scipy.spatial.transform import Rotation as R
            robot_yaw = R.from_quat(
                robot_quat[[1, 2, 3, 0]]
            ).as_euler('xyz')[2]

            # Advance person
            person_state = person_ctrl.step(data, dt_ctrl)

            # Perceive
            target = perception.update(None, person_state, data.time)

            # Behavior FSM: FOLLOW / SEARCH / EXPLORE / WAIT / RECOVER
            cmd = behavior.update(
                robot_pos[:2], robot_yaw,
                target, person_state, dt_ctrl,
            )
        else:
            cmd = FollowCommand()

        # Build observation with follow command as velocity
        obs = get_obs(data, cmd.vx, cmd.vy, cmd.dyaw, last_action)
        obs_history.append(obs.copy())
        while len(obs_history) < history_size:
            obs_history.appendleft(obs.copy())

        obs_flat = np.concatenate(list(obs_history)).reshape(1, -1)
        raw_action = sess.run(None, {input_name: obs_flat})[0].squeeze()
        action[:] = raw_action

        if count > warmup_steps:
            scaled = action * ACTION_SCALE
            target_q = scaled + DEFAULT_ANGLES
            last_action = action.copy()
        else:
            target_q = DEFAULT_ANGLES.copy()

        # Record video frame (tracking overhead camera)
        if frames is not None and count % (decimation * 2) == 0:
            # Track midpoint between robot and person
            rx, ry = data.qpos[0], data.qpos[1]
            px, py = person_state.position[0], person_state.position[1]
            vid_cam.lookat[0] = (rx + px) / 2
            vid_cam.lookat[1] = (ry + py) / 2
            vid_cam.lookat[2] = 0.5
            renderer.update_scene(data, vid_cam)
            frame_rgb = renderer.render().copy()
            # Draw HUD text on frame
            import cv2
            rx_f, ry_f = float(rx), float(ry)
            px_f, py_f = float(px), float(py)
            dist = math.hypot(rx_f - px_f, ry_f - py_f)
            cv2.putText(frame_rgb, f"t={count*dt_sim:.1f}s  dist={dist:.2f}m  cmd=({cmd.vx:.2f},{cmd.vy:.2f},{cmd.dyaw:.2f})",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(frame_rgb, f"robot=({rx_f:.1f},{ry_f:.1f})  person=({px_f:.1f},{py_f:.1f})",
                        (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            frames.append(frame_rgb)

        # Status print
        if count % (int(1.0 / dt_sim)) == 0:
            rx, ry = data.qpos[0], data.qpos[1]
            px, py = person_state.position[0], person_state.position[1]
            dist = math.hypot(rx - px, ry - py)
            elapsed = time.monotonic() - t_start
            rtf = (count * dt_sim) / max(elapsed, 0.001)
            print(
                f"  t={count * dt_sim:5.1f}s  "
                f"robot=({rx:.1f},{ry:.1f})  "
                f"person=({px:.1f},{py:.1f})  "
                f"dist={dist:.2f}m  "
                f"cmd=({cmd.vx:.2f},{cmd.vy:.2f},{cmd.dyaw:.2f})  "
                f"[{behavior.state.value}]  "
                f"RTF={rtf:.1f}x"
            )

    if args.headless:
        for _ in range(sim_steps // decimation):
            step_fn()
            if trajectory.is_complete and count > warmup_steps + int(3.0 / dt_sim):
                break
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and count < sim_steps:
                step_fn()
                viewer.sync()
                if trajectory.is_complete and count > warmup_steps + int(3.0 / dt_sim):
                    break

    # ── Save video ──
    if frames and args.render_video:
        import cv2
        out_path = args.render_video
        h, w = frames[0].shape[:2]
        writer = cv2.VideoWriter(
            out_path, cv2.VideoWriter_fourcc(*"mp4v"), 25, (w, h)
        )
        for f in frames:
            writer.write(f[:, :, ::-1])  # RGB → BGR
        writer.release()
        print(f"\nVideo saved: {out_path} ({len(frames)} frames)")

    # ── Final report ──
    rx, ry = data.qpos[0], data.qpos[1]
    px, py = person_state.position[0], person_state.position[1]
    dist = math.hypot(rx - px, ry - py)
    print(f"\n=== Simulation Complete ===")
    print(f"Duration: {count * dt_sim:.1f}s")
    print(f"Robot final: ({rx:.2f}, {ry:.2f})")
    print(f"Person final: ({px:.2f}, {py:.2f})")
    print(f"Final distance: {dist:.2f}m (target: {args.target_distance}m)")
    print(f"Trajectory complete: {trajectory.is_complete}")

    os.unlink(tmp.name)


if __name__ == "__main__":
    main()
