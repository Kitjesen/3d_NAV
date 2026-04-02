#!/usr/bin/env python3
"""Benchmark: PD vs Pure Pursuit across multiple trajectories.

Runs both controllers on each trajectory, records metrics,
and generates comparison plots.

Usage:
    cd lingtu
    python sim/scripts/benchmark_following.py
    # Output: sim/output/benchmark/
"""
import math
import os
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT / "sim"))

import mujoco

from following.interfaces import FollowCommand
from following.person.person_model import PersonController, get_person_xml
from following.person.trajectory import (
    StraightWalk, LTurnWalk, UTurnWalk, StopAndGoWalk,
    CircleWalk, Figure8Walk,
)
from following.perception.ground_truth import GroundTruthPerception
from following.controller.pd_follower import PDFollower
from following.controller.pure_pursuit import PurePursuitFollower
from following.metrics.recorder import MetricsRecorder
from following.metrics.plotter import (
    plot_distance_comparison,
    plot_trajectory_2d,
    plot_command_smoothness,
    plot_summary_radar,
)

# Reuse RL policy code from run_person_following
from scripts.run_person_following import (
    DOF_IDS, DOF_VEL, DEFAULT_ANGLES, ACTION_SCALE, KP, KD,
    get_obs, pd_control, build_scene_xml,
)

_BRAINSTEM_SIM = _ROOT.parent / "brainstem" / "sim"


def find_policy() -> str:
    for c in [
        _BRAINSTEM_SIM / "model" / "policy_v9_800.onnx",
        _ROOT.parent / "brainstem" / "sim" / "model" / "policy_v9_800.onnx",
    ]:
        if c.exists():
            return str(c)
    raise FileNotFoundError("No ONNX policy found")


def run_scenario(
    model, data, sess, input_name, history_size,
    trajectory, controller, target_distance=1.5,
    duration=20.0,
) -> MetricsRecorder:
    """Run one scenario and return recorded metrics."""
    from scipy.spatial.transform import Rotation as R

    perception = GroundTruthPerception()
    recorder = MetricsRecorder(target_distance=target_distance)
    person_ctrl = PersonController(model, trajectory)

    # Reset
    mujoco.mj_resetData(model, data)
    data.qpos[:3] = [10.0, 4.5, 0.45]  # closer to person
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[DOF_IDS] = DEFAULT_ANGLES
    mujoco.mj_forward(model, data)
    person_state = person_ctrl.reset(data)
    controller.reset()

    obs_history = deque(maxlen=history_size)
    action = np.zeros(16, dtype=np.float64)
    last_action = np.zeros(16, dtype=np.float64)
    target_q = DEFAULT_ANGLES.copy()
    cmd = FollowCommand()
    decimation = 4
    dt_sim = model.opt.timestep
    dt_ctrl = dt_sim * decimation
    sim_steps = int(duration / dt_sim)
    warmup_steps = int(1.0 / dt_sim)
    count = 0

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
            robot_yaw = R.from_quat(
                data.qpos[3:7][[1, 2, 3, 0]]
            ).as_euler('xyz')[2]

            person_state = person_ctrl.step(data, dt_ctrl)
            target = perception.update(None, person_state, data.time)

            if target is not None:
                cmd = controller.compute(robot_pos[:2], robot_yaw, target, dt_ctrl)
            else:
                cmd = FollowCommand()

            recorder.record(
                data.time, robot_pos, robot_yaw,
                person_state, target, cmd,
            )

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

        if trajectory.is_complete and count > warmup_steps + int(2.0 / dt_sim):
            break

    return recorder


def main():
    import onnxruntime as ort

    sim_dir = _ROOT / "sim"
    robot_xml = sim_dir / "robots" / "nova_dog" / "robot_with_camera.xml"
    scene_xml = sim_dir / "scenes" / "go2_room_nova.xml"
    policy_path = find_policy()
    output_dir = sim_dir / "output" / "benchmark"
    os.makedirs(output_dir, exist_ok=True)

    print(f"Policy:  {Path(policy_path).name}")
    print(f"Output:  {output_dir}")

    # Build scene
    person_start = [10.0, 6.0, 0.0]
    merged_xml = build_scene_xml(robot_xml, scene_xml, person_start)

    import tempfile
    tmp = tempfile.NamedTemporaryFile(
        suffix=".xml", delete=False, mode="w", encoding="utf-8",
        dir=str(sim_dir / "robots" / "nova_dog"),
    )
    tmp.write(merged_xml)
    tmp.close()

    model = mujoco.MjModel.from_xml_path(tmp.name)
    data = mujoco.MjData(model)

    sess = ort.InferenceSession(policy_path)
    input_name = sess.get_inputs()[0].name
    history_size = sess.get_inputs()[0].shape[1] // 57

    # Define test matrix
    CONTROLLERS = {
        "PD": lambda: PDFollower(target_distance=1.5),
        "PurePursuit": lambda: PurePursuitFollower(target_distance=1.5, lookahead_distance=2.0),
    }

    TRAJECTORIES = {
        "straight": lambda: StraightWalk(start=(10, 6), speed=0.4, distance=8),
        "l_turn": lambda: LTurnWalk(start=(10, 6), speed=0.5, leg1=4, leg2=4),
        "u_turn": lambda: UTurnWalk(start=(10, 6), speed=0.4, distance=4),
        "stop_and_go": lambda: StopAndGoWalk(start=(10, 6), speed=0.4),
        "circle": lambda: CircleWalk(center=(10, 6), speed=0.4, radius=2.5),
    }

    all_summaries = {}
    all_timeseries = {}

    for traj_name, traj_fn in TRAJECTORIES.items():
        print(f"\n{'='*60}")
        print(f"Trajectory: {traj_name}")

        traj_ts = {}
        traj_summaries = {}

        for ctrl_name, ctrl_fn in CONTROLLERS.items():
            label = f"{ctrl_name}_{traj_name}"
            print(f"  Running {ctrl_name}...", end=" ", flush=True)

            t0 = time.monotonic()
            recorder = run_scenario(
                model, data, sess, input_name, history_size,
                trajectory=traj_fn(),
                controller=ctrl_fn(),
                duration=20.0,
            )
            elapsed = time.monotonic() - t0
            summary = recorder.get_summary()
            ts = recorder.get_timeseries()

            print(
                f"done ({elapsed:.1f}s) "
                f"dist_err={summary.get('mean_distance_error', 0):.2f}m "
                f"smooth={summary.get('smoothness', 0):.3f}"
            )

            recorder.save(str(output_dir), label)
            traj_ts[ctrl_name] = ts
            traj_summaries[ctrl_name] = summary
            all_summaries[label] = summary
            all_timeseries[label] = ts

        # Per-trajectory comparison plots
        if traj_ts:
            plot_distance_comparison(
                traj_ts, target_distance=1.5,
                output_path=str(output_dir / f"distance_{traj_name}.png"),
                title=f"Following Distance — {traj_name}",
            )
            plot_trajectory_2d(
                traj_ts,
                output_path=str(output_dir / f"trajectory_{traj_name}.png"),
                title=f"Trajectories — {traj_name}",
            )
            plot_command_smoothness(
                traj_ts,
                output_path=str(output_dir / f"jerk_{traj_name}.png"),
                title=f"Command Smoothness — {traj_name}",
            )

    # Overall radar comparison
    if all_summaries:
        # Group by controller
        ctrl_agg = {}
        for ctrl_name in CONTROLLERS:
            ctrl_sums = [
                v for k, v in all_summaries.items() if k.startswith(ctrl_name)
            ]
            if ctrl_sums:
                ctrl_agg[ctrl_name] = {
                    key: float(np.mean([s[key] for s in ctrl_sums]))
                    for key in ctrl_sums[0]
                    if isinstance(ctrl_sums[0][key], (int, float))
                }

        plot_summary_radar(
            ctrl_agg,
            output_path=str(output_dir / "radar_overall.png"),
            title="Overall Controller Comparison",
        )

    # Print summary table
    print(f"\n{'='*60}")
    print("BENCHMARK RESULTS")
    print(f"{'='*60}")
    print(f"{'Scenario':<25} {'Dist Err':>10} {'Smoothness':>12} {'Final Dist':>12}")
    print("-" * 60)
    for label, s in sorted(all_summaries.items()):
        print(
            f"{label:<25} "
            f"{s.get('mean_distance_error', 0):>10.3f} "
            f"{s.get('smoothness', 0):>12.4f} "
            f"{s.get('final_distance', 0):>12.2f}"
        )

    print(f"\nPlots saved to: {output_dir}")
    os.unlink(tmp.name)


if __name__ == "__main__":
    main()
