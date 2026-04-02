"""Per-step metric recording for following simulation benchmarks.

Records timestamped metrics to numpy arrays for efficient batch
analysis and comparison plotting.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from sim.following.interfaces import FollowCommand, PerceivedTarget, PersonState


@dataclass
class StepRecord:
    """One step of recorded metrics."""
    timestamp: float = 0.0
    robot_x: float = 0.0
    robot_y: float = 0.0
    robot_yaw: float = 0.0
    person_x: float = 0.0
    person_y: float = 0.0
    following_distance: float = 0.0
    distance_error: float = 0.0
    lateral_error: float = 0.0
    heading_error: float = 0.0
    cmd_vx: float = 0.0
    cmd_vy: float = 0.0
    cmd_dyaw: float = 0.0
    perception_detected: bool = False
    person_visible: bool = True


class MetricsRecorder:
    """Records per-step following metrics to numpy arrays."""

    def __init__(self, target_distance: float = 1.5, max_steps: int = 100000):
        self.target_distance = target_distance
        self._records: list[StepRecord] = []
        self._prev_cmd: Optional[FollowCommand] = None
        self._prev_dt: float = 0.02

    def record(
        self,
        timestamp: float,
        robot_pos: np.ndarray,
        robot_yaw: float,
        person_state: PersonState,
        target: Optional[PerceivedTarget],
        cmd: FollowCommand,
    ) -> None:
        """Record one step of metrics."""
        rx, ry = float(robot_pos[0]), float(robot_pos[1])
        px, py = float(person_state.position[0]), float(person_state.position[1])

        dist = math.hypot(rx - px, ry - py)
        dist_err = abs(dist - self.target_distance)

        # Lateral error: perpendicular distance from person's heading line
        pvx = float(person_state.velocity[0])
        pvy = float(person_state.velocity[1])
        pspeed = math.hypot(pvx, pvy)
        if pspeed > 0.01:
            # Unit direction of person's movement
            dx, dy = pvx / pspeed, pvy / pspeed
            # Vector from person to robot
            rx_rel, ry_rel = rx - px, ry - py
            # Lateral = cross product magnitude
            lateral = abs(dx * ry_rel - dy * rx_rel)
        else:
            lateral = 0.0

        # Heading error: angle between robot heading and vector-to-person
        angle_to_person = math.atan2(py - ry, px - rx)
        heading_err = abs(
            (angle_to_person - robot_yaw + math.pi) % (2 * math.pi) - math.pi
        )

        self._records.append(StepRecord(
            timestamp=timestamp,
            robot_x=rx, robot_y=ry, robot_yaw=robot_yaw,
            person_x=px, person_y=py,
            following_distance=dist,
            distance_error=dist_err,
            lateral_error=lateral,
            heading_error=heading_err,
            cmd_vx=cmd.vx, cmd_vy=cmd.vy, cmd_dyaw=cmd.dyaw,
            perception_detected=(target is not None),
            person_visible=person_state.visible,
        ))

        self._prev_cmd = cmd

    # ── Aggregate metrics ──

    @property
    def steps(self) -> int:
        return len(self._records)

    def get_timeseries(self) -> dict[str, np.ndarray]:
        """Return all metrics as numpy arrays keyed by name."""
        if not self._records:
            return {}
        return {
            "timestamp": np.array([r.timestamp for r in self._records]),
            "robot_x": np.array([r.robot_x for r in self._records]),
            "robot_y": np.array([r.robot_y for r in self._records]),
            "person_x": np.array([r.person_x for r in self._records]),
            "person_y": np.array([r.person_y for r in self._records]),
            "following_distance": np.array([r.following_distance for r in self._records]),
            "distance_error": np.array([r.distance_error for r in self._records]),
            "lateral_error": np.array([r.lateral_error for r in self._records]),
            "heading_error": np.array([r.heading_error for r in self._records]),
            "cmd_vx": np.array([r.cmd_vx for r in self._records]),
            "cmd_vy": np.array([r.cmd_vy for r in self._records]),
            "cmd_dyaw": np.array([r.cmd_dyaw for r in self._records]),
            "perception_detected": np.array([r.perception_detected for r in self._records]),
        }

    def get_summary(self) -> dict:
        """Compute aggregate metrics."""
        if not self._records:
            return {}

        ts = self.get_timeseries()
        dist_err = ts["distance_error"]
        cmd_vx = ts["cmd_vx"]
        cmd_vy = ts["cmd_vy"]
        cmd_dyaw = ts["cmd_dyaw"]

        # Command jerk (change rate)
        jerk_vx = np.diff(cmd_vx)
        jerk_vy = np.diff(cmd_vy)
        jerk_dyaw = np.diff(cmd_dyaw)
        jerk_mag = np.sqrt(jerk_vx**2 + jerk_vy**2 + jerk_dyaw**2)
        mean_jerk = float(np.mean(jerk_mag)) if len(jerk_mag) > 0 else 0.0

        return {
            "steps": self.steps,
            "duration": float(ts["timestamp"][-1] - ts["timestamp"][0]),
            "mean_distance_error": float(np.mean(dist_err)),
            "max_distance_error": float(np.max(dist_err)),
            "std_distance_error": float(np.std(dist_err)),
            "mean_lateral_error": float(np.mean(ts["lateral_error"])),
            "mean_heading_error": float(np.mean(ts["heading_error"])),
            "tracking_ratio": float(np.mean(ts["perception_detected"])),
            "smoothness": float(1.0 / (1.0 + mean_jerk)),
            "mean_jerk": mean_jerk,
            "final_distance": float(ts["following_distance"][-1]),
        }

    def save(self, output_dir: str, label: str = "run") -> dict[str, str]:
        """Save timeseries + summary to disk."""
        os.makedirs(output_dir, exist_ok=True)
        ts = self.get_timeseries()
        summary = self.get_summary()

        npz_path = os.path.join(output_dir, f"{label}_timeseries.npz")
        np.savez_compressed(npz_path, **ts)

        json_path = os.path.join(output_dir, f"{label}_summary.json")
        with open(json_path, "w") as f:
            json.dump(summary, f, indent=2)

        return {"npz": npz_path, "json": json_path}
