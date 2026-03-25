"""SafetyRingModule — unified safety + evaluation + dialogue in one Module.

Replaces 3 separate modules (SafetyModule, EvaluatorModule, DialogueModule).
Three internal rings, one Module:

  Ring 1 (Reflex):  obstacle/link checks → stop_cmd
  Ring 2 (Cognition): cross-track error, progress rate → assessment
  Ring 3 (Dialogue):  aggregate state → user-facing status

Usage::

    bp.add(SafetyRingModule)
    # auto_wire connects odometry, path, cmd_vel, etc.
"""

from __future__ import annotations

import logging
import math
import time
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from core.module import Module
from core.stream import In, Out
from core.msgs.nav import Odometry, Path
from core.msgs.geometry import Twist, Vector3
from core.msgs.semantic import SafetyState, ExecutionEval, MissionStatus
from core.registry import register

logger = logging.getLogger(__name__)


class SafetyLevel(Enum):
    SAFE = 0
    WARN = 1
    STOP = 2


class Assessment(Enum):
    IDLE = "IDLE"
    ON_TRACK = "ON_TRACK"
    DRIFTING = "DRIFTING"
    STALLED = "STALLED"
    REGRESSING = "REGRESSING"


@register("safety", "ring", description="Unified safety ring (reflex + eval + dialogue)")
class SafetyRingModule(Module, layer=0):
    """Three-ring safety architecture in one Module.

    Ring 1 (reflex): obstacle detection → stop_cmd
    Ring 2 (cognition): execution quality evaluation
    Ring 3 (dialogue): user-facing state aggregation
    """

    # -- Inputs --
    odometry: In[Odometry]
    path: In[Path]
    cmd_vel: In[Twist]
    mission_status: In[dict]

    # -- Outputs --
    stop_cmd: Out[int]           # 0=clear, 1=soft, 2=hard
    safety_state: Out[SafetyState]
    execution_eval: Out[ExecutionEval]
    dialogue_state: Out[dict]

    def __init__(
        self,
        cross_track_warn: float = 1.5,
        cross_track_danger: float = 3.0,
        stall_threshold: float = 0.05,
        progress_window_sec: float = 3.0,
        odom_timeout_ms: float = 500.0,
        cmd_vel_timeout_ms: float = 300.0,
        **kw,
    ):
        super().__init__(**kw)
        # Ring 1 config
        self._odom_timeout = odom_timeout_ms / 1000.0
        self._cmdvel_timeout = cmd_vel_timeout_ms / 1000.0

        # Ring 2 config
        self._ct_warn = cross_track_warn
        self._ct_danger = cross_track_danger
        self._stall_thr = stall_threshold
        self._progress_window = progress_window_sec

        # Ring 1 state
        self._last_odom_time = time.time()
        self._last_cmdvel_time = time.time()
        self._safety_level = SafetyLevel.SAFE

        # Ring 2 state
        self._robot_xy = np.zeros(2)
        self._robot_yaw = 0.0
        self._path_points: Optional[np.ndarray] = None
        self._goal_xy: Optional[np.ndarray] = None
        self._cmd_speed = 0.0
        self._actual_speed = 0.0
        self._progress_history: List[Tuple[float, float]] = []
        self._last_progress_time = time.monotonic()
        self._last_distance = float("inf")
        self._assessment = Assessment.IDLE

        # Ring 3 state
        self._latest_mission: Optional[dict] = None
        self._instruction = ""

    def setup(self):
        self.odometry.subscribe(self._on_odom)
        self.path.subscribe(self._on_path)
        self.cmd_vel.subscribe(self._on_cmdvel)
        self.mission_status.subscribe(self._on_mission)

    # -- Ring 1: Reflex Safety -----------------------------------------------

    def _check_links(self) -> SafetyLevel:
        """Check communication link health."""
        now = time.time()
        odom_alive = (now - self._last_odom_time) < self._odom_timeout
        cmd_alive = (now - self._last_cmdvel_time) < self._cmdvel_timeout

        if not odom_alive:
            return SafetyLevel.STOP
        if not cmd_alive:
            return SafetyLevel.WARN
        return SafetyLevel.SAFE

    def _publish_safety(self):
        level = self._check_links()
        if level != self._safety_level:
            self._safety_level = level
            if level == SafetyLevel.STOP:
                self.stop_cmd.publish(2)
            elif level == SafetyLevel.WARN:
                self.stop_cmd.publish(1)
            else:
                self.stop_cmd.publish(0)

        self.safety_state.publish(SafetyState(
            level=level.value,
        ))

    # -- Ring 2: Execution Evaluation ----------------------------------------

    def _cross_track_error(self) -> float:
        if self._path_points is None or len(self._path_points) < 2:
            return 0.0
        p = self._robot_xy
        a = self._path_points[:-1]
        b = self._path_points[1:]
        ab = b - a
        ap = p - a
        ab_sq = np.sum(ab * ab, axis=1)
        ab_sq = np.where(ab_sq < 1e-10, 1.0, ab_sq)
        t = np.clip(np.sum(ap * ab, axis=1) / ab_sq, 0.0, 1.0)
        proj = a + t[:, np.newaxis] * ab
        dists = np.linalg.norm(p - proj, axis=1)
        return float(np.min(dists))

    def _distance_to_goal(self) -> float:
        if self._goal_xy is None:
            return float("inf")
        return float(np.linalg.norm(self._robot_xy - self._goal_xy))

    def _progress_rate(self) -> float:
        now = time.monotonic()
        dist = self._distance_to_goal()
        self._progress_history.append((now, dist))
        cutoff = now - self._progress_window
        self._progress_history = [(t, d) for t, d in self._progress_history if t > cutoff]
        if len(self._progress_history) < 2:
            return 0.0
        t0, d0 = self._progress_history[0]
        t1, d1 = self._progress_history[-1]
        dt = t1 - t0
        return (d1 - d0) / dt if dt > 0.1 else 0.0

    def _evaluate(self):
        if self._path_points is None:
            self._assessment = Assessment.IDLE
            return

        cte = self._cross_track_error()
        rate = self._progress_rate()
        now = time.monotonic()
        dist = self._distance_to_goal()

        if dist < self._last_distance - 0.1:
            self._last_progress_time = now
            self._last_distance = dist
        stall = now - self._last_progress_time

        if rate > 0.02 and stall > 3.0:
            self._assessment = Assessment.REGRESSING
        elif stall > self._progress_window:
            self._assessment = Assessment.STALLED
        elif cte > self._ct_warn:
            self._assessment = Assessment.DRIFTING
        else:
            self._assessment = Assessment.ON_TRACK

        self.execution_eval.publish(ExecutionEval(
            assessment=self._assessment.value,
            cross_track_error=round(cte, 2),
            distance_to_goal=round(dist, 1),
            progress_rate=round(rate, 3),
        ))

    # -- Ring 3: Dialogue State ----------------------------------------------

    def _publish_dialogue(self):
        mission = self._latest_mission or {}
        self.dialogue_state.publish({
            "safety": self._safety_level.name,
            "assessment": self._assessment.value,
            "mission": mission.get("state", "IDLE"),
            "instruction": self._instruction,
            "ts": time.time(),
        })

    # -- Input callbacks -----------------------------------------------------

    def _on_odom(self, odom: Odometry):
        self._last_odom_time = time.time()
        x, y = odom.x, odom.y
        if math.isfinite(x) and math.isfinite(y):
            self._robot_xy = np.array([x, y])
        self._robot_yaw = odom.yaw
        self._actual_speed = math.hypot(odom.vx, odom.vy)

        # Tick all three rings
        self._publish_safety()
        self._evaluate()
        self._publish_dialogue()

    def _on_path(self, msg: Path):
        pts = []
        for ps in msg.poses:
            if math.isfinite(ps.x) and math.isfinite(ps.y):
                pts.append([ps.x, ps.y])
        if len(pts) >= 2:
            self._path_points = np.array(pts)
            self._goal_xy = self._path_points[-1].copy()
            self._progress_history.clear()
            self._last_progress_time = time.monotonic()
            self._last_distance = float("inf")
        else:
            self._path_points = None
            self._goal_xy = None

    def _on_cmdvel(self, msg: Twist):
        self._last_cmdvel_time = time.time()
        self._cmd_speed = math.hypot(msg.linear.x, msg.linear.y)

    def _on_mission(self, status: dict):
        self._latest_mission = status

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["safety_ring"] = {
            "level": self._safety_level.name,
            "assessment": self._assessment.value,
            "has_path": self._path_points is not None,
        }
        return info
