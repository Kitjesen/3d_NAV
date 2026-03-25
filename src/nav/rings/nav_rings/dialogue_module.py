# DEPRECATED: Use SafetyRingModule instead (from nav.safety_ring_module)
#!/usr/bin/env python3
"""
Ring 3 -- DialogueModule -- hive Module version
================================================
Extracted pure aggregation logic from dialogue_manager.py into core.Module.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional

import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from core import Module, In, Out
from core.msgs.nav import Odometry
from core.msgs.semantic import (
    DialogueState,
    ExecutionEval,
    MissionStatus,
    SafetyState,
)


_ACTION_TEXT = {
    "IDLE": "待命中",
    "PLANNING": "正在规划路线",
    "EXECUTING": "正在前往目标",
    "RECOVERING": "遇到障碍，尝试恢复",
    "REPLANNING": "重新规划路线",
    "COMPLETE": "已到达目标",
    "FAILED": "任务失败",
}

_ASSESSMENT_TEXT = {
    "IDLE": "",
    "ON_TRACK": "路线跟踪正常",
    "DRIFTING": "偏离规划路线，正在修正",
    "STALLED": "前进受阻，等待恢复",
    "REGRESSING": "偏离目标，可能需要重新规划",
}

_SAFETY_TEXT = {
    "OK": "安全",
    "DEGRADED": "部分传感器离线",
    "WARN": "注意: 减速或障碍预警",
    "DANGER": "危险: 关键链路异常",
    "ESTOP": "急停",
}


class DialogueModule(Module, layer=6):
    """Ring 3: dialogue state aggregator (hive Module).
    Aggregates safety + eval + mission into user-facing state.
    Pure logic, no ROS2. Driven by tick().
    """

    safety_state: In[SafetyState]
    execution_eval: In[ExecutionEval]
    mission_status: In[MissionStatus]
    planner_status: In[str]
    odometry: In[Odometry]
    instruction: In[str]

    dialogue_state: Out[DialogueState]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        self._safety: Optional[SafetyState] = None
        self._eval: Optional[ExecutionEval] = None
        self._mission: Optional[MissionStatus] = None
        self._planner_status_val: str = "IDLE"
        self._robot_xy = (0.0, 0.0)
        self._instruction_text: str = ""

    def setup(self) -> None:
        self.safety_state.subscribe(self._on_safety)
        self.execution_eval.subscribe(self._on_eval)
        self.mission_status.subscribe(self._on_mission)
        self.planner_status.subscribe(self._on_planner)
        self.odometry.subscribe(self._on_odom)
        self.instruction.subscribe(self._on_instruction)

    def _on_safety(self, msg: SafetyState) -> None:
        self._safety = msg

    def _on_eval(self, msg: ExecutionEval) -> None:
        self._eval = msg

    def _on_mission(self, msg: MissionStatus) -> None:
        self._mission = msg

    def _on_planner(self, status: str) -> None:
        self._planner_status_val = status.strip()

    def _on_odom(self, msg: Odometry) -> None:
        x, y = msg.x, msg.y
        if math.isfinite(x) and math.isfinite(y):
            self._robot_xy = (x, y)

    def _on_instruction(self, text: str) -> None:
        text = text.strip()
        if text:
            self._instruction_text = text

    def build_state(self) -> DialogueState:
        """Aggregate all cached state into a unified DialogueState."""
        mission_state = self._mission.state if self._mission else "IDLE"
        action_text = _ACTION_TEXT.get(mission_state, mission_state)

        understood = self._instruction_text or None
        if not understood and self._mission and self._mission.goal:
            understood = f"前往 {self._mission.goal}"

        progress_pct = self._mission.progress_pct if self._mission else 0.0

        dist = self._eval.distance_to_goal if self._eval else None
        if dist is not None and dist == float("inf"):
            dist = None

        assessment = self._eval.assessment if self._eval else "IDLE"
        eval_text = _ASSESSMENT_TEXT.get(assessment, "")

        safety_level = self._safety.level if self._safety else "OK"
        safety_text = _SAFETY_TEXT.get(safety_level, safety_level)
        safety_issues = self._safety.issues if self._safety else []

        issue = None
        if safety_level in ("DANGER", "ESTOP"):
            issue = "; ".join(safety_issues[:2]) if safety_issues else "安全异常"
        elif assessment in ("STALLED", "REGRESSING"):
            issue = eval_text
        elif assessment == "DRIFTING":
            cte = self._eval.cross_track_error if self._eval else 0
            issue = f"偏离路线 {cte:.1f}m" if cte else eval_text
        elif safety_level == "WARN":
            issue = "; ".join(safety_issues[:2]) if safety_issues else "预警"
        elif self._planner_status_val == "FAILED":
            issue = "规划失败"

        eta_sec = None
        if dist and dist > 0.5 and mission_state == "EXECUTING":
            progress_rate = self._eval.progress_rate if self._eval else 0
            if progress_rate < -0.05:
                eta_sec = round(dist / abs(progress_rate))
                eta_sec = min(eta_sec, 9999)

        doing = action_text
        if mission_state == "EXECUTING" and eval_text:
            doing = f"{action_text} -- {eval_text}"

        state = DialogueState(
            understood=understood,
            doing=doing,
            progress_pct=round(progress_pct, 1),
            distance_m=round(dist, 1) if dist is not None else None,
            issue=issue,
            safety=safety_level,
            safety_text=safety_text,
            eta_sec=eta_sec,
            mission_state=mission_state,
            position_x=round(self._robot_xy[0], 2),
            position_y=round(self._robot_xy[1], 2),
        )
        return state

    def tick(self) -> DialogueState:
        """Aggregate and publish. Return DialogueState."""
        state = self.build_state()
        self.dialogue_state.publish(state)
        return state

    @property
    def current_instruction(self) -> str:
        return self._instruction_text

    @property
    def robot_position(self) -> tuple:
        return self._robot_xy
