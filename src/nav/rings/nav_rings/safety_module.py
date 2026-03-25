#!/usr/bin/env python3
"""
Ring 1 -- SafetyModule -- hive Module version
==============================================
Extracted pure rule-based logic from safety_monitor.py into core.Module.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from core import Module, In, Out
from core.msgs.nav import Odometry
from core.msgs.geometry import Twist
from core.msgs.semantic import SafetyState


class SafetyLevel(Enum):
    OK = "OK"
    DEGRADED = "DEGRADED"
    WARN = "WARN"
    DANGER = "DANGER"
    ESTOP = "ESTOP"


_LEVEL_ORDER = {
    SafetyLevel.OK: 0,
    SafetyLevel.DEGRADED: 1,
    SafetyLevel.WARN: 2,
    SafetyLevel.DANGER: 3,
    SafetyLevel.ESTOP: 4,
}


def _max_level(a: SafetyLevel, b: SafetyLevel) -> SafetyLevel:
    return a if _LEVEL_ORDER[a] >= _LEVEL_ORDER[b] else b


@dataclass
class LinkStatus:
    """A monitored communication link."""
    name: str
    timeout_sec: float = 2.0
    critical: bool = True
    last_seen: float = 0.0

    @property
    def alive(self) -> bool:
        if self.last_seen == 0.0:
            return True
        return (time.monotonic() - self.last_seen) < self.timeout_sec

    @property
    def stale_sec(self) -> float:
        if self.last_seen == 0.0:
            return 0.0
        return time.monotonic() - self.last_seen


class SafetyModule(Module, layer=0):
    """Ring 1: reflex safety aggregator (hive Module).
    Pure rule-based logic, no ROS2. Driven by tick().
    """

    odometry: In[Odometry]
    terrain_hb: In[float]
    cmd_vel: In[Twist]
    stop_signal: In[int]
    slow_down: In[int]
    watchdog: In[bool]
    planner_status: In[str]
    loc_quality: In[float]

    safety_state: Out[SafetyState]
    stop_cmd: Out[int]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        self._odom_timeout = config.get("odom_timeout_sec", 2.0)
        self._driver_timeout = config.get("driver_timeout_sec", 2.0)
        self._terrain_timeout = config.get("terrain_timeout_sec", 5.0)
        self._cmdvel_timeout = config.get("cmdvel_timeout_sec", 3.0)
        self._holdoff = config.get("escalation_holdoff_sec", 1.0)
        self._loc_quality_warn = config.get("loc_quality_warn_threshold", 0.3)

        self._links: Dict[str, LinkStatus] = {
            "slam": LinkStatus(name="SLAM", timeout_sec=self._odom_timeout, critical=True),
            "driver": LinkStatus(name="Driver", timeout_sec=self._driver_timeout, critical=True),
            "terrain": LinkStatus(name="Terrain", timeout_sec=self._terrain_timeout, critical=False),
            "cmdvel": LinkStatus(name="PathFollower", timeout_sec=self._cmdvel_timeout, critical=False),
        }
        self._stop_level: int = 0
        self._slow_down_level: int = 0
        self._watchdog_active: bool = False
        self._planner_status_val: str = "IDLE"
        self._navigating: bool = False
        self._loc_quality_val: float = 1.0
        self._last_escalation: float = 0.0

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.terrain_hb.subscribe(self._on_terrain)
        self.cmd_vel.subscribe(self._on_cmdvel)
        self.stop_signal.subscribe(self._on_stop)
        self.slow_down.subscribe(self._on_slow_down)
        self.watchdog.subscribe(self._on_watchdog)
        self.planner_status.subscribe(self._on_planner_status)
        self.loc_quality.subscribe(self._on_loc_quality)

    def _on_odom(self, msg: Odometry) -> None:
        self._links["slam"].last_seen = time.monotonic()

    def _on_terrain(self, ts: float) -> None:
        self._links["terrain"].last_seen = time.monotonic()

    def _on_cmdvel(self, msg: Twist) -> None:
        self._links["cmdvel"].last_seen = time.monotonic()
        vx = msg.linear.x
        vy = msg.linear.y
        self._navigating = (
            math.isfinite(vx) and math.isfinite(vy)
            and (abs(vx) > 0.01 or abs(vy) > 0.01)
        )

    def _on_stop(self, level: int) -> None:
        self._stop_level = level

    def _on_slow_down(self, level: int) -> None:
        self._slow_down_level = level

    def _on_watchdog(self, active: bool) -> None:
        self._links["driver"].last_seen = time.monotonic()
        self._watchdog_active = active

    def _on_planner_status(self, status: str) -> None:
        self._planner_status_val = status.strip()

    def _on_loc_quality(self, quality: float) -> None:
        if math.isfinite(quality):
            self._loc_quality_val = quality

    def tick(self) -> SafetyState:
        """Run one safety evaluation cycle. Publish and return SafetyState."""
        now = time.monotonic()
        level = SafetyLevel.OK
        issues: List[str] = []

        for key, link in self._links.items():
            if not link.alive and link.last_seen > 0:
                stale = link.stale_sec
                if link.critical:
                    level = _max_level(level, SafetyLevel.DANGER)
                    issues.append(f"{link.name} stale {stale:.1f}s (CRITICAL)")
                else:
                    level = _max_level(level, SafetyLevel.DEGRADED)
                    issues.append(f"{link.name} stale {stale:.1f}s")

        if self._stop_level >= 2:
            level = SafetyLevel.ESTOP
            issues.append("stop=2 (collision/estop)")
        elif self._stop_level == 1:
            level = _max_level(level, SafetyLevel.WARN)
            issues.append("stop=1 (soft stop)")

        if self._watchdog_active:
            level = _max_level(level, SafetyLevel.WARN)
            issues.append("watchdog active (no cmd_vel)")

        if self._planner_status_val == "STUCK":
            level = _max_level(level, SafetyLevel.WARN)
            issues.append("STUCK detected")
        elif self._planner_status_val == "WARN_STUCK":
            level = _max_level(level, SafetyLevel.WARN)
            issues.append("WARN_STUCK")

        if self._slow_down_level > 0:
            level = _max_level(level, SafetyLevel.DEGRADED)
            issues.append(f"slow_down={self._slow_down_level}")

        if 0 < self._loc_quality_val < self._loc_quality_warn:
            level = _max_level(level, SafetyLevel.WARN)
            issues.append(f"loc_quality={self._loc_quality_val:.2f} (low)")

        if (level in (SafetyLevel.DANGER, SafetyLevel.ESTOP)
                and self._navigating
                and (now - self._last_escalation) > self._holdoff):
            self._last_escalation = now
            self.stop_cmd.publish(2)

        state = SafetyState(level=level.value, issues=issues if issues else [])
        self.safety_state.publish(state)
        return state

    @property
    def links(self) -> Dict[str, LinkStatus]:
        return dict(self._links)

    @property
    def navigating(self) -> bool:
        return self._navigating

    @property
    def current_stop_level(self) -> int:
        return self._stop_level
