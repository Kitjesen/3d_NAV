"""CmdVelMux — priority-based velocity command arbitration.

Multiple sources (teleop, visual servo, path follower, recovery) can publish
cmd_vel.  CmdVelMux selects the highest-priority *active* source and forwards
only that source's commands to the driver.

A source is "active" if it published within its timeout window.  When the
active source goes idle, the mux falls through to the next lower-priority
source that has recently published.

Priority (higher = wins):
    teleop          100   — human joystick override
    visual_servo     80   — close-range PD tracking
    recovery         60   — NavigationModule stuck recovery
    path_follower    40   — normal autonomy

All incoming cmd_vel arrive on separate In ports so the mux knows *which*
source sent the message.  One Out[Twist] port goes to the driver.

The mux also publishes `active_source: Out[str]` so other modules (Gateway,
SafetyRing, etc.) can display who controls the robot.
"""

from __future__ import annotations

import logging
import time
from typing import Dict, Optional

from core.module import Module
from core.msgs.geometry import Twist
from core.stream import In, Out

logger = logging.getLogger(__name__)

# Source definitions: name → default priority
_DEFAULT_PRIORITIES: dict[str, int] = {
    "teleop": 100,
    "visual_servo": 80,
    "recovery": 60,
    "path_follower": 40,
}


class CmdVelMux(Module, layer=0):
    """Priority-based cmd_vel multiplexer.

    Each source has its own In port.  The mux forwards the highest-priority
    active source to driver_cmd_vel.
    """

    # -- Inputs (one per source) --
    teleop_cmd_vel:        In[Twist]
    visual_servo_cmd_vel:  In[Twist]
    recovery_cmd_vel:      In[Twist]
    path_follower_cmd_vel: In[Twist]

    # -- Outputs --
    driver_cmd_vel: Out[Twist]     # → driver
    active_source:  Out[str]       # who currently controls

    def __init__(
        self,
        source_timeout: float = 0.5,
        **kw,
    ):
        super().__init__(**kw)
        self._source_timeout = source_timeout

        # Per-source state: last twist + last publish time
        self._sources: dict[str, dict] = {}
        for name, priority in _DEFAULT_PRIORITIES.items():
            self._sources[name] = {
                "priority": priority,
                "last_time": 0.0,
                "last_twist": Twist(),
            }

        self._active: str = ""
        self._last_publish_time: float = 0.0

    def setup(self) -> None:
        self.teleop_cmd_vel.subscribe(
            lambda t: self._on_source("teleop", t))
        self.visual_servo_cmd_vel.subscribe(
            lambda t: self._on_source("visual_servo", t))
        self.recovery_cmd_vel.subscribe(
            lambda t: self._on_source("recovery", t))
        self.path_follower_cmd_vel.subscribe(
            lambda t: self._on_source("path_follower", t))

    def _on_source(self, name: str, twist: Twist) -> None:
        """Handle incoming cmd_vel from a named source."""
        now = time.time()
        src = self._sources[name]
        src["last_time"] = now
        src["last_twist"] = twist

        # Select highest-priority active source
        winner = self._select_active(now)
        if winner != self._active:
            if self._active and winner:
                logger.info("CmdVelMux: %s → %s", self._active, winner)
            elif winner:
                logger.info("CmdVelMux: %s active", winner)
            self._active = winner
            self.active_source.publish(winner)

        # Only forward if this source is the winner
        if name == winner:
            self.driver_cmd_vel.publish(twist)
            self._last_publish_time = now
        elif winner and winner != name:
            # Another source has higher priority — forward winner's last twist
            # (already being forwarded by its own callback)
            pass

    def _select_active(self, now: float) -> str:
        """Return the name of the highest-priority source that is still active."""
        best_name = ""
        best_priority = -1
        for name, src in self._sources.items():
            age = now - src["last_time"]
            if age <= self._source_timeout and src["priority"] > best_priority:
                best_name = name
                best_priority = src["priority"]
        return best_name

    def health(self) -> dict:
        now = time.time()
        sources = {}
        for name, src in self._sources.items():
            age = now - src["last_time"]
            sources[name] = {
                "priority": src["priority"],
                "active": age <= self._source_timeout,
                "age_ms": round(age * 1000),
            }
        return {
            "active_source": self._active,
            "sources": sources,
        }
