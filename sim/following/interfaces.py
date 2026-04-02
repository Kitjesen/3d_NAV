"""Core interfaces for the person-following simulation framework.

All swappable layers are Protocol-based. Callers depend only on the
protocol; implementations are injected via config/factory.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Protocol, runtime_checkable

import numpy as np


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class PersonState:
    """Ground-truth person state from the trajectory generator."""
    position: np.ndarray      # [x, y, z] world frame (meters)
    velocity: np.ndarray      # [vx, vy, vz] world frame (m/s)
    heading: float = 0.0      # yaw angle (radians)
    visible: bool = True      # line-of-sight from robot start area
    timestamp: float = 0.0


@dataclass
class PerceivedTarget:
    """Output of any perception pipeline."""
    position_world: np.ndarray   # [x, y, z] estimated world position
    confidence: float = 1.0      # 0.0 to 1.0
    bbox: Optional[tuple] = None # (x, y, w, h) image pixels, None for GT
    track_id: int = -1           # FusionMOT track ID, -1 for GT
    timestamp: float = 0.0


@dataclass
class FollowCommand:
    """Velocity command output of the following controller."""
    vx: float = 0.0     # forward m/s
    vy: float = 0.0     # lateral m/s
    dyaw: float = 0.0   # yaw rate rad/s


# ---------------------------------------------------------------------------
# Protocol interfaces (independently replaceable)
# ---------------------------------------------------------------------------

@runtime_checkable
class PersonTrajectory(Protocol):
    """Generates a person's walking path through the scene."""

    def reset(self) -> PersonState:
        """Reset to initial position. Returns starting state."""
        ...

    def step(self, dt: float) -> PersonState:
        """Advance by dt seconds. Returns updated state."""
        ...

    @property
    def is_complete(self) -> bool:
        """Whether the trajectory has ended."""
        ...


@runtime_checkable
class PerceptionPipeline(Protocol):
    """Detects and tracks the person from sensor data."""

    def update(
        self,
        engine,
        person_gt: Optional[PersonState],
        timestamp: float,
    ) -> Optional[PerceivedTarget]:
        """Process one frame. Returns target or None if not detected."""
        ...


@runtime_checkable
class FollowingController(Protocol):
    """Computes velocity commands to follow the perceived target."""

    def compute(
        self,
        robot_pos: np.ndarray,
        robot_yaw: float,
        target: PerceivedTarget,
        dt: float,
    ) -> FollowCommand:
        """Compute (vx, vy, dyaw) to follow the target."""
        ...

    def reset(self) -> None:
        """Reset controller state."""
        ...
