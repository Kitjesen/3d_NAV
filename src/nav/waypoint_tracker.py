"""WaypointTracker — waypoint advancement and stuck detection.

Not a Module. Used internally by NavigationModule to separate tracking
logic from the mission FSM.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

# Event tokens returned by update()
EV_WAYPOINT_REACHED = "waypoint_reached"
EV_PATH_COMPLETE    = "path_complete"
EV_STUCK_WARN       = "stuck_warn"
EV_STUCK            = "stuck"


@dataclass
class TrackerStatus:
    """Snapshot of tracker state for one odometry tick."""
    wp_index: int
    wp_total: int
    event: str | None = field(default=None)


class WaypointTracker:
    """Tracks progress along a waypoint path.

    Handles:
      - Waypoint arrival detection (2D distance threshold)
      - Stuck detection with progressive warn (50%) → stuck (100%) thresholds
      - Each event fires exactly once per stuck/arrival occurrence

    Usage::

        tracker = WaypointTracker(threshold=1.5, stuck_timeout=10.0,
                                   stuck_dist=0.15)

        # On new path:
        tracker.reset(path, robot_pos)

        # In odometry callback:
        status = tracker.update(robot_pos)
        if status.event == EV_PATH_COMPLETE:
            ...
        elif status.event == EV_WAYPOINT_REACHED:
            next_wp = tracker.current_waypoint  # already advanced
            ...
        elif status.event == EV_STUCK:
            ...  # trigger replan
    """

    def __init__(
        self,
        threshold: float = 1.5,
        stuck_timeout: float = 10.0,
        stuck_dist: float = 0.15,
    ) -> None:
        self._threshold = threshold
        self._stuck_timeout = stuck_timeout
        self._stuck_dist = stuck_dist

        self._path: list[np.ndarray] = []
        self._wp_index: int = 0
        self._last_progress_time: float = 0.0
        self._last_progress_pos: np.ndarray = np.zeros(3)
        self._stuck_warn_sent: bool = False
        self._stuck_sent: bool = False

    # ------------------------------------------------------------------ #
    # Public API                                                           #
    # ------------------------------------------------------------------ #

    def reset(self, path: list[np.ndarray], robot_pos: np.ndarray) -> None:
        """Start tracking a new path. Clears all stuck state."""
        self._path = path
        self._wp_index = 0
        self._last_progress_time = time.time()
        self._last_progress_pos = robot_pos.copy()
        self._stuck_warn_sent = False
        self._stuck_sent = False

    def clear(self) -> None:
        """Discard current path (e.g. on stop/cancel)."""
        self._path = []
        self._wp_index = 0
        self._stuck_warn_sent = False
        self._stuck_sent = False

    def pause(self) -> None:
        """Pause tracking — reset stuck timer but keep waypoint state."""
        self._last_progress_time = time.time()
        self._stuck_warn_sent = False
        self._stuck_sent = False

    def update(self, robot_pos: np.ndarray) -> TrackerStatus:
        """Process a new odometry position.

        Returns TrackerStatus with an optional event string.
        On EV_WAYPOINT_REACHED the index has already been incremented —
        call current_waypoint to get the next target.
        """
        if not self._path or self._wp_index >= len(self._path):
            return TrackerStatus(self._wp_index, len(self._path))

        # -- Arrival check ---------------------------------------------------
        wp = self._path[self._wp_index]
        if np.linalg.norm(robot_pos[:2] - wp[:2]) < self._threshold:
            self._wp_index += 1
            self._last_progress_time = time.time()
            self._last_progress_pos = robot_pos.copy()
            self._stuck_warn_sent = False
            self._stuck_sent = False

            if self._wp_index >= len(self._path):
                return TrackerStatus(self._wp_index, len(self._path),
                                     event=EV_PATH_COMPLETE)
            return TrackerStatus(self._wp_index, len(self._path),
                                 event=EV_WAYPOINT_REACHED)

        # -- Stuck detection -------------------------------------------------
        now = time.time()
        moved = np.linalg.norm(robot_pos[:2] - self._last_progress_pos[:2])
        elapsed = now - self._last_progress_time

        # Reset if robot has moved enough
        if moved >= self._stuck_dist:
            self._last_progress_time = now
            self._last_progress_pos = robot_pos.copy()
            self._stuck_warn_sent = False
            self._stuck_sent = False
            return TrackerStatus(self._wp_index, len(self._path))

        # Warn at 50% of timeout (fires once)
        if elapsed > self._stuck_timeout * 0.5 and not self._stuck_warn_sent:
            self._stuck_warn_sent = True
            return TrackerStatus(self._wp_index, len(self._path),
                                 event=EV_STUCK_WARN)

        # Stuck at 100% of timeout (fires once per stuck episode)
        if elapsed > self._stuck_timeout and not self._stuck_sent:
            self._stuck_sent = True
            return TrackerStatus(self._wp_index, len(self._path),
                                 event=EV_STUCK)

        return TrackerStatus(self._wp_index, len(self._path))

    # ------------------------------------------------------------------ #
    # Read-only properties                                                 #
    # ------------------------------------------------------------------ #

    @property
    def current_waypoint(self) -> np.ndarray | None:
        """Current waypoint to pursue, or None if path complete/empty."""
        if self._wp_index < len(self._path):
            return self._path[self._wp_index]
        return None

    @property
    def wp_index(self) -> int:
        return self._wp_index

    @property
    def path_length(self) -> int:
        return len(self._path)

    @property
    def has_path(self) -> bool:
        return bool(self._path) and self._wp_index < len(self._path)
