"""
Simulation scenario base class

All scenarios inherit from this class and implement setup / is_complete / is_success.
"""
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional


class Scenario(ABC):
    """Abstract base class for navigation simulation scenarios.

    Attributes:
        name        unique scenario identifier
        description scenario description
        max_time    maximum run time (seconds); timeout counts as failure
    """

    name: str = "base"
    description: str = ""
    max_time: float = 120.0

    def __init__(self):
        self._start_time: Optional[float] = None
        self._end_time: Optional[float] = None
        self._metrics: Dict[str, Any] = {}

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def setup(self, engine) -> None:
        """Scenario initialization: place robot, publish goal, set obstacles, etc.

        Args:
            engine: simulation engine instance (implements SimEngine interface)
        """
        self._start_time = time.time()
        self._metrics = {}

    @abstractmethod
    def is_complete(self, engine) -> bool:
        """Return True when the scenario has ended (success or failure)."""

    @abstractmethod
    def is_success(self, engine) -> bool:
        """Return True if the scenario completed successfully."""

    def teardown(self, engine) -> None:
        """Cleanup after scenario ends (override as needed)."""
        self._end_time = time.time()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def is_timeout(self) -> bool:
        """Return True if max_time has been exceeded."""
        if self._start_time is None:
            return False
        return (time.time() - self._start_time) > self.max_time

    def elapsed(self) -> float:
        """Elapsed run time in seconds."""
        if self._start_time is None:
            return 0.0
        end = self._end_time if self._end_time is not None else time.time()
        return end - self._start_time

    def get_metrics(self) -> Dict[str, Any]:
        """Return scenario metrics. Subclasses should populate _metrics before is_complete returns True."""
        return {
            "name": self.name,
            "elapsed_sec": round(self.elapsed(), 2),
            "success": self.is_success(None) if self._end_time else None,
            **self._metrics,
        }

    # ── Internal utilities ────────────────────────────────────────────────────

    def _distance_2d(self, pos_a, pos_b) -> float:
        """Euclidean distance in XY plane."""
        import math
        return math.sqrt((pos_a[0] - pos_b[0]) ** 2 +
                         (pos_a[1] - pos_b[1]) ** 2)
