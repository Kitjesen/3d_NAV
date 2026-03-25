"""Protocol interface for the Safety layer (Ring 1 reflex arc).

Defines the minimal contract for safety-checking components that
aggregate link liveness, terrain hazards, and E-stop signals into a
unified safety state.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class SafetyChecker(Protocol):
    """Safety checker interface.

    Implementations: SafetyMonitor (Ring 1 reflex-arc aggregation).
    The ``check`` method inspects the current robot/environment state
    and returns a safety verdict dict (level, issues, etc.).
    """

    def check(self, state: dict) -> dict:
        """Evaluate *state* and return a safety verdict.

        The returned dict should contain at least:
        - ``level``: safety level string (e.g. ``"OK"``, ``"WARN"``, ``"STOP"``)
        - ``issues``: list of issue descriptions (may be empty)
        """
        ...
