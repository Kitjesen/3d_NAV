"""Ground-truth perception — reads person position directly from MuJoCo state.

Zero latency, zero noise. Used to isolate controller performance
from perception quality.
"""
from __future__ import annotations

from typing import Optional

from sim.following.interfaces import PerceivedTarget, PersonState


class GroundTruthPerception:
    """Wraps person_gt from the scenario loop as a PerceivedTarget."""

    def update(
        self,
        engine,
        person_gt: Optional[PersonState] = None,
        timestamp: float = 0.0,
    ) -> Optional[PerceivedTarget]:
        if person_gt is None or not person_gt.visible:
            return None
        return PerceivedTarget(
            position_world=person_gt.position.copy(),
            velocity_world=person_gt.velocity.copy(),
            confidence=1.0,
            track_id=-1,
            timestamp=timestamp,
        )
