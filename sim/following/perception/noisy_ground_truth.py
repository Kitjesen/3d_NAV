"""Noisy ground-truth perception — GT with configurable degradation.

Bridges the gap between perfect GT and full camera pipeline.
Useful for understanding how much perception noise the controller
can tolerate before breaking.
"""
from __future__ import annotations

import random
from typing import Optional

import numpy as np

from sim.following.interfaces import PerceivedTarget, PersonState


class NoisyGroundTruth:
    """Ground truth + Gaussian noise + dropout + latency."""

    def __init__(
        self,
        position_noise_std: float = 0.1,
        dropout_probability: float = 0.05,
        latency_steps: int = 2,
    ):
        self._noise_std = position_noise_std
        self._dropout_prob = dropout_probability
        self._latency = latency_steps
        self._buffer: list[Optional[PerceivedTarget]] = []

    def update(
        self,
        engine,
        person_gt: Optional[PersonState] = None,
        timestamp: float = 0.0,
    ) -> Optional[PerceivedTarget]:
        if person_gt is None or not person_gt.visible:
            self._buffer.append(None)
            return self._get_delayed()

        # Dropout
        if random.random() < self._dropout_prob:
            self._buffer.append(None)
            return self._get_delayed()

        # Add noise
        noisy_pos = person_gt.position.copy()
        noisy_pos[:2] += np.random.normal(0, self._noise_std, 2)

        target = PerceivedTarget(
            position_world=noisy_pos,
            confidence=max(0.3, 1.0 - self._noise_std),
            timestamp=timestamp,
        )
        self._buffer.append(target)
        return self._get_delayed()

    def _get_delayed(self) -> Optional[PerceivedTarget]:
        """Return the target from N steps ago (simulates latency)."""
        if len(self._buffer) <= self._latency:
            return None
        return self._buffer[-(self._latency + 1)]
