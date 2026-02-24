"""
mocks.py — Mock ROS2 transport layer for Windows testing.

Provides lightweight mock objects that allow algorithm modules
to be tested on platforms without ROS2 installed.

Usage in tests:
    from mappilot_sdk.mocks import MockLLMClient, MockROS2Publisher, make_mock_scene_graph

    llm = MockLLMClient(response='{"object_id": "obj_1", "confidence": 0.9}')
    pub = MockROS2Publisher()
    sg_json = make_mock_scene_graph(["chair", "desk", "fire extinguisher"])
"""

from __future__ import annotations

import asyncio
import json
import logging
from typing import Any, Callable, Dict, List, Optional
from unittest.mock import AsyncMock, MagicMock

import numpy as np

logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────────────
#  Mock LLM Client
# ─────────────────────────────────────────────────────────────────────────────

class MockLLMClient:
    """
    Mock LLM client for offline testing.

    Supports fixed responses or callback-based dynamic responses.

    Usage:
        llm = MockLLMClient(response='{"object_id": "obj_1"}')
        # or:
        llm = MockLLMClient(callback=lambda prompt: '{"object_id": null}')
    """

    def __init__(
        self,
        response: str = '{"object_id": null, "confidence": 0.0, "reasoning": "mock"}',
        callback: Optional[Callable[[str], str]] = None,
        latency: float = 0.0,
    ):
        self._response = response
        self._callback = callback
        self._latency = latency
        self.call_count = 0
        self.last_prompt = None

    async def chat(self, messages: List[Dict], **kwargs) -> str:
        self.call_count += 1
        self.last_prompt = messages[-1]["content"] if messages else ""

        if self._latency > 0:
            await asyncio.sleep(self._latency)

        if self._callback is not None:
            return self._callback(self.last_prompt)
        return self._response

    def reset(self) -> None:
        self.call_count = 0
        self.last_prompt = None


# ─────────────────────────────────────────────────────────────────────────────
#  Mock ROS2 Node Infrastructure
# ─────────────────────────────────────────────────────────────────────────────

class MockPublisher:
    """Records published messages for assertion."""

    def __init__(self, topic: str = "/mock"):
        self.topic = topic
        self.published: List[Any] = []

    def publish(self, msg: Any) -> None:
        self.published.append(msg)

    @property
    def last(self) -> Optional[Any]:
        return self.published[-1] if self.published else None


class MockLogger:
    """Logs to Python logging (useful for debugging tests)."""

    def __init__(self, name: str = "mock_node"):
        self._logger = logging.getLogger(name)

    def info(self, msg, *args): self._logger.info(msg, *args)
    def warn(self, msg, *args): self._logger.warning(msg, *args)
    def warning(self, msg, *args): self._logger.warning(msg, *args)
    def error(self, msg, *args): self._logger.error(msg, *args)
    def debug(self, msg, *args): self._logger.debug(msg, *args)


class MockROS2Node:
    """
    Minimal mock ROS2 Node for testing modules that expect a node context.

    Does NOT require rclpy to be installed.
    """

    def __init__(self, name: str = "mock_node"):
        self.name = name
        self._publishers: Dict[str, MockPublisher] = {}
        self._params: Dict[str, Any] = {}
        self._logger = MockLogger(name)

    def get_logger(self):
        return self._logger

    def declare_parameter(self, name: str, default=None):
        self._params.setdefault(name, default)

    def get_parameter(self, name: str):
        mock = MagicMock()
        mock.value = self._params.get(name)
        mock.as_double = lambda: float(self._params.get(name, 0.0))
        mock.as_bool = lambda: bool(self._params.get(name, False))
        mock.as_string = lambda: str(self._params.get(name, ""))
        mock.as_integer = lambda: int(self._params.get(name, 0))
        return mock

    def set_parameter(self, name: str, value: Any):
        self._params[name] = value

    def create_publisher(self, msg_type, topic: str, qos=10) -> MockPublisher:
        pub = MockPublisher(topic)
        self._publishers[topic] = pub
        return pub

    def create_subscription(self, *args, **kwargs):
        return MagicMock()

    def create_timer(self, *args, **kwargs):
        return MagicMock()

    def get_publisher(self, topic: str) -> Optional[MockPublisher]:
        return self._publishers.get(topic)

    def get_clock(self):
        mock_clock = MagicMock()
        mock_clock.now = MagicMock(return_value=MagicMock(nanoseconds=0))
        return mock_clock


# ─────────────────────────────────────────────────────────────────────────────
#  Scene Graph Factory
# ─────────────────────────────────────────────────────────────────────────────

def make_mock_scene_graph(
    labels: Optional[List[str]] = None,
    seed: int = 42,
) -> str:
    """
    Generate a mock scene graph JSON for testing.

    Args:
        labels: Object labels to include. Defaults to common objects.
        seed: Random seed for reproducible CLIP features.

    Returns:
        JSON string compatible with /nav/semantic/scene_graph format.
    """
    rng = np.random.RandomState(seed)

    if labels is None:
        labels = ["chair", "desk", "monitor", "fire extinguisher", "door"]

    objects = []
    for i, label in enumerate(labels):
        feat = rng.rand(512).astype(np.float32)
        feat /= np.linalg.norm(feat)
        objects.append({
            "id": f"obj_{i:03d}",
            "label": label,
            "position": [
                float(rng.uniform(-5, 5)),
                float(rng.uniform(-5, 5)),
                float(rng.uniform(0.3, 2.0)),
            ],
            "confidence": float(rng.uniform(0.7, 0.99)),
            "clip_feature": feat.tolist(),
            "detection_count": int(rng.randint(1, 10)),
        })

    data = {
        "objects": objects,
        "relations": [],
        "regions": [
            {"name": "room_0", "object_ids": [o["id"] for o in objects]},
        ],
        "rooms": [
            {
                "room_id": 0,
                "name": "room_0",
                "center": {"x": 0.0, "y": 0.0},
                "semantic_labels": labels[:8],
            }
        ],
        "topology_edges": [],
    }
    return json.dumps(data, ensure_ascii=False)


def make_mock_detection(
    label: str = "chair",
    position: tuple = (1.0, 2.0, 0.5),
    score: float = 0.9,
    seed: Optional[int] = None,
):
    """
    Create a mock Detection3D for testing PerceptionSDK / InstanceTracker.

    Requires semantic_perception to be installed.
    """
    from semantic_perception.projection import Detection3D

    rng = np.random.RandomState(seed)
    feat = rng.rand(512).astype(np.float32)
    feat /= np.linalg.norm(feat)

    return Detection3D(
        position=np.array(position, dtype=np.float32),
        label=label,
        score=score,
        bbox_2d=np.array([0.0, 0.0, 100.0, 100.0], dtype=np.float32),
        depth=float(np.linalg.norm(position[:2])),
        features=feat,
    )
