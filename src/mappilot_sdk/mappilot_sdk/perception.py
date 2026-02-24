"""
perception.py — PerceptionSDK

Offline scene graph building without ROS2.

Usage:
    from mappilot_sdk import PerceptionSDK

    sdk = PerceptionSDK()
    tracked = sdk.update([det1, det2, det3])
    scene_graph_json = sdk.get_scene_graph_json()
"""

from __future__ import annotations

import json
import logging
from typing import Any, Dict, List, Optional

import numpy as np

logger = logging.getLogger(__name__)


class PerceptionSDK:
    """
    ROS2-free scene graph building SDK.

    Wraps InstanceTracker, IndustrialKnowledgeGraph, and TopologySemGraph
    for offline use (CI, Windows, integration tests).

    No rclpy, no cv_bridge, no sensor_msgs — works anywhere Python runs.
    """

    def __init__(
        self,
        merge_distance: float = 0.5,
        clip_threshold: float = 0.75,
        max_objects: int = 200,
        use_knowledge_graph: bool = True,
    ):
        self._tracker = None
        self._kg = None
        self._initialized = False

        self._try_init(
            merge_distance=merge_distance,
            clip_threshold=clip_threshold,
            max_objects=max_objects,
            use_knowledge_graph=use_knowledge_graph,
        )

    def _try_init(self, **kwargs) -> None:
        try:
            from semantic_perception.instance_tracker import InstanceTracker

            kg = None
            if kwargs.pop("use_knowledge_graph", True):
                try:
                    from semantic_perception.knowledge_graph import IndustrialKnowledgeGraph
                    kg = IndustrialKnowledgeGraph()
                    self._kg = kg
                except ImportError:
                    pass

            self._tracker = InstanceTracker(**kwargs, knowledge_graph=kg)
            self._initialized = True
        except ImportError as e:
            logger.warning("InstanceTracker not available: %s", e)

    @property
    def is_available(self) -> bool:
        return self._initialized

    # ── Detection Update ─────────────────────────────────────────────────────

    def update_from_detections(self, detections: List[Any]) -> int:
        """
        Update scene graph from a list of Detection3D objects.

        Args:
            detections: List of semantic_perception.projection.Detection3D instances.

        Returns:
            Number of currently tracked objects.
        """
        if self._tracker is None:
            return 0
        self._tracker.update(detections)
        return len(self._tracker.objects)

    def get_scene_graph_json(self) -> str:
        """Export current scene graph as JSON string."""
        if self._tracker is None:
            return json.dumps({"objects": [], "relations": [], "rooms": []})
        return self._tracker.get_scene_graph_json()

    def get_scene_graph_dict(self) -> Dict[str, Any]:
        """Export current scene graph as dict."""
        return json.loads(self.get_scene_graph_json())

    def get_object_count(self) -> int:
        """Return number of tracked objects."""
        if self._tracker is None:
            return 0
        return len(self._tracker.objects)

    def clear(self) -> None:
        """Reset the tracker (for new environment / test isolation)."""
        if self._tracker is not None:
            self._tracker._objects.clear()
            self._tracker._next_id = 0
            self._tracker._views.clear()

    # ── Knowledge Graph Queries ──────────────────────────────────────────────

    def get_safety_level(self, label: str) -> str:
        """Query safety level for an object label ('safe'|'caution'|'dangerous'|'forbidden')."""
        if self._kg is None:
            return "safe"
        return self._kg.get_safety_level(label).value

    def get_affordances(self, label: str) -> List[str]:
        """Query affordances for an object label."""
        if self._kg is None:
            return []
        return [a.value for a in self._kg.get_affordances(label)]

    def enrich_object(self, label: str) -> Dict[str, Any]:
        """Enrich object with KG knowledge (safety, affordances, typical locations)."""
        if self._kg is None:
            return {"kg_matched": False}
        return self._kg.enrich_object_properties(label)

    def check_safety(self, label: str, action: str) -> Optional[Dict[str, Any]]:
        """
        Check if an action on an object is safe.

        Returns:
            None if safe, or dict with 'response', 'message_zh', 'message_en'
            if a constraint is triggered.
        """
        if self._kg is None:
            return None
        constraint = self._kg.check_safety(label, action)
        if constraint is None:
            return None
        return {
            "constraint_id": constraint.constraint_id,
            "response": constraint.response,
            "message_zh": constraint.message_zh,
            "message_en": constraint.message_en,
            "max_approach_distance": constraint.max_approach_distance,
        }
