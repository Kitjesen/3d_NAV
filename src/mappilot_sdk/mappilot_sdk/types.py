"""
types.py — Pure Python dataclasses for the MapPilot SDK public API.

No ROS2, no torch, no external deps beyond numpy.
These types are transport-agnostic: they can be serialized to JSON
for ROS2 topics or used directly in offline/test pipelines.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np


@dataclass
class GoalResult:
    """Result from goal resolution (Fast Path or Slow Path)."""
    success: bool
    object_id: Optional[str]            # Matched object ID in scene graph
    label: Optional[str]                # Object label
    position: Optional[np.ndarray]      # [x, y, z] world frame
    confidence: float
    path: str                           # "fast" | "slow" | "failed"
    reasoning: str = ""
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "success": self.success,
            "object_id": self.object_id,
            "label": self.label,
            "position": self.position.tolist() if self.position is not None else None,
            "confidence": self.confidence,
            "path": self.path,
            "reasoning": self.reasoning,
        }


@dataclass
class NavigationCommand:
    """Command issued to the robot's motion planner."""
    command_type: str                   # "goal" | "velocity" | "cancel" | "explore"
    target_position: Optional[np.ndarray] = None   # [x, y, z] for "goal"
    target_label: Optional[str] = None             # Human-readable target
    confidence: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "command_type": self.command_type,
            "target_position": (
                self.target_position.tolist()
                if self.target_position is not None
                else None
            ),
            "target_label": self.target_label,
            "confidence": self.confidence,
        }


@dataclass
class SceneGraphInput:
    """Scene graph passed into NavigationSDK methods.

    Mirrors the JSON structure from /nav/semantic/scene_graph topic.
    Can be constructed offline (no ROS2 needed).
    """
    objects: List[Dict[str, Any]] = field(default_factory=list)
    relations: List[Dict[str, Any]] = field(default_factory=list)
    regions: List[Dict[str, Any]] = field(default_factory=list)
    rooms: List[Dict[str, Any]] = field(default_factory=list)
    topology_edges: List[Dict[str, Any]] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SceneGraphInput":
        return cls(
            objects=data.get("objects", []),
            relations=data.get("relations", []),
            regions=data.get("regions", []),
            rooms=data.get("rooms", []),
            topology_edges=data.get("topology_edges", []),
        )

    @classmethod
    def from_json(cls, json_str: str) -> "SceneGraphInput":
        import json
        return cls.from_dict(json.loads(json_str))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "objects": self.objects,
            "relations": self.relations,
            "regions": self.regions,
            "rooms": self.rooms,
            "topology_edges": self.topology_edges,
        }
