"""base_autonomy.modules - Python Module layer for C++ autonomy stack."""

from __future__ import annotations

from importlib import import_module
from typing import Any

from .autonomy_module import add_autonomy_stack

__all__ = [
    "LocalPlannerModule",
    "PathFollowerModule",
    "TerrainModule",
    "add_autonomy_stack",
]


_EXPORTS = {
    "TerrainModule": "base_autonomy.modules.terrain_module",
    "LocalPlannerModule": "base_autonomy.modules.local_planner_module",
    "PathFollowerModule": "base_autonomy.modules.path_follower_module",
}


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(name)
    module = import_module(module_name)
    return getattr(module, name)
