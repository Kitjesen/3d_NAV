"""Protocol interfaces for the Navigation stack.

Defines structural contracts for the full navigation pipeline.
Any module implementing these protocols can be wired into the Blueprint.
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class NavigationStack(Protocol):
    """Full navigation stack interface — from goal to cmd_vel."""

    def navigate_to(self, x: float, y: float, z: float = 0.0) -> None:
        """Send a navigation goal."""
        ...

    def stop(self) -> None:
        """Emergency stop."""
        ...

    def get_status(self) -> str:
        """Current status: IDLE / NAVIGATING / STUCK / ARRIVED / ESTOP."""
        ...


@runtime_checkable
class VoxelMapper(Protocol):
    """Voxel grid mapper interface (dimos VoxelGridMapper equivalent)."""

    def add_frame(self, pointcloud: Any) -> None:
        """Ingest a new point cloud frame."""
        ...

    def get_global_map(self) -> Any:
        """Return accumulated global point cloud."""
        ...


@runtime_checkable
class CostMapper(Protocol):
    """Cost map generator interface (dimos CostMapper equivalent)."""

    def update(self, global_map: Any) -> None:
        """Update cost map from global point cloud."""
        ...

    def get_costmap(self) -> Any:
        """Return current occupancy grid."""
        ...


@runtime_checkable
class LocalPlanner(Protocol):
    """Local planner interface — obstacle avoidance + velocity output."""

    def compute_velocity(self, costmap: Any, goal: Any, odom: Any) -> Any:
        """Compute velocity command given costmap, goal, and odometry."""
        ...
