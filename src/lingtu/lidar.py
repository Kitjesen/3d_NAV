"""LiDAR — compatibility shim. Use drivers.lidar.Lidar directly."""

from drivers.lidar import Lidar as LiDAR  # noqa: F401 (re-export)

__all__ = ["LiDAR"]
