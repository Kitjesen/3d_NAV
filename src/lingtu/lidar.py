"""LiDAR — compatibility shim. Use drivers.lidar.Lidar directly."""

from drivers.lidar import Lidar as LiDAR

__all__ = ["LiDAR"]
