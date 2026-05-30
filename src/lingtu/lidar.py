"""LiDAR — compatibility shim. Use drivers.real.lidar.Lidar directly."""

from drivers.real.lidar import Lidar as LiDAR

__all__ = ["LiDAR"]
