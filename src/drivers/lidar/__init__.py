"""LiDAR driver package — Livox MID-360.

Usage::

    from drivers.lidar import Lidar

    lidar = Lidar()
    lidar.connect("192.168.1.115")      # start driver + DDS bridge
    lidar.on_cloud(lambda pts: print(pts.shape))
    cloud = lidar.get_cloud()           # numpy (N, 4): x, y, z, intensity
    lidar.disconnect()

    # context manager
    with Lidar("192.168.1.115") as lidar:
        cloud = lidar.get_cloud()

Blueprint usage::

    from drivers.lidar import LidarModule
    bp.add(LidarModule)
"""

from .lidar import Lidar
from .lidar_module import LidarModule

__all__ = ["Lidar", "LidarModule"]
