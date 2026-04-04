"""LiDAR driver package — Livox MID-360.

Replaces ``ros2 launch livox_ros_driver2 ...`` with Python:

    from drivers.lidar import Lidar

    lidar = Lidar()
    lidar.connect("192.168.1.115")      # starts driver + DDS bridge
    lidar.on_cloud(lambda pts: ...)     # numpy (N, 4): x, y, z, intensity
    lidar.on_imu(lambda imu: ...)       # core.msgs.sensor.Imu
    cloud = lidar.wait_for_cloud()      # block until first frame
    print(lidar.health)                 # fps, uptime, point count, ...
    lidar.disconnect()

    # context manager
    with Lidar("192.168.1.115") as lidar:
        cloud = lidar.wait_for_cloud()

Blueprint usage:

    from drivers.lidar import LidarModule

    bp.add(LidarModule)
    bp.wire("LidarModule", "scan", "TerrainModule", "cloud")
"""

from .lidar import Lidar, LidarHealth, LidarState
from .lidar_module import LidarModule

__all__ = ["Lidar", "LidarHealth", "LidarModule", "LidarState"]
