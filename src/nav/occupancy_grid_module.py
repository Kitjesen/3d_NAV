"""OccupancyGridModule — real-time 2D occupancy grid + costmap from LiDAR.

Subscribes to SLAMModule.map_cloud (PointCloud2).  Projects 3-D points into a
robot-centric 2-D grid, marks occupied cells, inflates obstacles with a
circular kernel, then publishes:

  occupancy_grid → ESDFModule  (typed OccupancyGrid msg)
  costmap        → NavigationModule.costmap  (dict: grid/resolution/origin)

Ports:
  In:  map_cloud (PointCloud2), odometry (Odometry)
  Out: occupancy_grid (OccupancyGrid), costmap (dict)
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, Optional

import numpy as np

from core.module import Module
from core.msgs.geometry import Pose, Quaternion, Vector3
from core.msgs.nav import OccupancyGrid, Odometry
from core.msgs.sensor import PointCloud2
from core.registry import register
from core.stream import In, Out

logger = logging.getLogger(__name__)


@register("map", "occupancy_grid", description="2D occupancy grid from LiDAR point cloud")
class OccupancyGridModule(Module, layer=2):
    """Project LiDAR cloud → 2-D occupancy grid + inflated costmap.

    The grid is robot-centric: origin tracks the robot so the robot is always
    at the centre.  Published at a throttled rate (expensive per-frame rebuild).

    Inflation uses a pre-built circular binary structuring element via
    scipy.ndimage.binary_dilation (falls back to pure-numpy if scipy absent).
    """

    map_cloud: In[PointCloud2]
    odometry:  In[Odometry]

    occupancy_grid: Out[OccupancyGrid]  # typed msg → ESDFModule
    costmap:        Out[dict]           # dict    → NavigationModule.costmap

    def __init__(
        self,
        resolution: float = 0.2,         # metres per cell
        map_radius: float = 30.0,         # half-width of robot-centric grid (m)
        z_min: float = 0.10,              # ignore points below this height
        z_max: float = 2.00,              # ignore points above this height
        inflation_radius: float = 0.50,   # obstacle inflation radius (m)
        robot_clear_radius: float = 0.60, # clear self footprint from local costmap
        publish_hz: float = 2.0,
        **kw,
    ):
        super().__init__(**kw)
        self._res = resolution
        self._radius = map_radius
        self._z_min = z_min
        self._z_max = z_max
        self._inf_radius = inflation_radius
        self._robot_clear_radius = robot_clear_radius
        self._interval = 1.0 / publish_hz
        self._robot_xy = np.zeros(2, dtype=np.float64)
        self._gs = int(2 * map_radius / resolution)  # grid side (cells)
        self._kernel: np.ndarray | None = None    # circular dilation kernel

    def setup(self) -> None:
        try:
            import scipy.ndimage  # noqa: F401  — required by _inflate
        except ImportError as e:
            raise RuntimeError(
                "OccupancyGridModule requires scipy for binary_dilation. "
                "Install with: pip install scipy"
            ) from e
        self._kernel = self._make_circle_kernel(self._inf_radius, self._res)
        self.map_cloud.subscribe(self._on_cloud)
        self.odometry.subscribe(self._on_odom)
        self.map_cloud.set_policy("throttle", interval=self._interval)

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_xy[0] = odom.x
        self._robot_xy[1] = odom.y

    def _on_cloud(self, cloud: PointCloud2) -> None:
        if cloud.is_empty:
            return
        pts = cloud.points[:, :3]

        # height filter — keep obstacle-height points only
        mask = (pts[:, 2] > self._z_min) & (pts[:, 2] < self._z_max)
        pts2d = pts[mask, :2]
        if pts2d.shape[0] == 0:
            return

        origin_xy = self._robot_xy - self._radius   # bottom-left corner
        gs = self._gs

        # LiDAR often sees the robot's own legs/body in sim; filter near-body
        # returns and clear a local free-space disk around the robot center.
        d_robot = np.linalg.norm(pts2d - self._robot_xy[None, :], axis=1)
        pts2d = pts2d[d_robot >= self._robot_clear_radius]
        if pts2d.shape[0] == 0:
            return

        ix = np.floor((pts2d[:, 0] - origin_xy[0]) / self._res).astype(np.int32)
        iy = np.floor((pts2d[:, 1] - origin_xy[1]) / self._res).astype(np.int32)
        valid = (ix >= 0) & (ix < gs) & (iy >= 0) & (iy < gs)
        ix, iy = ix[valid], iy[valid]

        binary = np.zeros((gs, gs), dtype=np.bool_)
        binary[iy, ix] = True
        clear_mask = self._robot_clear_mask(origin_xy)
        binary[clear_mask] = False

        inflated = self._inflate(binary)
        # occupied cells stay at 100; inflated fringe scaled by proximity
        cost = (inflated.astype(np.float32) * 100.0).clip(0, 100)
        cost[binary] = 100.0
        cost[clear_mask] = 0.0
        grid_int8 = cost.astype(np.int8)

        origin_pose = Pose(
            position=Vector3(float(origin_xy[0]), float(origin_xy[1]), 0.0),
            orientation=Quaternion(0, 0, 0, 1),
        )
        og = OccupancyGrid(
            grid=grid_int8,
            resolution=self._res,
            origin=origin_pose,
            ts=time.time(),
            frame_id="map",
        )
        self.occupancy_grid.publish(og)
        self.costmap.publish({
            "grid":       cost,               # float32 0-100
            "resolution": self._res,
            "origin":     origin_xy.tolist(),
            "ts":         og.ts,
        })

    def _inflate(self, binary: np.ndarray) -> np.ndarray:
        """Binary dilation with pre-built circular kernel."""
        try:
            from scipy.ndimage import binary_dilation
            return binary_dilation(binary, structure=self._kernel).astype(np.float32)
        except ImportError:
            # pure-numpy fallback: sliding-window max (slower, same result)
            from numpy.lib.stride_tricks import sliding_window_view
            k = int(self._kernel.shape[0])
            pad = k // 2
            padded = np.pad(binary.astype(np.float32), pad, mode="constant")
            windows = sliding_window_view(padded, (k, k))
            return windows.max(axis=(-2, -1))

    def _robot_clear_mask(self, origin_xy: np.ndarray) -> np.ndarray:
        gs = self._gs
        rx = int(np.floor((self._robot_xy[0] - origin_xy[0]) / self._res))
        ry = int(np.floor((self._robot_xy[1] - origin_xy[1]) / self._res))
        r = max(1, int(np.ceil(self._robot_clear_radius / self._res)))
        yy, xx = np.ogrid[:gs, :gs]
        return (xx - rx) ** 2 + (yy - ry) ** 2 <= r ** 2

    @staticmethod
    def _make_circle_kernel(radius_m: float, res: float) -> np.ndarray:
        r = max(1, int(np.ceil(radius_m / res)))
        y, x = np.ogrid[-r: r + 1, -r: r + 1]
        return (x ** 2 + y ** 2) <= r ** 2

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["occupancy_grid"] = {
            "resolution":    self._res,
            "map_radius":    self._radius,
            "grid_size":     self._gs,
            "z_range":       [self._z_min, self._z_max],
            "inflation_m":   self._inf_radius,
            "robot_clear_m": self._robot_clear_radius,
        }
        return info
