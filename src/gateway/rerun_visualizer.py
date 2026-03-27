"""RerunVisualizerModule — real-time 3D visualization via Rerun.

Subscribes to navigation data (odometry, point cloud, paths, cmd_vel)
and logs to Rerun for browser-based 3D visualization.

Usage::

    # Add to blueprint
    bp.add(RerunVisualizerModule, port=9090)

    # View in browser
    # Open: http://localhost:9090

Requirements::

    pip install rerun-sdk
"""
from __future__ import annotations

import logging
import math
import time
from typing import Any, Dict, Optional

import numpy as np

from core.module import Module
from core.stream import In
from core.msgs.geometry import Twist, PoseStamped
from core.msgs.nav import Odometry, Path
from core.msgs.sensor import PointCloud

logger = logging.getLogger(__name__)


class RerunVisualizerModule(Module, layer=6):
    """Real-time 3D navigation visualization via Rerun.

    Subscribes to navigation data streams and logs them to Rerun so any
    browser connected to the serve port gets a live 3D view of the robot.

    Inputs
    ------
    odometry    : robot pose + velocity (from NavigationModule / ThunderDriver)
    lidar_cloud : raw LiDAR point cloud (from AutonomyModule)
    cmd_vel     : velocity command being sent to the robot
    global_path : planned path as a Path message or list of waypoints
    waypoint    : current immediate target waypoint
    terrain_map : processed terrain point cloud (from AutonomyModule)
    """

    # -- input ports ---------------------------------------------------------
    odometry: In[Odometry]
    lidar_cloud: In[PointCloud]
    cmd_vel: In[Twist]
    global_path: In[Path]
    waypoint: In[PoseStamped]
    terrain_map: In[PointCloud]

    # -- constants -----------------------------------------------------------
    _GRID_HALF = 10        # grid extends ±10 m from origin
    _MAX_TRAIL = 2000      # maximum robot trail points kept in memory

    def __init__(self, port: int = 9090, **kw: Any) -> None:
        super().__init__(**kw)
        self._port = port
        self._rr: Optional[Any] = None   # rerun module handle; None if not installed
        self._trail: list[list[float]] = []
        self._frame_count: int = 0

    # -- lifecycle -----------------------------------------------------------

    def setup(self) -> None:
        # Wire all input subscriptions.
        self.odometry.subscribe(self._on_odom)
        self.lidar_cloud.subscribe(self._on_cloud)
        self.cmd_vel.subscribe(self._on_cmd_vel)
        self.global_path.subscribe(self._on_path)
        self.waypoint.subscribe(self._on_waypoint)
        self.terrain_map.subscribe(self._on_terrain)

        try:
            import rerun as rr  # type: ignore[import-untyped]
            self._rr = rr

            rr.init("LingTu Navigation", spawn=False)
            # Start the gRPC log sink, then serve the web viewer that connects to it.
            # rerun >= 0.30: serve_grpc() is the log sink; serve_web_viewer() hosts the UI.
            grpc_uri = rr.serve_grpc()
            rr.serve_web_viewer(connect_to=grpc_uri, web_port=self._port, open_browser=False)

            # Declare coordinate system: right-hand, Z-up (standard robotics).
            rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

            # Draw a static ground-reference grid.
            self._log_ground_grid()

            logger.info(
                "RerunVisualizerModule: serving at http://0.0.0.0:%d", self._port
            )
        except ImportError:
            logger.warning(
                "RerunVisualizerModule: rerun-sdk not installed — module is a no-op. "
                "Run: pip install rerun-sdk"
            )

    def stop(self) -> None:
        super().stop()
        logger.info("RerunVisualizerModule: stopped")

    # -- internal helpers ----------------------------------------------------

    def _log_ground_grid(self) -> None:
        """Log a static ground reference grid as line strips."""
        rr = self._rr
        half = self._GRID_HALF
        segments: list[list[list[float]]] = []

        for i in range(-half, half + 1):
            # Vertical lines (constant x)
            segments.append([[i, -half, 0.0], [i, half, 0.0]])
            # Horizontal lines (constant y)
            segments.append([[-half, i, 0.0], [half, i, 0.0]])

        rr.log(
            "world/grid",
            rr.LineStrips3D(
                np.array(segments, dtype=np.float32),
                colors=[[100, 100, 100, 80]] * len(segments),
            ),
            static=True,
        )

    # -- callbacks -----------------------------------------------------------

    def _on_odom(self, odom: Odometry) -> None:
        if self._rr is None:
            return
        rr = self._rr

        x = float(odom.x)
        y = float(odom.y)
        z = float(odom.z)
        robot_z = z + 0.15   # small offset so marker floats above ground

        # Robot body marker — green sphere.
        rr.log(
            "world/robot/position",
            rr.Points3D([[x, y, robot_z]], colors=[[0, 255, 0]], radii=[0.08]),
        )

        # Heading arrow derived from yaw.
        yaw = float(odom.yaw)
        dx = math.cos(yaw) * 0.4
        dy = math.sin(yaw) * 0.4
        rr.log(
            "world/robot/heading",
            rr.Arrows3D(
                origins=[[x, y, robot_z]],
                vectors=[[dx, dy, 0.0]],
                colors=[[0, 255, 0]],
            ),
        )

        # Accumulate trail; cap at _MAX_TRAIL to bound memory.
        self._trail.append([x, y, z + 0.02])
        if len(self._trail) > self._MAX_TRAIL:
            self._trail = self._trail[-self._MAX_TRAIL:]

        if len(self._trail) > 1:
            rr.log(
                "world/trail",
                rr.LineStrips3D(
                    [np.array(self._trail, dtype=np.float32)],
                    colors=[[0, 100, 255, 180]],
                ),
            )

        self._frame_count += 1

    def _on_cloud(self, cloud: PointCloud) -> None:
        if self._rr is None:
            return
        if cloud.is_empty:
            return

        pts = cloud.points[:, :3]   # always use XYZ columns only

        # Color by height: red channel = high, green channel = low.
        z = pts[:, 2]
        z_range = float(z.max() - z.min())
        z_norm = np.clip((z - z.min()) / max(z_range, 0.01), 0.0, 1.0)

        colors = np.zeros((len(pts), 3), dtype=np.uint8)
        colors[:, 0] = (255 * z_norm).astype(np.uint8)        # R — high
        colors[:, 1] = (100 * (1.0 - z_norm)).astype(np.uint8)  # G — low
        colors[:, 2] = 50                                       # B — constant

        self._rr.log(
            "world/lidar",
            self._rr.Points3D(pts, colors=colors, radii=[0.02]),
        )

    def _on_cmd_vel(self, twist: Twist) -> None:
        if self._rr is None:
            return
        vx = float(twist.linear.x)
        wz = float(twist.angular.z)
        self._rr.log("cmd_vel/vx", self._rr.Scalar(vx))
        self._rr.log("cmd_vel/wz", self._rr.Scalar(wz))

    def _on_path(self, path: Path) -> None:
        if self._rr is None:
            return
        if not path:
            return

        # Extract XYZ from each PoseStamped in the Path.
        pts = np.array(
            [[float(ps.pose.position.x), float(ps.pose.position.y), 0.05]
             for ps in path],
            dtype=np.float32,
        )

        self._rr.log(
            "world/global_path",
            self._rr.LineStrips3D([pts], colors=[[0, 255, 255]], radii=[0.03]),
        )

    def _on_waypoint(self, wp: PoseStamped) -> None:
        if self._rr is None:
            return
        x = float(wp.pose.position.x)
        y = float(wp.pose.position.y)
        self._rr.log(
            "world/waypoint",
            self._rr.Points3D([[x, y, 0.1]], colors=[[255, 255, 0]], radii=[0.12]),
        )

    def _on_terrain(self, cloud: PointCloud) -> None:
        if self._rr is None:
            return
        if cloud.is_empty:
            return
        pts = cloud.points[:, :3]
        self._rr.log(
            "world/terrain",
            self._rr.Points3D(pts, colors=[[100, 200, 100, 120]], radii=[0.04]),
        )

    # -- health --------------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        return {
            "rerun": self._rr is not None,
            "port": self._port,
            "frames": self._frame_count,
            "trail_len": len(self._trail),
        }
