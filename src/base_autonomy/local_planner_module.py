"""LocalPlannerModule — local path planning as a pluggable Module.

Takes terrain map + waypoint + odometry, produces obstacle-free local path.

Backends:
  "cmu"  — C++ local_planner (NativeModule, obstacle avoidance + path scoring)
  "simple" — straight-line path for testing

Usage::

    bp.add(LocalPlannerModule, backend="cmu")
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

import numpy as np

from core.module import Module
from core.stream import In, Out
from core.msgs.nav import Odometry, Path
from core.msgs.geometry import PoseStamped, Pose, Vector3, Quaternion
from core.msgs.sensor import PointCloud
from core.registry import register

logger = logging.getLogger(__name__)


@register("local_planner", "cmu", description="CMU-style C++ local planner (NativeModule)")
class LocalPlannerModule(Module, layer=2):
    """Local path planning — obstacle avoidance + path scoring.

    "cmu" backend: C++ local_planner NativeModule
    "simple" backend: straight-line to waypoint (testing)
    """

    # -- Inputs --
    odometry: In[Odometry]
    terrain_map: In[PointCloud]
    waypoint: In[PoseStamped]

    # -- Outputs --
    local_path: Out[Path]
    alive: Out[bool]

    def __init__(self, backend: str = "cmu", **kw):
        super().__init__(**kw)
        self._backend = backend
        self._node = None
        self._robot_pos = np.zeros(3)
        self._latest_waypoint = None

    def setup(self):
        self.odometry.subscribe(self._on_odom)
        self.terrain_map.subscribe(self._on_terrain)
        self.waypoint.subscribe(self._on_waypoint)

        if self._backend == "cmu":
            self._setup_cmu()
        else:
            logger.info("LocalPlannerModule: simple backend (straight-line)")

    def _setup_cmu(self):
        try:
            from core.config import get_config
            from core.native_factories import local_planner

            cfg = get_config()
            self._node = local_planner(cfg)
            try:
                self._node.setup()
            except (FileNotFoundError, PermissionError) as e:
                logger.warning("LocalPlannerModule: setup failed: %s", e)
                self._node = None
        except ImportError as e:
            logger.warning("LocalPlannerModule: C++ backend not available: %s", e)

    def start(self):
        super().start()
        if self._node:
            try:
                self._node.start()
                logger.info("LocalPlannerModule [cmu]: C++ node started")
            except Exception as e:
                logger.error("LocalPlannerModule: start failed: %s", e)
        self.alive.publish(True)

    def stop(self):
        if self._node:
            try:
                self._node.stop()
            except Exception:
                pass
            self._node = None
        self.alive.publish(False)
        super().stop()

    def _on_odom(self, odom: Odometry):
        self._robot_pos = np.array([odom.x, odom.y, getattr(odom, 'z', 0.0)])

    def _on_terrain(self, cloud: PointCloud):
        """Terrain map received — C++ backend handles internally via DDS."""
        pass

    def _on_waypoint(self, wp: PoseStamped):
        """Waypoint received — simple backend generates straight-line path."""
        self._latest_waypoint = wp
        if self._backend == "simple" and wp is not None:
            goal = np.array([wp.pose.position.x, wp.pose.position.y, 0.0])
            path_points = self._straight_line(self._robot_pos, goal, step=0.5)
            poses = [PoseStamped(
                pose=Pose(position=Vector3(p[0], p[1], p[2]),
                          orientation=Quaternion(0, 0, 0, 1)),
                frame_id="map",
            ) for p in path_points]
            self.local_path.publish(Path(poses=poses))

    def _straight_line(self, start, goal, step=0.5):
        diff = goal - start
        dist = np.linalg.norm(diff)
        if dist < step:
            return [goal]
        n = max(int(dist / step), 2)
        return [start + diff * (i / n) for i in range(1, n + 1)]

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["local_planner"] = {
            "backend": self._backend,
            "running": self._node is not None and getattr(self._node, '_process', None) is not None,
        }
        return info
