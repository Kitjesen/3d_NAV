"""SLAMModule — LiDAR-inertial SLAM as a pluggable Module.

Produces odometry + map point cloud from LiDAR + IMU.

Backends:
  "fastlio2"  — Fast-LIO2 (C++ NativeModule, default)
  "pointlio"  — Point-LIO (C++ NativeModule, alternative)

Usage::

    bp.add(SLAMModule, backend="fastlio2")
    # Out: odometry, map_cloud
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from core.module import Module
from core.stream import In, Out
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud
from core.registry import register

logger = logging.getLogger(__name__)


@register("slam", "fastlio2", description="Fast-LIO2 LiDAR-inertial SLAM")
@register("slam", "pointlio", description="Point-LIO alternative SLAM")
class SLAMModule(Module, layer=1):
    """LiDAR-inertial SLAM — produces odometry + map.

    Both backends are C++ executables managed via NativeModule.
    """

    # -- Outputs --
    odometry: Out[Odometry]
    map_cloud: Out[PointCloud]
    alive: Out[bool]

    def __init__(self, backend: str = "fastlio2", **kw):
        super().__init__(**kw)
        self._backend = backend
        self._node = None

    def setup(self):
        if self._backend == "fastlio2":
            self._setup_fastlio2()
        elif self._backend == "pointlio":
            self._setup_pointlio()
        else:
            raise ValueError(f"Unknown SLAM backend: {self._backend}. Available: fastlio2, pointlio")

    def _setup_fastlio2(self):
        try:
            from core.config import get_config
            from core.native_factories import slam_fastlio2
            cfg = get_config()
            self._node = slam_fastlio2(cfg)
            self._node.setup()
        except (ImportError, FileNotFoundError, PermissionError) as e:
            logger.warning("SLAMModule [fastlio2]: not available: %s", e)

    def _setup_pointlio(self):
        try:
            from core.config import get_config
            from core.native_factories import slam_pointlio
            cfg = get_config()
            self._node = slam_pointlio(cfg)
            self._node.setup()
        except (ImportError, FileNotFoundError, PermissionError) as e:
            logger.warning("SLAMModule [pointlio]: not available: %s", e)

    def start(self):
        super().start()
        if self._node:
            try:
                self._node.start()
                logger.info("SLAMModule [%s]: C++ node started", self._backend)
            except Exception as e:
                logger.error("SLAMModule: start failed: %s", e)
        self.alive.publish(self._node is not None)

    def stop(self):
        if self._node:
            try:
                self._node.stop()
            except Exception:
                pass
            self._node = None
        self.alive.publish(False)
        super().stop()

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        node_info = {}
        if self._node:
            h = self._node.health()
            native = h.get("native", {})
            node_info = {
                "running": native.get("running", False),
                "pid": native.get("pid"),
                "restarts": native.get("restarts", 0),
            }
        info["slam"] = {
            "backend": self._backend,
            "node": node_info,
        }
        return info
