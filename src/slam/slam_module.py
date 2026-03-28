"""SLAMModule — LiDAR-inertial SLAM / localization as a pluggable Module.

Produces odometry + map point cloud from LiDAR + IMU.

Backends:
  "fastlio2"   — Fast-LIO2 SLAM (build map + localize simultaneously)
  "pointlio"   — Point-LIO SLAM (alternative)
  "localizer"  — ICP Localizer (localize against pre-built map, no mapping)

Usage::

    bp.add(SLAMModule, backend="fastlio2")   # mapping mode
    bp.add(SLAMModule, backend="localizer")  # navigation mode (has map)
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
@register("slam", "localizer", description="ICP localizer against pre-built map")
class SLAMModule(Module, layer=1):
    """LiDAR-inertial SLAM / localization — produces odometry + map.

    All backends are C++ executables managed via NativeModule.
    """

    # -- Outputs --
    odometry: Out[Odometry]
    map_cloud: Out[PointCloud]
    alive: Out[bool]

    def __init__(self, backend: str = "fastlio2", **kw):
        super().__init__(**kw)
        self._backend = backend
        self._node = None
        self._lio_node = None  # localizer mode: Fast-LIO2 companion

    def setup(self):
        if self._backend == "fastlio2":
            self._setup_fastlio2()
        elif self._backend == "pointlio":
            self._setup_pointlio()
        elif self._backend == "localizer":
            self._setup_localizer()
        else:
            raise ValueError(f"Unknown SLAM backend: {self._backend}. Available: fastlio2, pointlio, localizer")

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

    def _setup_localizer(self):
        """Localizer needs Fast-LIO2 (odometry source) + localizer_node (ICP map matching)."""
        try:
            from core.config import get_config
            from core.native_factories import slam_fastlio2, slam_localizer
            cfg = get_config()
            # Fast-LIO2 provides /cloud_registered + /Odometry that localizer subscribes to
            self._lio_node = slam_fastlio2(cfg)
            self._lio_node.setup()
            self._node = slam_localizer(cfg)
            self._node.setup()
        except (ImportError, FileNotFoundError, PermissionError) as e:
            logger.warning("SLAMModule [localizer]: not available: %s", e)

    def start(self):
        super().start()
        if self._lio_node:
            try:
                self._lio_node.start()
                logger.info("SLAMModule [localizer]: Fast-LIO2 companion started")
            except Exception as e:
                logger.error("SLAMModule: Fast-LIO2 companion start failed: %s", e)
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
        if self._lio_node:
            try:
                self._lio_node.stop()
            except Exception:
                pass
            self._lio_node = None
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
