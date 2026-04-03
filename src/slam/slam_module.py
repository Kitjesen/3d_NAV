"""SLAMModule — C++ SLAM subprocess lifecycle manager.

Manages C++ SLAM executables (Fast-LIO2, Point-LIO, Localizer) as NativeModule
subprocesses. Does NOT produce data directly — SLAM data flows through
SlamBridgeModule which subscribes to DDS topics published by the C++ nodes.

Architecture:
    SLAMModule (this)          → starts/stops C++ subprocesses
    SlamBridgeModule (separate) → subscribes to DDS topics → Out[odometry], Out[map_cloud]

Backends:
  "fastlio2"   — Livox driver + Fast-LIO2 SLAM + PGO (mapping mode)
  "pointlio"   — Point-LIO SLAM (alternative)
  "localizer"  — Livox driver + Fast-LIO2 + ICP Localizer (navigation mode)

Usage::

    bp.add(SLAMModule, backend="fastlio2")   # mapping mode
    bp.add(SLAMModule, backend="localizer")  # navigation mode (has map)
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from core.module import Module
from core.stream import Out
from core.registry import register

logger = logging.getLogger(__name__)


@register("slam", "fastlio2", description="Fast-LIO2 LiDAR-inertial SLAM")
@register("slam", "pointlio", description="Point-LIO alternative SLAM")
@register("slam", "localizer", description="ICP localizer against pre-built map")
class SLAMModule(Module, layer=1):
    """C++ SLAM subprocess lifecycle manager.

    Starts/stops C++ SLAM executables via NativeModule. Data output is handled
    by SlamBridgeModule (DDS subscription), not by this module's ports.
    """

    alive: Out[bool]

    def __init__(self, backend: str = "fastlio2", **kw):
        super().__init__(**kw)
        self._backend = backend
        self._node = None
        self._lio_node = None
        self._pgo_node = None
        self._lidar_node = None

    def setup(self):
        if self._backend == "fastlio2":
            self._setup_fastlio2()
        elif self._backend == "pointlio":
            self._setup_pointlio()
        elif self._backend == "localizer":
            self._setup_localizer()
        else:
            raise ValueError(f"Unknown SLAM backend: {self._backend}. "
                             f"Available: fastlio2, pointlio, localizer")

    def _setup_lidar_driver(self):
        """Start Livox LiDAR driver (shared by all SLAM modes)."""
        try:
            from core.config import get_config
            from core.native_factories import livox_driver
            cfg = get_config()
            self._lidar_node = livox_driver(cfg)
            self._lidar_node.setup()
        except (ImportError, FileNotFoundError, PermissionError) as e:
            logger.warning("SLAMModule: Livox driver not available: %s", e)

    def _setup_fastlio2(self):
        """Livox driver + Fast-LIO2 SLAM + PGO."""
        self._setup_lidar_driver()
        try:
            from core.config import get_config
            from core.native_factories import slam_fastlio2, slam_pgo
            cfg = get_config()
            self._node = slam_fastlio2(cfg)
            self._node.setup()
            self._pgo_node = slam_pgo(cfg)
            self._pgo_node.setup()
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
        """Livox driver + Fast-LIO2 (odometry source) + ICP localizer (map matching)."""
        self._setup_lidar_driver()
        try:
            from core.config import get_config
            from core.native_factories import slam_fastlio2, slam_localizer
            cfg = get_config()
            self._lio_node = slam_fastlio2(cfg)
            self._lio_node.setup()
            self._node = slam_localizer(cfg)
            self._node.setup()
        except (ImportError, FileNotFoundError, PermissionError) as e:
            logger.warning("SLAMModule [localizer]: not available: %s", e)

    def start(self):
        super().start()
        started = 0
        for name, node in [("Livox driver", self._lidar_node),
                           ("Fast-LIO2 companion", self._lio_node),
                           ("PGO", self._pgo_node),
                           (self._backend, self._node)]:
            if node:
                try:
                    node.start()
                    started += 1
                    logger.info("SLAMModule: %s started", name)
                except Exception as e:
                    logger.error("SLAMModule: %s start failed: %s", name, e)
        self.alive.publish(self._node is not None)

    def stop(self):
        for node in [self._node, self._pgo_node, self._lio_node, self._lidar_node]:
            if node:
                try:
                    node.stop()
                except Exception:
                    pass
        self._node = None
        self._pgo_node = None
        self._lio_node = None
        self._lidar_node = None
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
