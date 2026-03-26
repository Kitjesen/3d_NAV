"""TerrainModule — traversability analysis as a pluggable Module.

Analyzes point cloud + odometry to produce terrain maps for local planning.

Backends:
  "cmu"    — C++ terrain_analysis + terrain_analysis_ext (NativeModule, S100P)
  "simple" — Python passthrough (testing, no analysis)

Usage::

    bp.add(TerrainModule, backend="cmu")
    # auto_wire connects odometry, map_cloud → terrain_map, traversability
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


@register("terrain", "cmu", description="CMU-style C++ terrain analysis (NativeModule)")
class TerrainModule(Module, layer=2):
    """Traversability analysis — ground estimation + obstacle detection.

    "cmu" backend: manages 2 C++ NativeModule processes
      - terrain_analysis: ground/obstacle classification
      - terrain_analysis_ext: connectivity + 2.5D height map

    "simple" backend: passthrough for testing (no actual analysis)
    """

    # -- Inputs --
    odometry: In[Odometry]
    map_cloud: In[PointCloud]

    # -- Outputs --
    terrain_map: Out[PointCloud]
    traversability: Out[dict]
    alive: Out[bool]

    def __init__(self, backend: str = "cmu", **kw):
        super().__init__(**kw)
        self._backend = backend
        self._nodes: Dict[str, Any] = {}

    def setup(self):
        self.odometry.subscribe(self._on_odom)
        self.map_cloud.subscribe(self._on_cloud)

        if self._backend == "cmu":
            self._setup_cmu()
        else:
            logger.info("TerrainModule: simple backend (passthrough)")

    def _setup_cmu(self):
        """Setup C++ NativeModule backends."""
        try:
            from core.config import get_config
            from core.native_module import NativeModule
            from core.native_factories import terrain_analysis, terrain_analysis_ext

            cfg = get_config()
            self._nodes = {
                "terrain": terrain_analysis(cfg),
                "terrain_ext": terrain_analysis_ext(cfg),
            }
            for name, node in self._nodes.items():
                try:
                    node.setup()
                except (FileNotFoundError, PermissionError) as e:
                    logger.warning("TerrainModule: %s setup failed: %s", name, e)
        except ImportError as e:
            logger.warning("TerrainModule: C++ backend not available: %s", e)

    def start(self):
        super().start()
        started = 0
        for name, node in self._nodes.items():
            try:
                node.start()
                started += 1
            except Exception as e:
                logger.error("TerrainModule: %s start failed: %s", name, e)
        self.alive.publish(started == len(self._nodes) or self._backend != "cmu")
        logger.info("TerrainModule [%s]: %d/%d nodes started",
                     self._backend, started, len(self._nodes))

    def stop(self):
        for name, node in reversed(list(self._nodes.items())):
            try:
                node.stop()
            except Exception as e:
                logger.warning("TerrainModule: %s stop error: %s", name, e)
        self._nodes.clear()
        self.alive.publish(False)
        super().stop()

    def _on_odom(self, odom: Odometry):
        """Forward odometry (simple backend publishes empty traversability)."""
        if self._backend == "simple":
            self.traversability.publish({"status": "passthrough"})

    def _on_cloud(self, cloud: PointCloud):
        """Forward point cloud (simple backend passes through as terrain_map)."""
        if self._backend == "simple":
            self.terrain_map.publish(cloud)

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        node_health = {}
        for name, node in self._nodes.items():
            h = node.health()
            native = h.get("native", {})
            node_health[name] = {
                "running": native.get("running", False),
                "pid": native.get("pid"),
            }
        info["terrain"] = {
            "backend": self._backend,
            "nodes": node_health,
        }
        return info
