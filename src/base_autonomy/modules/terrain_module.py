"""TerrainModule — traversability analysis as a pluggable Module.

Analyzes point cloud + odometry to produce terrain maps for local planning.

Backends:
  "nanobind" — C++ terrain_core via nanobind (zero-copy, GIL-released, preferred)
  "native"   — C++ NativeModule subprocess (legacy, S100P fallback)
  "simple"   — Python passthrough (testing, no analysis)

Usage::

    bp.add(TerrainModule, backend="nanobind")
    # In: odometry, map_cloud → Out: terrain_map, traversability, elevation_map
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict

import numpy as np

from base_autonomy.modules._nav_core_loader import nav_core_build_hint, try_import_nav_core
from core.module import Module
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud2
from core.registry import register
from core.stream import In, Out

logger = logging.getLogger(__name__)


@register("terrain", "nanobind", description="C++ terrain analysis via nanobind (zero-copy)")
@register("terrain", "native", description="C++ terrain analysis via NativeModule subprocess")
@register("terrain", "simple", description="Passthrough for testing")
class TerrainModule(Module, layer=2):
    """Traversability analysis — ground estimation + obstacle detection.

    "nanobind": C++ TerrainAnalysisCore called directly from Python.
      Data flows through In/Out ports. Zero-copy numpy. GIL released during C++.
    "native": Legacy NativeModule subprocess (C++ process + DDS).
    "simple": Passthrough for testing.
    """

    # -- Inputs --
    odometry: In[Odometry]
    map_cloud: In[PointCloud2]

    # -- Outputs --
    terrain_map: Out[PointCloud2]
    traversability: Out[dict]
    elevation_map: Out[np.ndarray]
    alive: Out[bool]

    def __init__(self, backend: str = "nanobind", **kw):
        super().__init__(**kw)
        self._backend = backend
        self._core = None       # nanobind: TerrainAnalysisCore
        self._nodes: dict[str, Any] = {}  # native: NativeModule dict
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_z = 0.0
        self._odom_yaw = 0.0

    def setup(self):
        self.odometry.subscribe(self._on_odom)
        self.map_cloud.subscribe(self._on_cloud)

        if self._backend == "nanobind":
            self._setup_nanobind()
        elif self._backend == "native":
            self._setup_native()
        else:
            logger.info("TerrainModule: simple backend (passthrough)")

    def _setup_nanobind(self):
        """Setup C++ terrain_core via nanobind binding."""
        _nav_core = try_import_nav_core()
        if _nav_core is None:
            logger.info(
                "TerrainModule: _nav_core.so not found — using simple backend.\n"
                "  To enable C++ terrain analysis:\n  %s", nav_core_build_hint()
            )
            self._backend = "simple"
            return
        try:
            params = _nav_core.TerrainParams()
            # Load from robot config if available
            try:
                from core.config import get_config
                cfg = get_config()
                ta = cfg.raw.get("terrain_analysis", {})
                if ta:
                    for attr in ["scan_voxel_size", "decay_time", "no_decay_dis",
                                 "obstacle_height_thre", "vehicle_height",
                                 "min_rel_z", "max_rel_z"]:
                        if attr in ta:
                            setattr(params, attr, ta[attr])
            except ImportError:
                pass

            self._core = _nav_core.TerrainAnalysisCore(params)
            logger.info("TerrainModule [nanobind]: C++ terrain_core loaded")
        except Exception as e:
            logger.warning("TerrainModule: _nav_core error: %s — using simple backend", e)
            self._backend = "simple"

    def _setup_native(self):
        """Setup C++ NativeModule backends (legacy)."""
        try:
            from base_autonomy.native_factories import terrain_analysis, terrain_analysis_ext
            from core.config import get_config
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
            logger.warning("TerrainModule: native backend not available: %s", e)

    def start(self):
        super().start()
        if self._backend == "native":
            started = 0
            for name, node in self._nodes.items():
                try:
                    node.start()
                    started += 1
                except Exception as e:
                    logger.error("TerrainModule: %s start failed: %s", name, e)
            self.alive.publish(started == len(self._nodes))
        else:
            self.alive.publish(True)
        logger.info("TerrainModule [%s]: started", self._backend)

    def stop(self):
        for name, node in reversed(list(self._nodes.items())):
            try:
                node.stop()
            except Exception:
                pass
        self._nodes.clear()
        if self._core:
            self._core.clear()
            self._core = None
        self.alive.publish(False)
        super().stop()

    def _on_odom(self, odom: Odometry):
        self._odom_x = odom.x
        self._odom_y = odom.y
        self._odom_z = getattr(odom, 'z', 0.0)
        self._odom_yaw = odom.yaw

        if self._core:
            # Feed odometry to C++ core (roll/pitch = 0 for ground robot)
            self._core.update_vehicle(
                odom.x, odom.y, self._odom_z,
                0.0, 0.0, odom.yaw)

        if self._backend == "simple":
            self.traversability.publish({"status": "passthrough"})

    def _on_cloud(self, cloud: PointCloud2):
        if self._backend == "simple":
            self.terrain_map.publish(cloud)
            return

        if self._backend == "nanobind" and self._core is not None:
            self._process_nanobind(cloud)

    def _process_nanobind(self, cloud: PointCloud2):
        """Process point cloud through C++ terrain_core. GIL released during C++."""
        pts = cloud.points  # numpy Nx3 or Nx4
        if pts is None or len(pts) == 0:
            return

        # Ensure Nx4 float32 (x, y, z, intensity)
        if pts.ndim == 2 and pts.shape[1] == 3:
            pts4 = np.zeros((len(pts), 4), dtype=np.float32)
            pts4[:, :3] = pts
        elif pts.ndim == 2 and pts.shape[1] >= 4:
            pts4 = pts[:, :4].astype(np.float32, copy=False)
        else:
            return

        # Flatten to 1D for C++ (Nx4 → 4N flat)
        flat = pts4.ravel().tolist()  # TODO: zero-copy via nb::ndarray when available
        ts = getattr(cloud, 'ts', time.time())

        result = self._core.process(flat, ts)

        # Publish terrain cloud
        if result.n_points > 0:
            arr = np.array(result.terrain_points, dtype=np.float32).reshape(-1, 4)
            self.terrain_map.publish(PointCloud2(
                points=arr[:, :3],
                frame_id="map",
                ts=ts,
            ))
            self.traversability.publish({
                "n_obstacles": result.n_points,
                "map_width": result.map_width,
                "map_resolution": result.map_resolution,
            })

        # Publish elevation map
        if result.map_width > 0:
            elev = np.array(result.elevation_map, dtype=np.float32).reshape(
                result.map_width, result.map_width)
            self.elevation_map.publish(elev)

    def health(self) -> dict[str, Any]:
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
            "has_core": self._core is not None,
            "nodes": node_health,
        }
        return info
