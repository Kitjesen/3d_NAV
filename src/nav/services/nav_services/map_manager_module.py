"""MapManagerModule -- hive Module version of MapManager.

Manages map listing/saving/deleting/renaming/activation and POI CRUD.

Maps are stored as directories under map_dir:

    ~/data/inovxio/data/maps/
    ├── active -> building_2f/     (symlink pointing to the active map dir)
    ├── building_2f/
    │   ├── map.pcd                (SLAM point cloud)
    │   ├── tomogram.pickle        (global planning map, auto-generated)
    │   └── occupancy.npz          (2D static occupancy grid, auto-generated)
    └── warehouse/
        ├── map.pcd
        ├── tomogram.pickle
        └── occupancy.npz

Ports:
    In:  map_command (str)   -- JSON command string
    Out: map_response (dict) -- operation result dict
"""

from __future__ import annotations

import json
import logging
import os
import shutil
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from core import In, Module, Out, skill
from nav.services.nav_services.yaml_helpers import load_yaml, save_yaml

logger = logging.getLogger(__name__)


class MapManagerModule(Module, layer=6):
    """Map and POI management module (hive Module).

    Maps are stored as subdirectories.  Each map directory contains:
        map.pcd          -- SLAM point cloud saved via ROS2 SaveMaps service
        tomogram.pickle  -- global planning index auto-built from map.pcd
        occupancy.npz    -- static 2D occupancy grid derived from map.pcd
                           keys: grid (int8 H×W, 0=free/100=occupied),
                                 resolution (float, metres/cell),
                                 origin (float[2], world XY of grid[0,0])

    Local planning note: LocalPlannerModule, TerrainModule, ElevationMapModule,
    and OccupancyGridModule are all online-only.  They subscribe to the live
    /nav/map_cloud LiDAR topic at runtime and rebuild their maps on each frame.
    No persistent artifact is needed or read by those modules at startup.

    The ``active`` entry is a symlink inside map_dir pointing to the
    currently selected map directory.
    """

    map_command: In[str]
    map_response: Out[dict]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        default_data_dir = config.get(
            "data_dir", os.path.expanduser("~/.lingtu")
        )
        default_map_dir = config.get(
            "map_dir",
            os.environ.get(
                "NAV_MAP_DIR",
                os.path.expanduser("~/data/inovxio/data/maps"),
            ),
        )
        self._data_dir = Path(default_data_dir)
        self._map_dir = Path(default_map_dir)
        self._data_dir.mkdir(parents=True, exist_ok=True)
        self._map_dir.mkdir(parents=True, exist_ok=True)

        self._poi_file = self._data_dir / "pois.yaml"
        self._active_map_file = self._data_dir / "active_map.yaml"

        self._pois: dict[str, dict[str, float]] = load_yaml(
            self._poi_file, default={}
        )
        self._active_map: str = load_yaml(
            self._active_map_file, default={}
        ).get("active", "")

    def setup(self) -> None:
        self.map_command.subscribe(self._on_command)

    # -- command dispatch -------------------------------------------------------

    def _on_command(self, raw: str) -> None:
        """Parse JSON command and dispatch to handler."""
        try:
            cmd = json.loads(raw) if isinstance(raw, str) else raw
        except (json.JSONDecodeError, TypeError):
            cmd = {}

        action = cmd.get("action", "")
        resp: dict[str, Any] = {"action": action, "success": False}

        try:
            if action == "list":
                resp = self._map_list()
            elif action == "save":
                resp = self._map_save(cmd.get("name", ""))
            elif action == "delete":
                resp = self._map_delete(cmd.get("name", ""))
            elif action == "rename":
                resp = self._map_rename(
                    cmd.get("name", ""), cmd.get("new_name", "")
                )
            elif action == "set_active":
                resp = self._map_set_active(cmd.get("name", ""))
            elif action == "build_tomogram":
                resp = self._build_tomogram(cmd.get("name", ""))
            elif action == "poi_set":
                resp = self._poi_set(cmd)
            elif action == "poi_delete":
                resp = self._poi_delete(cmd.get("name", ""))
            elif action == "poi_list":
                resp = self._poi_list()
            else:
                resp["message"] = f"unknown action: {action}"
        except Exception as exc:
            resp["message"] = str(exc)

        self.map_response.publish(resp)

    # -- map operations ---------------------------------------------------------

    def _map_list(self) -> dict[str, Any]:
        """List map directories (exclude the 'active' symlink entry)."""
        maps: list[dict[str, Any]] = []
        for entry in sorted(self._map_dir.iterdir()):
            if not entry.is_dir() or entry.name == "active":
                continue
            maps.append(
                {
                    "name": entry.name,
                    "has_pcd": (entry / "map.pcd").exists(),
                    "has_tomogram": (entry / "tomogram.pickle").exists(),
                    "has_occupancy": (entry / "occupancy.npz").exists(),
                }
            )
        return {
            "action": "list",
            "success": True,
            "maps": maps,
            "active": self._active_map,
        }

    def _map_save(self, name: str) -> dict[str, Any]:
        """Save SLAM map via ROS2 SaveMaps service, then build all planning artifacts.

        Pipeline
        --------
        1. ROS2 ``/pgo/save_maps`` → ``<map_dir>/<name>/map.pcd``
           Raw SLAM point cloud; required for all downstream steps.

        2. ``_build_tomogram(name)`` → ``<map_dir>/<name>/tomogram.pickle``
           PCT global planner (ele_planner.so) and A* backend both load this
           file at startup.  This step is required; failure blocks the save.

        3. ``_build_occupancy_snapshot(name)`` → ``<map_dir>/<name>/occupancy.npz``
           Static 2D occupancy grid derived from map.pcd (numpy-only, no ROS2).
           Stored as npz with keys: ``grid`` (int8 H×W, 0=free/100=occupied),
           ``resolution`` (float, metres per cell), ``origin`` (float[2],
           world-frame XY of grid[0,0]).
           Used by: A* backend as a static planning grid before the first live
           costmap arrives; Dashboard for map tile rendering.
           This step is optional — failure is logged but does NOT block the save.

        Local planning note
        -------------------
        LocalPlannerModule and TerrainModule are online-only: they subscribe to
        live /nav/map_cloud (LiDAR point cloud) at runtime and rebuild their
        terrain_map on every frame.  No persistent artifact is needed or read by
        those modules at startup.  ElevationMapModule and OccupancyGridModule are
        likewise online-only.
        """
        if not name:
            return {"action": "save", "success": False, "message": "missing map name"}

        map_dir = self._map_dir / name
        map_dir.mkdir(parents=True, exist_ok=True)
        pcd_path = map_dir / "map.pcd"

        # Step 1 — Call the ROS2 SLAM save_maps service
        try:
            import subprocess

            result = subprocess.run(
                [
                    "ros2",
                    "service",
                    "call",
                    "/pgo/save_maps",
                    "interface/srv/SaveMaps",
                    f"{{file_path: '{pcd_path}'}}",
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if result.returncode != 0:
                return {
                    "action": "save",
                    "success": False,
                    "message": f"SaveMaps failed: {result.stderr}",
                }
        except FileNotFoundError as e:
            return {
                "action": "save",
                "success": False,
                "message": f"ROS2 not available: {e}",
            }
        except subprocess.TimeoutExpired as e:
            return {
                "action": "save",
                "success": False,
                "message": f"ROS2 not available: {e}",
            }

        # Step 2 — Auto-build tomogram (required for global planner)
        tomo_result = self._build_tomogram(name)

        # Step 3 — Build static 2D occupancy snapshot (optional, non-blocking)
        occ_result = self._build_occupancy_snapshot(name)

        resp: dict[str, Any] = {
            "action": "save",
            "success": True,
            "map_dir": str(map_dir),
            "pcd": str(pcd_path),
            "tomogram": tomo_result.get("tomogram"),
            "tomogram_ok": tomo_result.get("success", False),
            "occupancy": occ_result.get("occupancy"),
            "occupancy_ok": occ_result.get("success", False),
        }
        if not occ_result.get("success"):
            resp["occupancy_message"] = occ_result.get("message", "")
        return resp

    def _build_tomogram(self, name: str) -> dict[str, Any]:
        """Build tomogram.pickle from map.pcd using the PCT planner pipeline."""
        if not name:
            return {"action": "build_tomogram", "success": False, "message": "missing map name"}

        map_dir = self._map_dir / name
        pcd_path = map_dir / "map.pcd"
        tomogram_path = map_dir / "tomogram.pickle"

        if not pcd_path.exists():
            return {
                "action": "build_tomogram",
                "success": False,
                "message": f"no PCD file at {pcd_path}",
            }

        try:
            from global_planning.PCT_planner.tomography.scripts.build_tomogram import (
                build_tomogram_from_pcd,
            )

            build_tomogram_from_pcd(str(pcd_path), str(tomogram_path))
            return {
                "action": "build_tomogram",
                "success": True,
                "tomogram": str(tomogram_path),
            }
        except Exception as e:
            return {
                "action": "build_tomogram",
                "success": False,
                "message": str(e),
            }

    def _build_occupancy_snapshot(self, name: str) -> dict[str, Any]:
        """Build a static 2D occupancy grid from map.pcd and save as occupancy.npz.

        Algorithm
        ---------
        1. Load the PCD file (open3d if available, ASCII fallback otherwise).
        2. Filter points to obstacle-height range (0.10 m – 2.00 m above median
           ground, matching OccupancyGridModule defaults).
        3. Project XY into a fixed-resolution grid (0.20 m/cell).
        4. Mark occupied cells = 100, free = 0.
        5. Save as npz: grid (int8 H×W), resolution (float), origin (float[2]).

        This step is wrapped in try/except; any failure returns success=False and
        the caller (_map_save) continues without blocking the overall save.
        """
        if not name:
            return {"action": "build_occupancy_snapshot", "success": False, "message": "missing map name"}

        map_dir = self._map_dir / name
        pcd_path = map_dir / "map.pcd"
        occ_path = map_dir / "occupancy.npz"

        if not pcd_path.exists():
            return {
                "action": "build_occupancy_snapshot",
                "success": False,
                "message": f"no PCD file at {pcd_path}",
            }

        try:
            pts = self._load_pcd_points(str(pcd_path))
            if pts is None or pts.shape[0] == 0:
                return {
                    "action": "build_occupancy_snapshot",
                    "success": False,
                    "message": "PCD file is empty or could not be parsed",
                }

            resolution: float = 0.20  # metres per cell — matches OccupancyGridModule default
            z_min_rel: float = 0.10   # obstacle bottom threshold above ground
            z_max_rel: float = 2.00   # obstacle top threshold above ground

            # Estimate ground level from median of lowest percentile
            ground_z = float(np.percentile(pts[:, 2], 5))
            z_lo = ground_z + z_min_rel
            z_hi = ground_z + z_max_rel

            # Height filter — keep only obstacle-height band
            mask = (pts[:, 2] >= z_lo) & (pts[:, 2] <= z_hi)
            obs_pts = pts[mask, :2]

            if obs_pts.shape[0] == 0:
                # No obstacle-height points — save an empty 1×1 grid so the file exists
                grid = np.zeros((1, 1), dtype=np.int8)
                origin = np.array([0.0, 0.0], dtype=np.float64)
                np.savez_compressed(str(occ_path), grid=grid, resolution=resolution, origin=origin)
                return {
                    "action": "build_occupancy_snapshot",
                    "success": True,
                    "occupancy": str(occ_path),
                    "message": "no obstacle-height points; saved empty grid",
                }

            # Compute grid bounds from all XY points (use full cloud for extent)
            xy_all = pts[:, :2]
            xy_min = xy_all.min(axis=0)
            xy_max = xy_all.max(axis=0)

            # Add a small border (1 cell) so boundary obstacles are visible
            border = resolution
            origin = xy_min - border
            grid_w = int(np.ceil((xy_max[0] + border - origin[0]) / resolution)) + 1
            grid_h = int(np.ceil((xy_max[1] + border - origin[1]) / resolution)) + 1

            grid = np.zeros((grid_h, grid_w), dtype=np.int8)

            # Rasterise obstacle points
            col_idx = np.floor((obs_pts[:, 0] - origin[0]) / resolution).astype(np.int32)
            row_idx = np.floor((obs_pts[:, 1] - origin[1]) / resolution).astype(np.int32)
            valid = (col_idx >= 0) & (col_idx < grid_w) & (row_idx >= 0) & (row_idx < grid_h)
            grid[row_idx[valid], col_idx[valid]] = 100

            np.savez_compressed(
                str(occ_path),
                grid=grid,
                resolution=np.float64(resolution),
                origin=origin.astype(np.float64),
            )
            logger.info(
                "Occupancy snapshot saved: %s  shape=%s  res=%.2f  origin=(%.1f,%.1f)",
                occ_path, grid.shape, resolution, origin[0], origin[1],
            )
            return {
                "action": "build_occupancy_snapshot",
                "success": True,
                "occupancy": str(occ_path),
                "grid_shape": list(grid.shape),
                "resolution": resolution,
                "origin": origin.tolist(),
            }

        except Exception as exc:
            logger.warning("build_occupancy_snapshot failed for '%s': %s", name, exc)
            return {
                "action": "build_occupancy_snapshot",
                "success": False,
                "message": str(exc),
            }

    @staticmethod
    def _load_pcd_points(pcd_path: str) -> np.ndarray | None:
        """Load XYZ points from a PCD file.

        Tries open3d first (fast, handles binary/compressed PCD).
        Falls back to a numpy ASCII reader for plain-text PCD files.
        Returns float32 (N, 3) array or None on failure.
        """
        # Attempt 1: open3d (preferred — handles binary and compressed PCD)
        try:
            import open3d as o3d  # type: ignore
            cloud = o3d.io.read_point_cloud(pcd_path)
            pts = np.asarray(cloud.points, dtype=np.float32)
            if pts.shape[0] > 0:
                return pts
        except Exception:
            pass

        # Attempt 2: numpy ASCII fallback for plain-text PCD files
        try:
            pts_list: list[list[float]] = []
            in_data = False
            with open(pcd_path, errors="replace") as f:
                for line in f:
                    line = line.strip()
                    if not in_data:
                        if line.upper().startswith("DATA"):
                            data_fmt = line.split()[-1].lower() if len(line.split()) > 1 else ""
                            if data_fmt not in ("ascii", ""):
                                # Binary PCD without open3d — cannot parse
                                return None
                            in_data = True
                        continue
                    parts = line.split()
                    if len(parts) >= 3:
                        try:
                            pts_list.append([float(parts[0]), float(parts[1]), float(parts[2])])
                        except ValueError:
                            continue
            if pts_list:
                return np.array(pts_list, dtype=np.float32)
        except Exception:
            pass

        return None

    def _map_delete(self, name: str) -> dict[str, Any]:
        """Delete an entire map directory."""
        if not name:
            return {"action": "delete", "success": False, "message": "missing map name"}

        map_dir = self._map_dir / name
        if not map_dir.is_dir():
            return {
                "action": "delete",
                "success": False,
                "message": f"map not found: {name}",
            }

        try:
            shutil.rmtree(map_dir)
            if self._active_map == name:
                # Remove the dangling active symlink as well
                active_link = self._map_dir / "active"
                if active_link.is_symlink() or active_link.exists():
                    active_link.unlink()
                self._active_map = ""
                self._save_active_map()
            return {"action": "delete", "success": True, "message": f"deleted: {name}"}
        except OSError as exc:
            return {"action": "delete", "success": False, "message": str(exc)}

    def _map_rename(self, name: str, new_name: str) -> dict[str, Any]:
        """Rename a map directory; update active symlink if needed."""
        if not name or not new_name:
            return {"action": "rename", "success": False, "message": "missing name(s)"}

        src = self._map_dir / name
        dst = self._map_dir / new_name

        if not src.is_dir():
            return {
                "action": "rename",
                "success": False,
                "message": f"map not found: {name}",
            }
        if dst.exists():
            return {
                "action": "rename",
                "success": False,
                "message": f"target exists: {new_name}",
            }

        try:
            shutil.move(str(src), str(dst))
            if self._active_map == name:
                # Re-point the active symlink to the renamed directory
                self._active_map = new_name
                self._save_active_map()
                active_link = self._map_dir / "active"
                if active_link.is_symlink() or active_link.exists():
                    active_link.unlink()
                active_link.symlink_to(dst)
            return {
                "action": "rename",
                "success": True,
                "message": f"{name} -> {new_name}",
            }
        except OSError as exc:
            return {"action": "rename", "success": False, "message": str(exc)}

    def _map_set_active(self, name: str) -> dict[str, Any]:
        """Create/update the ``active`` symlink to point at the named map dir."""
        if not name:
            return {"action": "set_active", "success": False, "message": "missing map name"}

        map_dir = self._map_dir / name
        if not map_dir.is_dir():
            return {
                "action": "set_active",
                "success": False,
                "message": f"map not found: {name}",
            }

        active_link = self._map_dir / "active"
        if active_link.is_symlink() or active_link.exists():
            active_link.unlink()
        active_link.symlink_to(map_dir)

        self._active_map = name
        self._save_active_map()
        return {
            "action": "set_active",
            "success": True,
            "active": name,
            "tomogram": str(map_dir / "tomogram.pickle"),
        }

    def _save_active_map(self) -> None:
        save_yaml(self._active_map_file, {"active": self._active_map})

    # -- public helpers for other modules ---------------------------------------

    def get_active_tomogram(self) -> str | None:
        """Return the tomogram.pickle path for the active map, or None.

        NavigationModule uses this to locate the PCT planner index at startup.
        """
        active = self._map_dir / "active" / "tomogram.pickle"
        if active.exists():
            return str(active)
        return None

    def get_active_occupancy(self) -> str | None:
        """Return the occupancy.npz path for the active map, or None.

        A* backend and Dashboard may use this as a static planning grid.
        Load with: data = np.load(path); grid=data['grid'], res=data['resolution'],
        origin=data['origin'].
        """
        active = self._map_dir / "active" / "occupancy.npz"
        if active.exists():
            return str(active)
        return None

    # -- MCP @skill (same logic as map_command handlers, JSON string results) --

    @skill
    def list_maps(self) -> str:
        """List saved maps and which one is active."""
        return json.dumps(self._map_list(), default=str)

    @skill
    def save_map(self, name: str) -> str:
        """Save current SLAM map as *name* (ROS2 SaveMaps) and build all artifacts."""
        return json.dumps(self._map_save(name), default=str)

    @skill
    def use_map(self, name: str) -> str:
        """Activate *name* as the current map (symlink + planner index path)."""
        return json.dumps(self._map_set_active(name), default=str)

    @skill
    def build_tomogram(self, name: str) -> str:
        """Build tomogram.pickle from map.pcd for map *name*."""
        return json.dumps(self._build_tomogram(name), default=str)

    # -- POI operations ---------------------------------------------------------

    def _poi_set(self, cmd: dict[str, Any]) -> dict[str, Any]:
        name = cmd.get("name", "")
        if not name:
            return {"action": "poi_set", "success": False, "message": "missing POI name"}
        self._pois[name] = {
            "x": float(cmd.get("x", 0.0)),
            "y": float(cmd.get("y", 0.0)),
            "z": float(cmd.get("z", 0.0)),
        }
        save_yaml(self._poi_file, self._pois)
        return {"action": "poi_set", "success": True, "message": f"POI set: {name}"}

    def _poi_delete(self, name: str) -> dict[str, Any]:
        if name not in self._pois:
            return {
                "action": "poi_delete",
                "success": False,
                "message": f"POI not found: {name}",
            }
        del self._pois[name]
        save_yaml(self._poi_file, self._pois)
        return {"action": "poi_delete", "success": True, "message": f"POI deleted: {name}"}

    def _poi_list(self) -> dict[str, Any]:
        return {"action": "poi_list", "success": True, "pois": dict(self._pois)}

    # -- health ----------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        maps = [
            e.name for e in sorted(self._map_dir.iterdir())
            if e.is_dir() and e.name != "active"
        ] if self._map_dir.exists() else []
        info["map_count"] = len(maps)
        info["active_map"] = self._active_map
        info["poi_count"] = len(self._pois)
        info["map_dir"] = str(self._map_dir)
        return info
