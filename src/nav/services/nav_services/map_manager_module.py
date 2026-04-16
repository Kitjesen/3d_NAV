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
        """Build ROS2-compatible 2D occupancy grid from SLAM output.

        Outputs in <map_dir>/<name>/:
            occupancy.npz  — numpy (grid int8: -1/0/100, resolution, origin)
            map.pgm        — ROS2 map_server PGM (254=free, 0=occ, 205=unknown)
            map.yaml       — ROS2 map_server metadata

        Prefers log-odds Bayesian raycasting when PGO wrote poses.txt + patches/;
        falls back to height-filtered XY projection (binary, no unknown) if PGO
        didn't save patches — matches legacy behaviour so Fast-LIO2-only setups
        still produce a usable grid.
        """
        if not name:
            return {"action": "build_occupancy_snapshot", "success": False,
                    "message": "missing map name"}

        map_dir = self._map_dir / name
        pcd_path = map_dir / "map.pcd"
        occ_path = map_dir / "occupancy.npz"
        pgm_path = map_dir / "map.pgm"
        yaml_path = map_dir / "map.yaml"
        poses_path = map_dir / "poses.txt"
        patches_dir = map_dir / "patches"

        if not pcd_path.exists():
            return {"action": "build_occupancy_snapshot", "success": False,
                    "message": f"no PCD file at {pcd_path}"}

        try:
            has_pgo = (
                poses_path.exists()
                and patches_dir.is_dir()
                and any(patches_dir.iterdir())
            )
            if has_pgo:
                grid, resolution, origin = self._build_occupancy_raycasting(
                    poses_path, patches_dir,
                )
                mode = "raycasting"
            else:
                grid, resolution, origin = self._build_occupancy_projection(pcd_path)
                mode = "projection"

            np.savez_compressed(
                str(occ_path),
                grid=grid,
                resolution=np.float64(resolution),
                origin=origin.astype(np.float64),
            )
            self._save_occupancy_pgm_yaml(grid, resolution, origin, pgm_path, yaml_path)

            n_unk = int((grid == -1).sum())
            n_fre = int((grid ==  0).sum())
            n_occ = int((grid == 100).sum())
            logger.info(
                "Occupancy '%s' (%s) shape=%s res=%.3f  unknown=%d free=%d occupied=%d",
                name, mode, grid.shape, resolution, n_unk, n_fre, n_occ,
            )
            return {
                "action":     "build_occupancy_snapshot",
                "success":    True,
                "occupancy":  str(occ_path),
                "pgm":        str(pgm_path),
                "yaml":       str(yaml_path),
                "mode":       mode,
                "grid_shape": list(grid.shape),
                "resolution": float(resolution),
                "origin":     origin.tolist(),
                "counts":     {"unknown": n_unk, "free": n_fre, "occupied": n_occ},
            }

        except Exception as exc:
            logger.warning("build_occupancy_snapshot failed for '%s': %s", name, exc)
            return {"action": "build_occupancy_snapshot", "success": False,
                    "message": str(exc)}

    # ── Occupancy algorithms ───────────────────────────────────────────────

    def _build_occupancy_raycasting(
        self, poses_path: Path, patches_dir: Path,
    ) -> tuple[np.ndarray, float, np.ndarray]:
        """Log-odds Bayesian occupancy via raycasting from keyframe poses."""
        resolution:  float = 0.10
        z_min_rel:   float = 0.10
        z_max_rel:   float = 2.00
        max_range:   float = 30.0
        log_occ:     float = 0.85
        log_free:    float = -0.40
        log_min:     float = -2.0
        log_max:     float = 3.5
        thresh_occ:  float = 0.65
        thresh_free: float = 0.196

        poses = self._parse_poses_txt(poses_path)
        if not poses:
            raise RuntimeError(f"no valid poses in {poses_path}")

        frame_data: list[tuple[np.ndarray, np.ndarray]] = []
        ground_z_list: list[float] = []
        xy_min = np.array([+np.inf, +np.inf], dtype=np.float32)
        xy_max = np.array([-np.inf, -np.inf], dtype=np.float32)

        for p in poses:
            patch_path = patches_dir / p["patch"]
            if not patch_path.is_file():
                continue
            body_pts = self._load_pcd_points(str(patch_path))
            if body_pts is None or body_pts.shape[0] == 0:
                continue
            R = self._quat_to_rot(p["q"])
            t = p["t"]
            world_pts = body_pts @ R.T + t
            dx = world_pts[:, 0] - t[0]
            dy = world_pts[:, 1] - t[1]
            dist = np.sqrt(dx * dx + dy * dy)
            world_pts = world_pts[dist < max_range]
            if world_pts.shape[0] == 0:
                continue
            frame_data.append((t[:2].copy(), world_pts))
            xy_min = np.minimum(xy_min, world_pts[:, :2].min(axis=0))
            xy_max = np.maximum(xy_max, world_pts[:, :2].max(axis=0))
            ground_z_list.append(float(np.percentile(world_pts[:, 2], 5)))

        if not frame_data:
            raise RuntimeError("no usable patches loaded")

        ground_z = float(np.median(ground_z_list))
        z_lo, z_hi = ground_z + z_min_rel, ground_z + z_max_rel

        border = 1.0
        origin = (xy_min - border).astype(np.float64)
        grid_w = int(np.ceil((xy_max[0] + border - origin[0]) / resolution)) + 1
        grid_h = int(np.ceil((xy_max[1] + border - origin[1]) / resolution)) + 1
        if grid_w <= 0 or grid_h <= 0 or grid_w * grid_h > 25_000_000:
            raise RuntimeError(f"grid size out of range: {grid_w}×{grid_h}")

        log_odds = np.zeros((grid_h, grid_w), dtype=np.float32)

        for origin_xy, world_pts in frame_data:
            ox = int(np.floor((origin_xy[0] - origin[0]) / resolution))
            oy = int(np.floor((origin_xy[1] - origin[1]) / resolution))

            end_col = np.floor((world_pts[:, 0] - origin[0]) / resolution).astype(np.int32)
            end_row = np.floor((world_pts[:, 1] - origin[1]) / resolution).astype(np.int32)

            self._raycast_free(log_odds, ox, oy, end_col, end_row,
                               grid_w, grid_h, log_free)

            obs_mask = (world_pts[:, 2] >= z_lo) & (world_pts[:, 2] <= z_hi)
            hc = end_col[obs_mask]
            hr = end_row[obs_mask]
            valid = (hc >= 0) & (hc < grid_w) & (hr >= 0) & (hr < grid_h)
            if valid.any():
                np.add.at(log_odds, (hr[valid], hc[valid]), log_occ)

            np.clip(log_odds, log_min, log_max, out=log_odds)

        grid = np.full((grid_h, grid_w), -1, dtype=np.int8)
        prob = 1.0 / (1.0 + np.exp(-log_odds))
        grid[prob > thresh_occ] = 100
        grid[(prob < thresh_free) & (log_odds < -0.01)] = 0

        return grid, resolution, origin

    def _build_occupancy_projection(
        self, pcd_path: Path,
    ) -> tuple[np.ndarray, float, np.ndarray]:
        """Fallback: height-filter + XY projection (binary, no unknown state)."""
        pts = self._load_pcd_points(str(pcd_path))
        if pts is None or pts.shape[0] == 0:
            raise RuntimeError("PCD file empty or unparseable")

        resolution: float = 0.20
        z_min_rel:  float = 0.10
        z_max_rel:  float = 2.00

        ground_z = float(np.percentile(pts[:, 2], 5))
        z_lo, z_hi = ground_z + z_min_rel, ground_z + z_max_rel
        mask = (pts[:, 2] >= z_lo) & (pts[:, 2] <= z_hi)
        obs_pts = pts[mask, :2]

        xy_min = pts[:, :2].min(axis=0)
        xy_max = pts[:, :2].max(axis=0)
        border = resolution
        origin = (xy_min - border).astype(np.float64)
        grid_w = int(np.ceil((xy_max[0] + border - origin[0]) / resolution)) + 1
        grid_h = int(np.ceil((xy_max[1] + border - origin[1]) / resolution)) + 1
        grid = np.zeros((grid_h, grid_w), dtype=np.int8)

        if obs_pts.shape[0] > 0:
            col = np.floor((obs_pts[:, 0] - origin[0]) / resolution).astype(np.int32)
            row = np.floor((obs_pts[:, 1] - origin[1]) / resolution).astype(np.int32)
            v = (col >= 0) & (col < grid_w) & (row >= 0) & (row < grid_h)
            grid[row[v], col[v]] = 100

        return grid, resolution, origin

    # ── Output helpers ─────────────────────────────────────────────────────

    def _save_occupancy_pgm_yaml(
        self, grid: np.ndarray, resolution: float, origin: np.ndarray,
        pgm_path: Path, yaml_path: Path,
    ) -> None:
        """ROS2 map_server compatible PGM + YAML."""
        pgm = np.full_like(grid, 205, dtype=np.uint8)
        pgm[grid == 0] = 254
        pgm[grid == 100] = 0
        pgm = np.flipud(pgm)

        h, w = pgm.shape
        with open(pgm_path, "wb") as f:
            f.write(f"P5\n{w} {h}\n255\n".encode())
            f.write(pgm.tobytes())

        yaml_body = {
            "image":           pgm_path.name,
            "resolution":      float(resolution),
            "origin":          [float(origin[0]), float(origin[1]), 0.0],
            "negate":          0,
            "occupied_thresh": 0.65,
            "free_thresh":     0.196,
            "mode":            "trinary",
        }
        try:
            import yaml as _yaml
            with open(yaml_path, "w") as f:
                _yaml.safe_dump(yaml_body, f, default_flow_style=False, sort_keys=False)
        except ImportError:
            yaml_path.write_text(
                f"image: {yaml_body['image']}\n"
                f"resolution: {yaml_body['resolution']}\n"
                f"origin: [{yaml_body['origin'][0]}, {yaml_body['origin'][1]}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
                f"mode: trinary\n"
            )

    @staticmethod
    def _raycast_free(
        log_odds: np.ndarray, ox: int, oy: int,
        end_col: np.ndarray, end_row: np.ndarray,
        grid_w: int, grid_h: int, log_free: float,
    ) -> None:
        """Vectorised free-space update along rays via DDA rasterisation."""
        if end_col.size == 0:
            return
        dx = end_col - ox
        dy = end_row - oy
        steps = np.maximum(np.abs(dx), np.abs(dy))
        steps = np.minimum(steps, 500).astype(np.int32)
        max_s = int(steps.max()) if steps.size else 0
        if max_s < 1:
            return

        chunk = max(1, 200_000 // max_s)
        n = end_col.size
        ks = np.arange(max_s, dtype=np.float32)

        for s in range(0, n, chunk):
            e = min(n, s + chunk)
            dx_b = dx[s:e].astype(np.float32)
            dy_b = dy[s:e].astype(np.float32)
            steps_b = steps[s:e].astype(np.float32)
            frac = ks[None, :] / np.maximum(steps_b[:, None], 1.0)
            cols = ox + frac * dx_b[:, None]
            rows = oy + frac * dy_b[:, None]
            valid_len = ks[None, :] < steps_b[:, None]
            cols = np.floor(cols).astype(np.int32)
            rows = np.floor(rows).astype(np.int32)
            in_bounds = (cols >= 0) & (cols < grid_w) & (rows >= 0) & (rows < grid_h)
            mask = in_bounds & valid_len
            if not mask.any():
                continue
            cols = cols[mask]
            rows = rows[mask]
            np.add.at(log_odds, (rows, cols), log_free)

    @staticmethod
    def _parse_poses_txt(path: Path) -> list[dict]:
        """Parse PGO poses.txt: each line 'patch.pcd tx ty tz qw qx qy qz'."""
        poses: list[dict] = []
        with open(path) as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) != 8:
                    continue
                try:
                    poses.append({
                        "patch": parts[0],
                        "t": np.array(
                            [float(parts[1]), float(parts[2]), float(parts[3])],
                            dtype=np.float32),
                        "q": np.array(
                            [float(parts[4]), float(parts[5]),
                             float(parts[6]), float(parts[7])],
                            dtype=np.float32),
                    })
                except ValueError:
                    continue
        return poses

    @staticmethod
    def _quat_to_rot(q: np.ndarray) -> np.ndarray:
        """Quaternion (w, x, y, z) → 3×3 rotation matrix."""
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        n = np.sqrt(w * w + x * x + y * y + z * z)
        if n < 1e-9:
            return np.eye(3, dtype=np.float32)
        w, x, y, z = w / n, x / n, y / n, z / n
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
            [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
        ], dtype=np.float32)

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
