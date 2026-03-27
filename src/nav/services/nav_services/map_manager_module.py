"""MapManagerModule -- hive Module version of MapManager.

Manages map listing/saving/deleting/renaming/activation and POI CRUD.

Maps are stored as directories under map_dir:

    ~/data/nova/maps/
    ├── active -> building_2f/     (symlink pointing to the active map dir)
    ├── building_2f/
    │   ├── map.pcd                (SLAM point cloud)
    │   └── tomogram.pickle        (global planning map, auto-generated)
    └── warehouse/
        ├── map.pcd
        └── tomogram.pickle

Ports:
    In:  map_command (str)   -- JSON command string
    Out: map_response (dict) -- operation result dict
"""

from __future__ import annotations

import json
import os
import shutil
from pathlib import Path
from typing import Any, Dict, List, Optional

from core import Module, In, Out

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore[assignment]


class MapManagerModule(Module, layer=6):
    """Map and POI management module (hive Module).

    Maps are stored as subdirectories.  Each map directory contains:
        map.pcd         -- SLAM point cloud saved via ROS2 SaveMaps service
        tomogram.pickle -- global planning index auto-built from map.pcd

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
            os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps")),
        )
        self._data_dir = Path(default_data_dir)
        self._map_dir = Path(default_map_dir)
        self._data_dir.mkdir(parents=True, exist_ok=True)
        self._map_dir.mkdir(parents=True, exist_ok=True)

        self._poi_file = self._data_dir / "pois.yaml"
        self._active_map_file = self._data_dir / "active_map.yaml"

        self._pois: Dict[str, Dict[str, float]] = self._load_yaml(
            self._poi_file, default={}
        )
        self._active_map: str = self._load_yaml(
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
        resp: Dict[str, Any] = {"action": action, "success": False}

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

    def _map_list(self) -> Dict[str, Any]:
        """List map directories (exclude the 'active' symlink entry)."""
        maps: List[Dict[str, Any]] = []
        for entry in sorted(self._map_dir.iterdir()):
            if not entry.is_dir() or entry.name == "active":
                continue
            maps.append(
                {
                    "name": entry.name,
                    "has_pcd": (entry / "map.pcd").exists(),
                    "has_tomogram": (entry / "tomogram.pickle").exists(),
                }
            )
        return {
            "action": "list",
            "success": True,
            "maps": maps,
            "active": self._active_map,
        }

    def _map_save(self, name: str) -> Dict[str, Any]:
        """Save SLAM map via ROS2 SaveMaps service, then build tomogram."""
        if not name:
            return {"action": "save", "success": False, "message": "missing map name"}

        map_dir = self._map_dir / name
        map_dir.mkdir(parents=True, exist_ok=True)
        pcd_path = map_dir / "map.pcd"

        # Call the ROS2 SLAM save_maps service
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

        # Auto-build tomogram from the saved PCD
        tomo_result = self._build_tomogram(name)

        return {
            "action": "save",
            "success": True,
            "pcd": str(pcd_path),
            "tomogram": tomo_result.get("tomogram"),
            "tomogram_ok": tomo_result.get("success", False),
        }

    def _build_tomogram(self, name: str) -> Dict[str, Any]:
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

    def _map_delete(self, name: str) -> Dict[str, Any]:
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

    def _map_rename(self, name: str, new_name: str) -> Dict[str, Any]:
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

    def _map_set_active(self, name: str) -> Dict[str, Any]:
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
        self._save_yaml(self._active_map_file, {"active": self._active_map})

    # -- public helpers for other modules ---------------------------------------

    def get_active_tomogram(self) -> Optional[str]:
        """Return the tomogram.pickle path for the active map, or None.

        NavigationModule uses this to locate the PCT planner index at startup.
        """
        active = self._map_dir / "active" / "tomogram.pickle"
        if active.exists():
            return str(active)
        return None

    # -- POI operations ---------------------------------------------------------

    def _poi_set(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        name = cmd.get("name", "")
        if not name:
            return {"action": "poi_set", "success": False, "message": "missing POI name"}
        self._pois[name] = {
            "x": float(cmd.get("x", 0.0)),
            "y": float(cmd.get("y", 0.0)),
            "z": float(cmd.get("z", 0.0)),
        }
        self._save_yaml(self._poi_file, self._pois)
        return {"action": "poi_set", "success": True, "message": f"POI set: {name}"}

    def _poi_delete(self, name: str) -> Dict[str, Any]:
        if name not in self._pois:
            return {
                "action": "poi_delete",
                "success": False,
                "message": f"POI not found: {name}",
            }
        del self._pois[name]
        self._save_yaml(self._poi_file, self._pois)
        return {"action": "poi_delete", "success": True, "message": f"POI deleted: {name}"}

    def _poi_list(self) -> Dict[str, Any]:
        return {"action": "poi_list", "success": True, "pois": dict(self._pois)}

    # -- YAML helpers -----------------------------------------------------------

    @staticmethod
    def _load_yaml(path: Path, default: Any = None) -> Any:
        if default is None:
            default = {}
        if not path.exists():
            return default
        try:
            with open(path, "r", encoding="utf-8") as f:
                if yaml is not None:
                    data = yaml.safe_load(f)
                else:
                    data = json.load(f)
                return data if data is not None else default
        except Exception:
            return default

    @staticmethod
    def _save_yaml(path: Path, data: Any) -> None:
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                if yaml is not None:
                    yaml.safe_dump(data, f, allow_unicode=True, default_flow_style=False)
                else:
                    json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception:
            pass
