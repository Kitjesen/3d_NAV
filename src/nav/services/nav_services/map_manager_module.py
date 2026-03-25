"""MapManagerModule -- hive Module version of MapManager.

Manages map listing/saving/deleting/renaming/activation and POI CRUD.
Extracted pure file-system logic; ROS2-specific save_map service call
is not available in Module mode (requires S100P ROS2 environment).

Ports:
    In:  map_command (str)  -- JSON command string
    Out: map_response (dict) -- operation result dict
"""

from __future__ import annotations

import json
import os
import shutil
from pathlib import Path
from typing import Any, Dict, List, Optional

from src.core import Module, In, Out

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore[assignment]


class MapManagerModule(Module, layer=6):
    """Map and POI management module (hive Module).

    Pure file-system operations -- no ROS2 dependency.
    The ``save`` command with SLAM service call is unavailable in Module
    mode; it requires the full ROS2 stack on S100P.
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
            os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/maps")),
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
        maps = sorted(f.stem for f in self._map_dir.glob("*.pcd"))
        return {
            "action": "list",
            "success": True,
            "maps": maps,
            "active": self._active_map,
        }

    def _map_save(self, name: str) -> Dict[str, Any]:
        if not name:
            return {"action": "save", "success": False, "message": "missing map name"}
        # NOTE: SLAM service call (SaveMaps) not available in Module mode.
        return {
            "action": "save",
            "success": False,
            "message": "save requires ROS2 SaveMaps service (not available in Module mode)",
        }

    def _map_delete(self, name: str) -> Dict[str, Any]:
        if not name:
            return {"action": "delete", "success": False, "message": "missing map name"}
        path = self._map_dir / f"{name}.pcd"
        if not path.exists():
            return {"action": "delete", "success": False, "message": f"map not found: {name}"}
        try:
            path.unlink()
            if self._active_map == name:
                self._active_map = ""
                self._save_active_map()
            return {"action": "delete", "success": True, "message": f"deleted: {name}"}
        except OSError as exc:
            return {"action": "delete", "success": False, "message": str(exc)}

    def _map_rename(self, name: str, new_name: str) -> Dict[str, Any]:
        if not name or not new_name:
            return {"action": "rename", "success": False, "message": "missing name(s)"}
        src = self._map_dir / f"{name}.pcd"
        dst = self._map_dir / f"{new_name}.pcd"
        if not src.exists():
            return {"action": "rename", "success": False, "message": f"map not found: {name}"}
        if dst.exists():
            return {"action": "rename", "success": False, "message": f"target exists: {new_name}"}
        try:
            shutil.move(str(src), str(dst))
            if self._active_map == name:
                self._active_map = new_name
                self._save_active_map()
            return {"action": "rename", "success": True, "message": f"{name} -> {new_name}"}
        except OSError as exc:
            return {"action": "rename", "success": False, "message": str(exc)}

    def _map_set_active(self, name: str) -> Dict[str, Any]:
        if not name:
            return {"action": "set_active", "success": False, "message": "missing map name"}
        path = self._map_dir / f"{name}.pcd"
        if not path.exists():
            return {"action": "set_active", "success": False, "message": f"map not found: {name}"}
        self._active_map = name
        self._save_active_map()
        return {"action": "set_active", "success": True, "active": name}

    def _save_active_map(self) -> None:
        self._save_yaml(self._active_map_file, {"active": self._active_map})

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
            return {"action": "poi_delete", "success": False, "message": f"POI not found: {name}"}
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
