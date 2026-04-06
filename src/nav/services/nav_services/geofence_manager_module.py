"""GeofenceManagerModule -- hive Module version of GeofenceManager.

Manages geofence polygons (add/remove/enable/disable/clear) and checks
robot intrusion via ray-casting point-in-polygon.

Ports:
    In:  odometry (Odometry)           -- robot pose
         geofence_command (str)        -- JSON command string
    Out: geofence_alert (dict)         -- intrusion warnings or command responses
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

from core import Module, In, Out
from core.msgs.nav import Odometry
from nav.services.nav_services.yaml_helpers import load_yaml, save_yaml


class GeofenceManagerModule(Module, layer=6):
    """Geofence management module (hive Module).

    Pure geometry + file-system logic.  Boundary publishing to
    /nav/navigation_boundary requires the ROS2 node version.
    Call ``check_intrusion()`` periodically to detect violations.
    """

    odometry: In[Odometry]
    geofence_command: In[str]
    geofence_alert: Out[dict]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        default_file = config.get(
            "geofence_file", os.path.expanduser("~/.lingtu/geofences.yaml")
        )
        self._geofence_file = Path(default_file)
        self._geofence_file.parent.mkdir(parents=True, exist_ok=True)
        self._fences: Dict[str, Dict[str, Any]] = self._load_fences()
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.geofence_command.subscribe(self._on_command)

    # -- callbacks --------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        self._robot_x = msg.x
        self._robot_y = msg.y

    def _on_command(self, raw: str) -> None:
        """Parse JSON command and dispatch to handler."""
        try:
            cmd = json.loads(raw) if isinstance(raw, str) else raw
        except (json.JSONDecodeError, TypeError):
            cmd = {}

        action = cmd.get("action", "")
        resp: Dict[str, Any] = {"action": action, "success": False}

        try:
            if action == "add":
                resp = self._add_fence(cmd)
            elif action == "remove":
                resp = self._remove_fence(cmd.get("name", ""))
            elif action == "list":
                resp = self._list_fences()
            elif action == "clear":
                resp = self._clear_fences()
            elif action == "enable":
                resp = self._set_enabled(cmd.get("name", ""), True)
            elif action == "disable":
                resp = self._set_enabled(cmd.get("name", ""), False)
            else:
                resp["message"] = f"unknown action: {action}"
        except Exception as exc:
            resp["message"] = str(exc)

        self.geofence_alert.publish(resp)

    # -- fence CRUD -------------------------------------------------------------

    def _add_fence(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        name = cmd.get("name", "")
        polygon = cmd.get("polygon", [])
        if not name:
            return {"action": "add", "success": False, "message": "missing fence name"}
        if len(polygon) < 3:
            return {"action": "add", "success": False, "message": "polygon needs >= 3 vertices"}
        self._fences[name] = {"polygon": polygon, "enabled": True}
        self._save_fences()
        return {"action": "add", "success": True, "message": f"fence added: {name}"}

    def _remove_fence(self, name: str) -> Dict[str, Any]:
        if not name:
            return {"action": "remove", "success": False, "message": "missing fence name"}
        if name not in self._fences:
            return {"action": "remove", "success": False, "message": f"fence not found: {name}"}
        del self._fences[name]
        self._save_fences()
        return {"action": "remove", "success": True, "message": f"fence removed: {name}"}

    def _list_fences(self) -> Dict[str, Any]:
        fences = [
            {
                "name": name,
                "vertex_count": len(data.get("polygon", [])),
                "enabled": data.get("enabled", True),
            }
            for name, data in self._fences.items()
        ]
        return {"action": "list", "success": True, "fences": fences}

    def _clear_fences(self) -> Dict[str, Any]:
        count = len(self._fences)
        self._fences.clear()
        self._save_fences()
        return {"action": "clear", "success": True, "message": f"cleared {count} fences"}

    def _set_enabled(self, name: str, enabled: bool) -> Dict[str, Any]:
        action_name = "enable" if enabled else "disable"
        if not name:
            return {"action": action_name, "success": False, "message": "missing fence name"}
        if name not in self._fences:
            return {"action": action_name, "success": False, "message": f"fence not found: {name}"}
        self._fences[name]["enabled"] = enabled
        self._save_fences()
        state = "enabled" if enabled else "disabled"
        return {"action": action_name, "success": True, "message": f"fence {state}: {name}"}

    # -- intrusion detection (call periodically) --------------------------------

    def check_intrusion(self) -> List[Dict[str, Any]]:
        """Check if robot is inside any enabled geofence.

        Returns list of intrusion alerts (empty if safe).
        Each alert is also published to geofence_alert.
        """
        alerts: List[Dict[str, Any]] = []
        for name, data in self._fences.items():
            if not data.get("enabled", True):
                continue
            if self._point_in_polygon(self._robot_x, self._robot_y, data["polygon"]):
                alert = {
                    "action": "warning",
                    "type": "intrusion",
                    "fence": name,
                    "robot_x": self._robot_x,
                    "robot_y": self._robot_y,
                    "message": f"robot inside restricted zone: {name}",
                }
                alerts.append(alert)
                self.geofence_alert.publish(alert)
        return alerts

    @staticmethod
    def _point_in_polygon(px: float, py: float, polygon: List) -> bool:
        """Ray-casting point-in-polygon test."""
        n = len(polygon)
        if n < 3:
            return False
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = float(polygon[i][0]), float(polygon[i][1])
            xj, yj = float(polygon[j][0]), float(polygon[j][1])
            if ((yi > py) != (yj > py)) and (
                px < (xj - xi) * (py - yi) / (yj - yi) + xi
            ):
                inside = not inside
            j = i
        return inside

    # -- persistence ------------------------------------------------------------

    def _load_fences(self) -> Dict[str, Dict[str, Any]]:
        data = load_yaml(self._geofence_file)
        return data if isinstance(data, dict) else {}

    def _save_fences(self) -> None:
        save_yaml(self._geofence_file, self._fences)
