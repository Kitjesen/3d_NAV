"""TaskSchedulerModule -- hive Module version of TaskScheduler.

Time-based task scheduler that triggers patrol routes on a
weekday/hour/minute schedule.  Pure datetime logic -- no ROS2 timers.
Call ``check_schedules()`` periodically (e.g. every 30s) to fire due tasks.

Ports:
    In:  schedule_command (str)   -- JSON command string
    Out: scheduled_task (dict)    -- fired patrol command or operation response
"""

from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from core import Module, In, Out

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore[assignment]


class TaskSchedulerModule(Module, layer=6):
    """Scheduled task management module (hive Module).

    Pure datetime + file-system logic.  Call ``check_schedules()``
    periodically to evaluate and fire due patrol tasks.
    """

    schedule_command: In[str]
    scheduled_task: Out[dict]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        default_file = config.get(
            "schedule_file", os.path.expanduser("~/.lingtu/schedules.yaml")
        )
        self._schedule_file = Path(default_file)
        self._schedule_file.parent.mkdir(parents=True, exist_ok=True)
        self._schedules: Dict[str, Dict[str, Any]] = self._load_schedules()
        # Tracks last-fired minute key per schedule to prevent double-firing
        self._last_fired: Dict[str, str] = {}

    def setup(self) -> None:
        self.schedule_command.subscribe(self._on_command)

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
            if action == "add":
                resp = self._add_schedule(cmd)
            elif action == "remove":
                resp = self._remove_schedule(cmd.get("name", ""))
            elif action == "list":
                resp = self._list_schedules()
            elif action == "enable":
                resp = self._set_enabled(cmd.get("name", ""), True)
            elif action == "disable":
                resp = self._set_enabled(cmd.get("name", ""), False)
            else:
                resp["message"] = f"unknown action: {action}"
        except Exception as exc:
            resp["message"] = str(exc)

        self.scheduled_task.publish(resp)

    # -- schedule CRUD ----------------------------------------------------------

    def _add_schedule(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        name = cmd.get("name", "")
        if not name:
            return {"action": "add", "success": False, "message": "missing schedule name"}
        patrol_route = cmd.get("patrol_route", "")
        if not patrol_route:
            return {"action": "add", "success": False, "message": "missing patrol_route"}

        self._schedules[name] = {
            "patrol_route": patrol_route,
            "hour": int(cmd.get("hour", 0)),
            "minute": int(cmd.get("minute", 0)),
            "weekdays": cmd.get("weekdays", [0, 1, 2, 3, 4, 5, 6]),
            "enabled": cmd.get("enabled", True),
        }
        self._save_schedules()
        return {"action": "add", "success": True, "message": f"schedule added: {name}"}

    def _remove_schedule(self, name: str) -> Dict[str, Any]:
        if not name:
            return {"action": "remove", "success": False, "message": "missing schedule name"}
        if name not in self._schedules:
            return {"action": "remove", "success": False, "message": f"schedule not found: {name}"}
        del self._schedules[name]
        self._last_fired.pop(name, None)
        self._save_schedules()
        return {"action": "remove", "success": True, "message": f"schedule removed: {name}"}

    def _list_schedules(self) -> Dict[str, Any]:
        items = []
        for name, data in self._schedules.items():
            items.append({
                "name": name,
                "patrol_route": data.get("patrol_route", ""),
                "hour": data.get("hour", 0),
                "minute": data.get("minute", 0),
                "weekdays": data.get("weekdays", []),
                "enabled": data.get("enabled", True),
                "last_fired": self._last_fired.get(name, ""),
            })
        return {"action": "list", "success": True, "schedules": items}

    def _set_enabled(self, name: str, enabled: bool) -> Dict[str, Any]:
        action_name = "enable" if enabled else "disable"
        if not name:
            return {"action": action_name, "success": False, "message": "missing schedule name"}
        if name not in self._schedules:
            return {"action": action_name, "success": False, "message": f"schedule not found: {name}"}
        self._schedules[name]["enabled"] = enabled
        self._save_schedules()
        state = "enabled" if enabled else "disabled"
        return {"action": action_name, "success": True, "message": f"schedule {state}: {name}"}

    # -- schedule evaluation (call periodically) --------------------------------

    def check_schedules(self, now: Optional[datetime] = None) -> List[Dict[str, Any]]:
        """Evaluate all schedules against current time and fire due tasks.

        Parameters
        ----------
        now : datetime, optional
            Override current time (useful for testing).

        Returns
        -------
        list of dict
            Fired patrol commands, each also published to ``scheduled_task``.
        """
        if now is None:
            now = datetime.now()

        current_weekday = now.weekday()
        current_hour = now.hour
        current_minute = now.minute
        minute_key = now.strftime("%Y-%m-%d %H:%M")

        fired: List[Dict[str, Any]] = []

        for name, data in self._schedules.items():
            if not data.get("enabled", True):
                continue
            if current_weekday not in data.get("weekdays", []):
                continue
            if current_hour != data.get("hour", -1):
                continue
            if current_minute != data.get("minute", -1):
                continue
            if self._last_fired.get(name) == minute_key:
                continue

            self._last_fired[name] = minute_key
            patrol_route = data.get("patrol_route", "")
            task = {
                "event": "schedule_fired",
                "schedule_name": name,
                "patrol_route": patrol_route,
                "patrol_command": {"action": "start", "name": patrol_route},
            }
            fired.append(task)
            self.scheduled_task.publish(task)

        return fired

    # -- persistence ------------------------------------------------------------

    def _load_schedules(self) -> Dict[str, Dict[str, Any]]:
        if not self._schedule_file.exists():
            return {}
        try:
            with open(self._schedule_file, "r", encoding="utf-8") as f:
                if yaml is not None:
                    data = yaml.safe_load(f)
                else:
                    data = json.load(f)
                return data if isinstance(data, dict) else {}
        except Exception:
            return {}

    def _save_schedules(self) -> None:
        try:
            self._schedule_file.parent.mkdir(parents=True, exist_ok=True)
            with open(self._schedule_file, "w", encoding="utf-8") as f:
                if yaml is not None:
                    yaml.safe_dump(
                        self._schedules, f, allow_unicode=True, default_flow_style=False
                    )
                else:
                    json.dump(self._schedules, f, ensure_ascii=False, indent=2)
        except Exception:
            pass
