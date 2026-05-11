"""Diagnostic export routes for GatewayModule."""

from __future__ import annotations

import io
import json
import pathlib
import subprocess
import tarfile
import tempfile
import time
from datetime import datetime
from typing import Any


def _json_ready(value: Any) -> Any:
    try:
        json.dumps(value, ensure_ascii=False, default=str)
        return value
    except Exception:
        return str(value)


def _location_entries(gw: Any) -> list[Any]:
    tlm = getattr(gw, "_tagged_loc_module", None)
    if tlm is None:
        return []
    try:
        return list(tlm.store.list_all())
    except Exception:
        try:
            return list(tlm.store._store.values())
        except Exception:
            return []


def _snapshot_or_error(name: str, builder) -> dict[str, Any]:
    try:
        value = builder()
        return {"ok": True, "data": _json_ready(value)}
    except Exception as exc:
        return {"ok": False, "error": str(exc), "snapshot": name}


def _maps_snapshot(gw: Any) -> dict[str, Any]:
    from gateway.services.map_paths import active_map_name, nav_map_root_str

    root = pathlib.Path(nav_map_root_str())
    maps: list[dict[str, Any]] = []
    if root.is_dir():
        for entry in sorted(root.iterdir()):
            if not entry.is_dir() or entry.name == "active":
                continue
            pcd = entry / "map.pcd"
            patches = entry / "patches"
            maps.append(
                {
                    "name": entry.name,
                    "has_pcd": pcd.is_file(),
                    "has_tomogram": (entry / "tomogram.pickle").is_file(),
                    "has_occupancy": (entry / "occupancy.npz").is_file(),
                    "patch_count": (
                        len(list(patches.iterdir())) if patches.is_dir() else 0
                    ),
                    "size_mb": (
                        round(pcd.stat().st_size / 1024 / 1024, 1)
                        if pcd.is_file()
                        else None
                    ),
                }
            )

    manager_snapshot: dict[str, Any] | None = None
    manager = getattr(gw, "_map_mgr", None)
    if manager is not None and callable(getattr(manager, "_map_list", None)):
        manager_snapshot = manager._map_list()

    return {
        "schema_version": 1,
        "map_dir": str(root),
        "active": active_map_name() or "",
        "maps": maps,
        "count": len(maps),
        "has_manager": manager is not None,
        "manager": manager_snapshot,
        "ts": time.time(),
    }


def _command_snapshot(gw: Any) -> dict[str, Any]:
    if callable(getattr(gw, "_command_stats_snapshot", None)):
        snapshot = gw._command_stats_snapshot()
        if isinstance(snapshot, dict):
            return snapshot
    return {
        "idempotency_supported": False,
        "stored_requests": 0,
        "accepted_commands": 0,
        "replayed_commands": 0,
        "rate_policy_hz": {},
        "rate_policy_enforcement": "unknown",
    }


def _build_app_web_snapshots(gw: Any) -> dict[str, dict[str, Any]]:
    from gateway.services.app_bootstrap import (
        build_app_bootstrap,
        build_app_capabilities,
        build_app_traffic,
    )
    from gateway.services.media_status import build_media_status
    from gateway.services.readiness import build_readiness_snapshot
    from gateway.services.runtime_status import (
        build_localization_status,
        build_navigation_status,
    )
    from gateway.services.state_snapshot import build_state_snapshot
    from gateway.services.telemetry_normalizers import (
        build_locations_response,
        build_path_response,
        build_scene_graph_response,
    )

    def _scene_graph():
        with gw._state_lock:
            sg = gw._sg_json
        return build_scene_graph_response(sg)

    def _path():
        with gw._state_lock:
            path = gw._last_path
            robot = gw._odom
        return build_path_response(path, robot)

    readiness = _snapshot_or_error(
        "readiness",
        lambda: build_readiness_snapshot(gw, include_details=True)[0],
    )
    return {
        "bootstrap": _snapshot_or_error(
            "bootstrap",
            lambda: build_app_bootstrap(gw),
        ),
        "capabilities": _snapshot_or_error(
            "capabilities",
            lambda: build_app_capabilities(gw),
        ),
        "traffic": _snapshot_or_error(
            "traffic",
            lambda: build_app_traffic(gw),
        ),
        "readiness": readiness,
        "state": _snapshot_or_error("state", lambda: build_state_snapshot(gw)),
        "localization": _snapshot_or_error(
            "localization",
            lambda: build_localization_status(gw),
        ),
        "navigation": _snapshot_or_error(
            "navigation",
            lambda: build_navigation_status(gw),
        ),
        "path": _snapshot_or_error("path", _path),
        "scene_graph": _snapshot_or_error("scene_graph", _scene_graph),
        "locations": _snapshot_or_error(
            "locations",
            lambda: build_locations_response(_location_entries(gw)),
        ),
        "media": _snapshot_or_error(
            "media",
            lambda: build_media_status(gw),
        ),
        "session": _snapshot_or_error(
            "session",
            lambda: gw._session_snapshot(),
        ),
        "maps": _snapshot_or_error("maps", lambda: _maps_snapshot(gw)),
        "commands": _snapshot_or_error("commands", lambda: _command_snapshot(gw)),
    }


def register_diagnostic_routes(app, gw) -> None:
    from fastapi.responses import FileResponse
    from starlette.background import BackgroundTask

    @app.get(
        "/api/v1/diagnostic_pack",
        summary="Export diagnostic tarball",
        responses={
            200: {
                "content": {
                    "application/gzip": {
                        "schema": {"type": "string", "format": "binary"}
                    }
                }
            }
        },
    )
    async def diagnostic_pack():
        repo_root = pathlib.Path(__file__).resolve().parents[3]
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        tmp_path = pathlib.Path(tempfile.gettempdir()) / f"diag_{stamp}.tar.gz"

        def _add_text(tar: tarfile.TarFile, arcname: str, text: str) -> None:
            data = text.encode("utf-8")
            info = tarfile.TarInfo(arcname)
            info.size = len(data)
            info.mtime = int(time.time())
            tar.addfile(info, io.BytesIO(data))

        with tarfile.open(tmp_path, "w:gz") as tar:
            modules_info: dict[str, Any] = {}
            for name, module in gw._all_modules.items():
                try:
                    if hasattr(module, "health"):
                        modules_info[name] = module.health()
                    elif hasattr(module, "port_summary"):
                        modules_info[name] = module.port_summary()
                except Exception as exc:
                    modules_info[name] = {"error": str(exc)}

            _add_text(
                tar,
                "diag/modules.json",
                json.dumps(
                    modules_info,
                    indent=2,
                    ensure_ascii=False,
                    default=str,
                ),
            )
            _add_text(
                tar,
                "diag/health.json",
                json.dumps(
                    {
                        "modules_ok": sum(
                            1 for v in modules_info.values() if "error" not in v
                        ),
                        "modules_fail": sum(
                            1 for v in modules_info.values() if "error" in v
                        ),
                        "count": len(modules_info),
                    },
                    indent=2,
                    ensure_ascii=False,
                    default=str,
                ),
            )

            try:
                git_out = subprocess.check_output(
                    ["git", "rev-parse", "HEAD"],
                    cwd=repo_root,
                    stderr=subprocess.DEVNULL,
                    timeout=3,
                ).decode().strip()
                short = git_out[:12]
            except Exception:
                git_out, short = "unknown", "unknown"
            _add_text(tar, "diag/git_head.txt", f"{git_out}\nshort: {short}\n")

            app_web_snapshots = _build_app_web_snapshots(gw)
            _add_text(
                tar,
                "diag/app_web_snapshots.json",
                json.dumps(
                    app_web_snapshots,
                    indent=2,
                    ensure_ascii=False,
                    default=str,
                ),
            )
            for name, snapshot in app_web_snapshots.items():
                _add_text(
                    tar,
                    f"diag/app_web/{name}.json",
                    json.dumps(
                        snapshot,
                        indent=2,
                        ensure_ascii=False,
                        default=str,
                    ),
                )

            cfg_path = repo_root / "config" / "robot_config.yaml"
            if cfg_path.exists():
                tar.add(str(cfg_path), arcname="diag/robot_config.yaml")

            logs_root = repo_root / "logs"
            if logs_root.exists():
                latest_log_dirs = sorted(
                    [p for p in logs_root.iterdir() if p.is_dir()],
                    key=lambda p: p.stat().st_mtime,
                    reverse=True,
                )[:1]
                for log_dir in latest_log_dirs:
                    for logfile in log_dir.glob("*.log"):
                        try:
                            tar.add(
                                str(logfile),
                                arcname=f"diag/logs/{log_dir.name}/{logfile.name}",
                            )
                        except Exception:
                            pass

        return FileResponse(
            path=str(tmp_path),
            filename=f"lingtu_diag_{stamp}.tar.gz",
            media_type="application/gzip",
            background=BackgroundTask(tmp_path.unlink, missing_ok=True),
        )
