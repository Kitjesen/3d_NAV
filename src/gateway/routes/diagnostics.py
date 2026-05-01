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
