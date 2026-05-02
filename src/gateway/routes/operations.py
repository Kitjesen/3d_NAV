"""Operational Gateway routes.

These endpoints talk to optional services or subprocess-backed capabilities.
Keeping them out of GatewayModule makes the runtime shell smaller without
changing the external API shape.
"""

from __future__ import annotations

import asyncio
import logging
import time
from typing import Any

from gateway.schemas import (
    BagOperationResponse,
    BagStatusResponse,
    BitrateRequest,
    ExplorationCommandResponse,
    ExplorationStatusResponse,
    GatewayErrorResponse,
    Go2RTCStatusResponse,
    SlamOperationResponse,
    SlamStatusResponse,
    TemporalMemoryResponse,
    WebRTCControlResponse,
    WebRTCStatsResponse,
)

logger = logging.getLogger(__name__)

try:
    from fastapi import Request as FastAPIRequest
except ImportError:  # FastAPI remains optional until routes are registered.
    FastAPIRequest = Any


def _parse_since(since: str) -> float:
    """Parse a human-readable duration into a Unix timestamp."""
    import re as _re

    now = time.time()
    m = _re.match(
        r"^(\d+(?:\.\d+)?)\s*(s|sec|m|min|h|hour|d|day)?",
        since.strip().lower(),
    )
    if m:
        value = float(m.group(1))
        unit = m.group(2) or "s"
        if unit in ("h", "hour"):
            return now - value * 3600
        if unit in ("m", "min"):
            return now - value * 60
        if unit in ("d", "day"):
            return now - value * 86400
        return now - value
    try:
        return now - float(since)
    except ValueError:
        return now - 3600


def register_operation_routes(app, gw) -> None:
    from fastapi import Body
    from fastapi.responses import JSONResponse, Response

    @app.get(
        "/api/v1/webrtc/stats",
        summary="WebRTC peer telemetry (bitrate, fps, encode time)",
        response_model=WebRTCStatsResponse,
        responses={500: {"model": WebRTCStatsResponse}},
    )
    async def get_webrtc_stats():
        if gw._webrtc is None:
            return {"enabled": False, "active_peers": 0}
        try:
            return await gw._webrtc.collect_stats()
        except Exception as e:
            logger.debug("webrtc stats error: %s", e)
            return JSONResponse(
                {"enabled": True, "error": str(e)},
                status_code=500,
            )

    go2rtc_upstream = gw._go2rtc_upstream

    @app.get(
        "/api/v1/webrtc/go2rtc/status",
        summary="Probe the go2rtc sidecar (image transmission fast path)",
        response_model=Go2RTCStatusResponse,
    )
    async def get_go2rtc_status():
        try:
            import httpx
        except ImportError:
            return {"available": False, "reason": "httpx_missing"}
        try:
            async with httpx.AsyncClient(timeout=0.5) as client:
                r = await client.get(f"{go2rtc_upstream}/api/streams")
                if r.status_code != 200:
                    return {"available": False, "status": r.status_code}
                data = r.json()
                streams = list(data.keys()) if isinstance(data, dict) else []
                return {"available": True, "streams": streams}
        except Exception as e:
            return {"available": False, "reason": type(e).__name__}

    @app.post(
        "/api/v1/webrtc/whep",
        summary="WHEP signalling proxy to go2rtc (image transmission path)",
        responses={
            200: {
                "content": {"application/sdp": {"schema": {"type": "string"}}}
            },
            503: {"model": GatewayErrorResponse},
        },
    )
    async def post_webrtc_whep(request: FastAPIRequest):
        try:
            import httpx
        except ImportError:
            return JSONResponse({"error": "httpx_missing"}, status_code=503)
        src = request.query_params.get("src", "cam")
        body = await request.body()
        try:
            async with httpx.AsyncClient(timeout=3.0) as client:
                r = await client.post(
                    f"{go2rtc_upstream}/api/webrtc?src={src}",
                    content=body,
                    headers={"content-type": "application/sdp"},
                )
        except Exception as e:
            logger.info("go2rtc unreachable: %s", e)
            return JSONResponse({"error": "go2rtc_unreachable"}, status_code=503)
        return Response(
            content=r.content,
            status_code=r.status_code,
            media_type=r.headers.get("content-type", "application/sdp"),
        )

    @app.post(
        "/api/v1/webrtc/offer",
        summary="WebRTC SDP offer/answer exchange for low-latency camera",
        response_model=WebRTCControlResponse,
        responses={
            400: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def post_webrtc_offer(body: dict = Body(...)):
        if gw._webrtc is None:
            return JSONResponse({"error": "webrtc_unavailable"}, status_code=503)
        try:
            return await gw._webrtc.handle_offer(body)
        except ValueError as e:
            return JSONResponse({"error": str(e)}, status_code=400)

    @app.post(
        "/api/v1/webrtc/bitrate",
        summary="Live-tune WebRTC max bitrate without reconnect",
        response_model=WebRTCControlResponse,
        responses={
            400: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def post_webrtc_bitrate(body: BitrateRequest):
        if gw._webrtc is None:
            return JSONResponse({"error": "webrtc_unavailable"}, status_code=503)
        try:
            return await gw._webrtc.set_max_bitrate(body.bps)
        except ValueError as e:
            return JSONResponse({"error": str(e)}, status_code=400)

    def _temporal_store():
        if gw._temporal_store is None:
            try:
                import os

                from memory.storage.temporal_store import TemporalStore

                mem_dir = os.environ.get(
                    "LINGTU_MEMORY_DIR",
                    os.path.join(os.path.expanduser("~"), ".nova", "semantic"),
                )
                gw._temporal_store = TemporalStore(
                    os.path.join(mem_dir, "temporal_memory.db")
                )
            except Exception as exc:
                logger.warning("GatewayModule: TemporalStore unavailable: %s", exc)
        return gw._temporal_store

    @app.get(
        "/api/v1/memory/temporal",
        summary="Query temporal entity observations",
        response_model=TemporalMemoryResponse,
        responses={503: {"model": GatewayErrorResponse}},
    )
    async def get_temporal_memory(
        label: str | None = None,
        since: str | None = None,
        near_x: float | None = None,
        near_y: float | None = None,
        radius: float | None = None,
        limit: int = 100,
    ):
        since_ts = _parse_since(since) if since else None
        store = _temporal_store()
        if store is None:
            return JSONResponse(
                status_code=503,
                content={
                    "error": "temporal_store_unavailable",
                    "detail": "TemporalMemoryModule not running or save_dir not set",
                },
            )
        loop = asyncio.get_event_loop()
        rows = await loop.run_in_executor(
            None,
            lambda: store.query(
                label=label,
                since_ts=since_ts,
                near_x=near_x,
                near_y=near_y,
                radius=radius,
                limit=max(1, min(limit, 1000)),
            ),
        )
        return {"observations": rows, "count": len(rows)}

    @app.post(
        "/api/v1/memory/temporal/semantic",
        summary="Semantic similarity search over temporal observations",
        response_model=TemporalMemoryResponse,
        responses={
            422: {"model": GatewayErrorResponse},
            503: {"model": GatewayErrorResponse},
        },
    )
    async def post_temporal_semantic(body: dict):
        import numpy as np

        raw_emb = body.get("embedding")
        if not raw_emb:
            return JSONResponse(
                status_code=422,
                content={"error": "embedding required"},
            )
        try:
            query_vec = np.asarray(raw_emb, dtype=np.float32)
        except Exception as exc:
            return JSONResponse(
                status_code=422,
                content={"error": f"invalid embedding: {exc}"},
            )

        since_ts = _parse_since(body["since"]) if body.get("since") else None
        store = _temporal_store()
        if store is None:
            return JSONResponse(
                status_code=503,
                content={"error": "temporal_store_unavailable"},
            )

        loop = asyncio.get_event_loop()
        rows = await loop.run_in_executor(
            None,
            lambda: store.query_semantic(
                query_vec,
                top_k=int(body.get("top_k", 10)),
                since_ts=since_ts,
                label=body.get("label") or None,
            ),
        )
        return {"observations": rows, "count": len(rows)}

    @app.post(
        "/api/v1/explore/start",
        summary="Start autonomous frontier exploration",
        response_model=ExplorationCommandResponse,
        responses={503: {"model": GatewayErrorResponse}},
    )
    async def explore_start():
        fe = gw._frontier_explorer
        if fe is None:
            return JSONResponse(
                status_code=503,
                content={"error": "WavefrontFrontierExplorer not running"},
            )
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, fe.begin_exploration)
        gw._exploring = True
        gw.push_event({"type": "exploring", "active": True})
        return {"status": result}

    @app.post(
        "/api/v1/explore/stop",
        summary="Stop autonomous frontier exploration",
        response_model=ExplorationCommandResponse,
        responses={503: {"model": GatewayErrorResponse}},
    )
    async def explore_stop():
        fe = gw._frontier_explorer
        if fe is None:
            return JSONResponse(
                status_code=503,
                content={"error": "WavefrontFrontierExplorer not running"},
            )
        loop = asyncio.get_event_loop()
        result = await loop.run_in_executor(None, fe.end_exploration)
        gw._exploring = False
        gw.push_event({"type": "exploring", "active": False})
        return {"status": result}

    @app.get(
        "/api/v1/explore/status",
        summary="Exploration status",
        response_model=ExplorationStatusResponse,
    )
    async def explore_status():
        fe = gw._frontier_explorer
        if fe is None:
            return {"available": False, "exploring": False}
        h = fe.health() if hasattr(fe, "health") else {}
        return {
            "available": True,
            "exploring": gw._exploring,
            "frontier_count": h.get("frontier_count", 0),
        }

    @app.get(
        "/api/v1/slam/status",
        summary="SLAM service status",
        response_model=SlamStatusResponse,
    )
    async def slam_status():
        try:
            from core.service_manager import get_service_manager

            svc = get_service_manager()
            services = svc.status("lidar", "slam", "slam_pgo", "localizer")
        except Exception:
            services = {
                "lidar": "unknown",
                "slam": "unknown",
                "slam_pgo": "unknown",
                "localizer": "unknown",
            }

        if services.get("slam_pgo") in ("running", "active"):
            mode = "fastlio2"
        elif services.get("localizer") in ("running", "active"):
            mode = "localizer"
        elif services.get("slam") in ("running", "active"):
            mode = "fastlio2"
        else:
            mode = "stopped"
        return {"mode": mode, "services": services}

    @app.post(
        "/api/v1/slam/switch",
        summary="Hot-switch SLAM profile",
        response_model=SlamOperationResponse,
        responses={
            400: {"model": SlamOperationResponse},
            500: {"model": SlamOperationResponse},
        },
    )
    async def slam_switch(body: dict):
        profile = body.get("profile", "")
        if profile not in ("fastlio2", "localizer", "stop"):
            return JSONResponse(
                {"success": False, "message": f"Unknown profile: {profile}"},
                status_code=400,
            )
        try:
            from core.service_manager import get_service_manager

            svc = get_service_manager()
            if profile == "fastlio2":
                svc.stop("localizer")
                svc.ensure("slam", "slam_pgo")
                ok = svc.wait_ready("slam", "slam_pgo", timeout=10.0)
            elif profile == "localizer":
                svc.stop("slam_pgo")
                svc.ensure("slam", "localizer")
                ok = svc.wait_ready("slam", "localizer", timeout=10.0)
            else:
                svc.stop("slam_pgo", "localizer", "slam")
                ok = True
            return {
                "success": ok,
                "profile": profile,
                "message": (
                    f"Switched to {profile}" if ok else "Services not ready after 10s"
                ),
            }
        except Exception as e:
            return JSONResponse({"success": False, "message": str(e)}, status_code=500)

    @app.post(
        "/api/v1/slam/auto_relocalize",
        summary="Global relocalize via 3D-BBS (no guess required)",
        response_model=SlamOperationResponse,
        responses={
            500: {"model": SlamOperationResponse},
            504: {"model": SlamOperationResponse},
        },
    )
    async def slam_auto_relocalize():
        import subprocess

        ros_env = (
            "source /opt/ros/humble/setup.bash && "
            "source ~/data/SLAM/navigation/install/setup.bash 2>/dev/null; "
            "unset RMW_IMPLEMENTATION; "
        )
        try:
            r = subprocess.run(
                [
                    "bash",
                    "-c",
                    ros_env
                    + "ros2 service call /nav/global_relocalize "
                    "std_srvs/srv/Trigger '{}'",
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )
            ok = "success=True" in r.stdout
            msg = r.stdout[-300:] if r.stdout else (r.stderr[-300:] or "no output")
            return {"success": ok, "message": msg}
        except subprocess.TimeoutExpired:
            return JSONResponse(
                {"success": False, "message": "call timeout > 10s"},
                status_code=504,
            )
        except Exception as e:
            return JSONResponse({"success": False, "message": str(e)}, status_code=500)

    @app.post(
        "/api/v1/slam/relocalize",
        summary="Relocalize against a saved map",
        response_model=SlamOperationResponse,
        responses={
            400: {"model": SlamOperationResponse},
            404: {"model": SlamOperationResponse},
            500: {"model": SlamOperationResponse},
        },
    )
    async def slam_relocalize(body: dict):
        import os
        import subprocess

        map_name = body.get("map_name", "")
        x = float(body.get("x", 0.0))
        y = float(body.get("y", 0.0))
        yaw = float(body.get("yaw", 0.0))
        if not map_name:
            return JSONResponse(
                {"success": False, "message": "map_name required"},
                status_code=400,
            )
        map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps"))
        pcd_path = os.path.join(map_dir, map_name, "map.pcd")
        if not os.path.isfile(pcd_path):
            alt = os.path.expanduser(f"~/data/inovxio/data/maps/{map_name}/map.pcd")
            if os.path.isfile(alt):
                pcd_path = alt
        if not os.path.isfile(pcd_path):
            return JSONResponse(
                {"success": False, "message": f"Map not found: {pcd_path}"},
                status_code=404,
            )
        ros_env = (
            "source /opt/ros/humble/setup.bash && "
            "source ~/data/SLAM/navigation/install/setup.bash 2>/dev/null; "
            "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
        )
        try:
            r = subprocess.run(
                [
                    "bash",
                    "-c",
                    ros_env
                    + "ros2 service call /nav/relocalize interface/srv/Relocalize "
                    f"\"{{pcd_path: '{pcd_path}', x: {x}, y: {y}, z: 0.0, "
                    f"yaw: {yaw}, pitch: 0.0, roll: 0.0}}\"",
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )
            ok = "success=True" in r.stdout
            quality = None
            for line in r.stdout.splitlines():
                ll = line.lower().strip()
                if any(k in ll for k in ("quality:", "score:", "fitness:")):
                    try:
                        quality = float(ll.split(":", 1)[-1].strip())
                        break
                    except ValueError:
                        pass
            if quality is None and ok:
                quality = float(getattr(gw, "_icp_quality", 0.0))
            if ok:
                gw._persist_last_nav_pose(map_name, x, y, yaw, quality)
            msg = r.stdout[-300:] if not ok else f"Relocalized to {map_name}"
            return {"success": ok, "message": msg, "quality": quality}
        except Exception as e:
            return JSONResponse({"success": False, "message": str(e)}, status_code=500)

    @app.post(
        "/api/v1/bag/start",
        summary="Start rosbag recording",
        response_model=BagOperationResponse,
        responses={
            409: {"model": BagOperationResponse},
            500: {"model": BagOperationResponse},
        },
    )
    async def bag_start(body: dict | None = None):
        import os
        import pathlib
        import subprocess

        body = body or {}
        duration = int(body.get("duration", 600))
        prefix = str(body.get("prefix", "web"))[:40]
        prefix = "".join(c for c in prefix if c.isalnum() or c in "-_") or "web"

        with gw._bag_lock:
            if gw._bag_proc is not None and gw._bag_proc.poll() is None:
                return JSONResponse(
                    status_code=409,
                    content={
                        "error": "recording_in_progress",
                        "path": gw._bag_path,
                        "pid": gw._bag_proc.pid,
                    },
                )

            repo_root = pathlib.Path(__file__).resolve().parents[3]
            script = repo_root / "scripts" / "record_bag.sh"
            if not script.exists():
                return JSONResponse(
                    status_code=500,
                    content={"error": "script_not_found", "path": str(script)},
                )

            stamp = time.strftime("%Y%m%d_%H%M%S")
            bag_dir = os.path.expanduser(f"~/data/bags/{prefix}_{stamp}")
            try:
                proc = subprocess.Popen(
                    ["bash", str(script), str(duration), prefix],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
            except FileNotFoundError as e:
                return JSONResponse(
                    status_code=500,
                    content={"error": "bash_not_found", "detail": str(e)},
                )

            gw._bag_proc = proc
            gw._bag_path = bag_dir
            gw._bag_started_ts = time.time()
            return {
                "status": "started",
                "path": bag_dir,
                "pid": proc.pid,
                "duration": duration,
                "prefix": prefix,
            }

    @app.post(
        "/api/v1/bag/stop",
        summary="Stop rosbag recording",
        response_model=BagOperationResponse,
        responses={404: {"model": BagOperationResponse}},
    )
    async def bag_stop():
        import os
        import signal

        with gw._bag_lock:
            proc = gw._bag_proc
            if proc is None or proc.poll() is not None:
                return JSONResponse(
                    status_code=404,
                    content={"error": "not_recording"},
                )
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError, AttributeError):
                try:
                    proc.terminate()
                except Exception:
                    pass
            return {"status": "stopping", "path": gw._bag_path, "pid": proc.pid}

    @app.get(
        "/api/v1/bag/status",
        summary="rosbag recording status",
        response_model=BagStatusResponse,
    )
    async def bag_status():
        import os
        import shutil

        with gw._bag_lock:
            proc = gw._bag_proc
            path = gw._bag_path
            started_ts = gw._bag_started_ts

        recording = proc is not None and proc.poll() is None
        size_bytes = 0
        if path and os.path.isdir(path):
            try:
                for root, _dirs, files in os.walk(path):
                    for filename in files:
                        fp = os.path.join(root, filename)
                        try:
                            size_bytes += os.path.getsize(fp)
                        except OSError:
                            pass
            except OSError:
                pass

        bag_disk = os.path.expanduser("~/data")
        if not os.path.isdir(bag_disk):
            bag_disk = os.path.expanduser("~")
        try:
            du = shutil.disk_usage(bag_disk)
            disk_free, disk_total = du.free, du.total
        except OSError:
            disk_free, disk_total = 0, 0
        return {
            "recording": recording,
            "path": path,
            "duration_s": (time.time() - started_ts) if started_ts else 0.0,
            "size_bytes": size_bytes,
            "pid": proc.pid if proc else None,
            "exit_code": (
                proc.returncode if proc and proc.poll() is not None else None
            ),
            "disk_free": disk_free,
            "disk_total": disk_total,
        }
