#!/usr/bin/env python3
"""LingTu Manager — product-grade control plane.

Always-on process that manages the robot's operation mode.
Exposes REST API + Web UI for customers to control the robot
from any browser — no SSH needed.

Endpoints:
  GET  /                    → Web UI
  GET  /api/status          → system status (mode, SLAM hz, battery, etc.)
  POST /api/mode            → switch mode: {"mode": "map|nav|explore|idle"}
  POST /api/map/save        → save map: {"name": "lab_01"}
  POST /api/map/use         → activate map: {"name": "lab_01"}
  GET  /api/map/list        → list available maps
  POST /api/navigate        → send goal: {"x": 5.0, "y": 3.0} or {"instruction": "go to the door"}
  POST /api/stop            → emergency stop
  POST /api/rerun           → toggle rerun: {"enabled": true}
  GET  /api/stream          → SSE telemetry stream

Boot:
  sudo systemctl enable lingtu-manager
  # Starts on boot, manages everything else
"""

import asyncio
import json
import logging
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("lingtu-manager")

app = FastAPI(title="LingTu Robot Manager", version="1.0.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

# ── Config ───────────────────────────────────────────────────────────────────

NAV_DIR = os.environ.get("NAV_DIR", "/home/sunrise/data/SLAM/navigation")
MAP_DIR = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps"))
MANAGER_PORT = int(os.environ.get("LINGTU_PORT", "5050"))

# ── State ────────────────────────────────────────────────────────────────────

class RobotState:
    mode: str = "idle"          # idle | map | nav | explore
    slam_profile: str = "none"  # none | fastlio2 | localizer
    active_map: str = ""
    lingtu_pid: Optional[int] = None
    last_error: str = ""

state = RobotState()


# ── Service helpers ──────────────────────────────────────────────────────────

def _systemctl(action: str, service: str, timeout: int = 15) -> bool:
    try:
        result = subprocess.run(
            ["sudo", "systemctl", action, service],
            capture_output=True, text=True, timeout=timeout)
        return result.returncode == 0
    except Exception as e:
        logger.error("systemctl %s %s: %s", action, service, e)
        return False


def _is_active(service: str) -> bool:
    try:
        result = subprocess.run(
            ["systemctl", "is-active", service],
            capture_output=True, text=True, timeout=5)
        return result.stdout.strip() == "active"
    except Exception:
        return False


def _start_lingtu(profile: str, extra_args: str = "") -> bool:
    """Start lingtu navigation system as background process."""
    if state.lingtu_pid and _pid_alive(state.lingtu_pid):
        _stop_lingtu()

    cmd = f"cd {NAV_DIR} && python3 main_nav.py {profile} --no-repl {extra_args}"
    try:
        proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=open("/tmp/lingtu_nav.log", "a"),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        state.lingtu_pid = proc.pid
        logger.info("Started lingtu %s (PID %d)", profile, proc.pid)
        return True
    except Exception as e:
        state.last_error = str(e)
        logger.error("Failed to start lingtu: %s", e)
        return False


def _stop_lingtu() -> bool:
    if state.lingtu_pid and _pid_alive(state.lingtu_pid):
        try:
            os.killpg(os.getpgid(state.lingtu_pid), 15)  # SIGTERM
            time.sleep(2)
            if _pid_alive(state.lingtu_pid):
                os.killpg(os.getpgid(state.lingtu_pid), 9)  # SIGKILL
            logger.info("Stopped lingtu (PID %d)", state.lingtu_pid)
        except Exception as e:
            logger.warning("Stop lingtu: %s", e)
    state.lingtu_pid = None
    return True


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except (OSError, ProcessLookupError):
        return False


# ── Mode switching ───────────────────────────────────────────────────────────

def _switch_to_idle():
    _stop_lingtu()
    _systemctl("stop", "slam")
    _systemctl("stop", "slam_pgo")
    state.mode = "idle"
    state.slam_profile = "none"


def _switch_to_map():
    _stop_lingtu()
    _systemctl("start", "slam")
    _systemctl("start", "slam_pgo")
    state.mode = "map"
    state.slam_profile = "fastlio2"


def _switch_to_nav():
    _stop_lingtu()
    # Check active map exists
    active_tomo = os.path.join(MAP_DIR, "active", "tomogram.pickle")
    if not os.path.exists(active_tomo):
        state.last_error = "No active map. Build a map first."
        return False
    _systemctl("stop", "slam_pgo")
    _systemctl("start", "slam")  # localizer mode uses same service
    _start_lingtu("nav")
    state.mode = "nav"
    state.slam_profile = "localizer"
    return True


def _switch_to_explore():
    _stop_lingtu()
    _systemctl("start", "slam")
    _systemctl("stop", "slam_pgo")
    _start_lingtu("explore")
    state.mode = "explore"
    state.slam_profile = "fastlio2"
    return True


# ── Map management ───────────────────────────────────────────────────────────

def _list_maps() -> list:
    maps = []
    if not os.path.isdir(MAP_DIR):
        return maps
    active_name = ""
    active_link = os.path.join(MAP_DIR, "active")
    if os.path.islink(active_link):
        active_name = os.path.basename(os.readlink(active_link))
    for d in sorted(os.listdir(MAP_DIR)):
        dp = os.path.join(MAP_DIR, d)
        if not os.path.isdir(dp) or d in ("active", "_staging"):
            continue
        maps.append({
            "name": d,
            "active": d == active_name,
            "has_pcd": os.path.exists(os.path.join(dp, "map.pcd")),
            "has_tomogram": os.path.exists(os.path.join(dp, "tomogram.pickle")),
        })
    return maps


def _save_map(name: str) -> dict:
    map_dir = os.path.join(MAP_DIR, name)
    os.makedirs(map_dir, exist_ok=True)
    pcd_path = os.path.join(map_dir, "map.pcd")
    # Must source ROS2 env for ros2 CLI
    cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source /opt/nav/install/setup.bash 2>/dev/null; "
        f"ros2 service call /nav/save_map interface/srv/SaveMaps "
        f"\"{{file_path: '{pcd_path}'}}\""
    )
    try:
        result = subprocess.run(
            ["bash", "-c", cmd],
            capture_output=True, text=True, timeout=30)
        if result.returncode != 0:
            return {"success": False, "error": result.stderr[:200]}
        return {"success": True, "pcd": pcd_path}
    except Exception as e:
        return {"success": False, "error": str(e)}


def _use_map(name: str) -> dict:
    map_path = os.path.join(MAP_DIR, name)
    if not os.path.isdir(map_path):
        return {"success": False, "error": f"Map not found: {name}"}
    active_link = os.path.join(MAP_DIR, "active")
    if os.path.islink(active_link) or os.path.exists(active_link):
        os.remove(active_link)
    os.symlink(map_path, active_link)
    state.active_map = name
    return {"success": True, "active": name}


# ── API Routes ───────────────────────────────────────────────────────────────

@app.get("/api/status")
async def api_status():
    return {
        "mode": state.mode,
        "slam_profile": state.slam_profile,
        "active_map": state.active_map,
        "services": {
            "slam": _is_active("slam"),
            "slam_pgo": _is_active("slam_pgo"),
            "lidar": _is_active("lidar"),
            "camera": _is_active("camera"),
            "brainstem": _is_active("brainstem"),
        },
        "lingtu_running": state.lingtu_pid is not None and _pid_alive(state.lingtu_pid),
        "last_error": state.last_error,
    }


@app.post("/api/mode")
async def api_mode(request: Request):
    body = await request.json()
    mode = body.get("mode", "")
    state.last_error = ""

    if mode == "idle":
        _switch_to_idle()
    elif mode == "map":
        _switch_to_map()
    elif mode == "nav":
        if not _switch_to_nav():
            return JSONResponse({"success": False, "error": state.last_error}, status_code=400)
    elif mode == "explore":
        _switch_to_explore()
    else:
        return JSONResponse({"success": False, "error": f"Unknown mode: {mode}"}, status_code=400)

    return {"success": True, "mode": state.mode}


@app.get("/api/map/list")
async def api_map_list():
    return {"maps": _list_maps()}


@app.post("/api/map/save")
async def api_map_save(request: Request):
    body = await request.json()
    name = body.get("name", "")
    if not name:
        return JSONResponse({"success": False, "error": "Missing map name"}, status_code=400)
    return _save_map(name)


@app.post("/api/map/use")
async def api_map_use(request: Request):
    body = await request.json()
    name = body.get("name", "")
    if not name:
        return JSONResponse({"success": False, "error": "Missing map name"}, status_code=400)
    return _use_map(name)


@app.post("/api/stop")
async def api_stop():
    """Emergency stop — halt all motion."""
    # Send stop to brainstem
    try:
        import grpc
        sys.path.insert(0, "/opt/nova/brainstem/v2.0.0/han_dog_message/python")
        from han_dog_message import cms_pb2_grpc
        from google.protobuf.empty_pb2 import Empty
        channel = grpc.insecure_channel("127.0.0.1:13145")
        stub = cms_pb2_grpc.CmsStub(channel)
        stub.SitDown(Empty(), timeout=3)
        channel.close()
    except Exception as e:
        logger.warning("Stop via brainstem failed: %s", e)
    return {"success": True, "action": "emergency_stop"}


@app.post("/api/navigate")
async def api_navigate(request: Request):
    body = await request.json()
    if state.mode != "nav":
        return JSONResponse({"success": False, "error": "Not in nav mode"}, status_code=400)
    # Forward to lingtu via its Gateway API
    try:
        import httpx
        async with httpx.AsyncClient() as client:
            resp = await client.post("http://127.0.0.1:5051/api/v1/goal", json=body, timeout=5)
            return resp.json()
    except Exception as e:
        return JSONResponse({"success": False, "error": str(e)}, status_code=500)


@app.get("/api/stream")
async def api_stream():
    """SSE telemetry stream."""
    async def generate():
        while True:
            data = {
                "mode": state.mode,
                "slam_active": _is_active("slam"),
                "timestamp": time.time(),
            }
            yield f"data: {json.dumps(data)}\n\n"
            await asyncio.sleep(1)
    return StreamingResponse(generate(), media_type="text/event-stream")


# ── Web UI ───────────────────────────────────────────────────────────────────

@app.get("/", response_class=HTMLResponse)
async def web_ui():
    return WEB_UI_HTML


WEB_UI_HTML = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LingTu 灵途</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { font-family: -apple-system, 'Helvetica Neue', sans-serif; background: #1a1a2e; color: #eee; }
  .header { background: #16213e; padding: 16px 24px; display: flex; align-items: center; gap: 16px; }
  .header h1 { font-size: 20px; font-weight: 600; }
  .header .mode { background: #0f3460; padding: 4px 12px; border-radius: 12px; font-size: 13px; }
  .container { max-width: 800px; margin: 20px auto; padding: 0 16px; }
  .card { background: #16213e; border-radius: 12px; padding: 20px; margin-bottom: 16px; }
  .card h2 { font-size: 15px; color: #a0a0b0; margin-bottom: 12px; text-transform: uppercase; letter-spacing: 1px; }
  .btn-group { display: flex; gap: 8px; flex-wrap: wrap; }
  .btn { padding: 12px 24px; border: none; border-radius: 8px; font-size: 15px; cursor: pointer; font-weight: 500; transition: all 0.2s; }
  .btn:active { transform: scale(0.96); }
  .btn-map { background: #e94560; color: white; }
  .btn-nav { background: #0f3460; color: white; border: 1px solid #533483; }
  .btn-explore { background: #533483; color: white; }
  .btn-idle { background: #333; color: #aaa; }
  .btn-stop { background: #ff0000; color: white; font-size: 18px; padding: 16px 32px; width: 100%; }
  .btn-save { background: #0a9396; color: white; }
  .btn-use { background: #005f73; color: white; }
  .status { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  .status-item { display: flex; justify-content: space-between; padding: 8px 12px; background: #1a1a2e; border-radius: 6px; }
  .dot { width: 10px; height: 10px; border-radius: 50%; display: inline-block; }
  .dot-on { background: #00ff88; }
  .dot-off { background: #ff4444; }
  .map-list { margin-top: 8px; }
  .map-item { display: flex; justify-content: space-between; align-items: center; padding: 8px 12px; background: #1a1a2e; border-radius: 6px; margin-bottom: 4px; }
  .map-item.active { border-left: 3px solid #00ff88; }
  .nav-input { display: flex; gap: 8px; margin-top: 8px; }
  .nav-input input { flex: 1; padding: 10px; background: #1a1a2e; border: 1px solid #333; border-radius: 6px; color: white; font-size: 14px; }
  .nav-input .btn { flex-shrink: 0; }
  #log { font-family: monospace; font-size: 12px; color: #888; max-height: 100px; overflow-y: auto; margin-top: 8px; padding: 8px; background: #111; border-radius: 6px; }
</style>
</head>
<body>
<div class="header">
  <h1>LingTu 灵途</h1>
  <span class="mode" id="modeLabel">--</span>
</div>

<div class="container">
  <!-- Emergency Stop -->
  <div class="card">
    <button class="btn btn-stop" onclick="apiPost('/api/stop')">STOP 急停</button>
  </div>

  <!-- Mode Switching -->
  <div class="card">
    <h2>Operation Mode</h2>
    <div class="btn-group">
      <button class="btn btn-map" onclick="setMode('map')">建图</button>
      <button class="btn btn-nav" onclick="setMode('nav')">导航</button>
      <button class="btn btn-explore" onclick="setMode('explore')">探索</button>
      <button class="btn btn-idle" onclick="setMode('idle')">待机</button>
    </div>
  </div>

  <!-- Status -->
  <div class="card">
    <h2>System Status</h2>
    <div class="status" id="statusGrid"></div>
  </div>

  <!-- Map Management -->
  <div class="card">
    <h2>Maps</h2>
    <div class="btn-group">
      <button class="btn btn-save" onclick="saveMap()">Save Map</button>
    </div>
    <div class="map-list" id="mapList"></div>
  </div>

  <!-- Navigation -->
  <div class="card">
    <h2>Navigate</h2>
    <div class="nav-input">
      <input type="text" id="navTarget" placeholder="go to the door / 5.0 3.0">
      <button class="btn btn-nav" onclick="navigate()">Go</button>
    </div>
  </div>

  <!-- Log -->
  <div id="log"></div>
</div>

<script>
const API = '';

function log(msg) {
  const el = document.getElementById('log');
  const t = new Date().toLocaleTimeString();
  el.innerHTML = `[${t}] ${msg}<br>` + el.innerHTML;
  el.innerHTML = el.innerHTML.split('<br>').slice(0, 20).join('<br>');
}

async function apiPost(url, body = {}) {
  try {
    const r = await fetch(API + url, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
    const data = await r.json();
    log(`${url}: ${JSON.stringify(data)}`);
    refresh();
    return data;
  } catch(e) { log(`ERROR: ${e}`); }
}

async function setMode(mode) { await apiPost('/api/mode', {mode}); }

async function saveMap() {
  const name = prompt('Map name:', 'map_' + Date.now());
  if (name) await apiPost('/api/map/save', {name});
}

async function navigate() {
  const target = document.getElementById('navTarget').value;
  if (!target) return;
  const parts = target.split(/\s+/);
  if (parts.length >= 2 && !isNaN(parts[0])) {
    await apiPost('/api/navigate', {x: parseFloat(parts[0]), y: parseFloat(parts[1])});
  } else {
    await apiPost('/api/navigate', {instruction: target});
  }
}

async function refresh() {
  try {
    const r = await fetch(API + '/api/status');
    const s = await r.json();
    document.getElementById('modeLabel').textContent = s.mode.toUpperCase();
    const grid = document.getElementById('statusGrid');
    grid.innerHTML = Object.entries(s.services).map(([k, v]) =>
      `<div class="status-item"><span>${k}</span><span class="dot ${v ? 'dot-on' : 'dot-off'}"></span></div>`
    ).join('');

    const r2 = await fetch(API + '/api/map/list');
    const m = await r2.json();
    const mapList = document.getElementById('mapList');
    mapList.innerHTML = m.maps.map(mp =>
      `<div class="map-item ${mp.active ? 'active' : ''}">
        <span>${mp.name} ${mp.active ? '(active)' : ''}</span>
        <button class="btn btn-use" onclick="apiPost('/api/map/use', {name:'${mp.name}'})" style="padding:4px 12px;font-size:12px;">Use</button>
      </div>`
    ).join('');
  } catch(e) {}
}

setInterval(refresh, 3000);
refresh();
</script>
</body>
</html>
"""

# ── Main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    # Detect active map on startup
    active_link = os.path.join(MAP_DIR, "active")
    if os.path.islink(active_link):
        state.active_map = os.path.basename(os.readlink(active_link))

    # Detect current mode from running services
    if _is_active("slam"):
        state.mode = "map"
        state.slam_profile = "fastlio2"
    else:
        state.mode = "idle"

    logger.info("LingTu Manager starting on port %d (mode=%s)", MANAGER_PORT, state.mode)
    uvicorn.run(app, host="0.0.0.0", port=MANAGER_PORT, log_level="warning")
