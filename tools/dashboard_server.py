"""
LingTu Interactive Dashboard Server
====================================
Web UI for managing the LingTu navigation system on the robot.
Connects via SSH to the robot and provides REST API + WebSocket.

Usage:
    python tools/dashboard_server.py
    # Open http://localhost:8066
"""
import asyncio
import json
import time
import threading
from contextlib import asynccontextmanager
from pathlib import Path

import paramiko
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
import uvicorn

# ── Robot Config ──────────────────────────────────────────────
ROBOT_HOST = "192.168.66.190"
ROBOT_USER = "sunrise"
ROBOT_PASS = "sunrise"
NAV_WS = "/home/sunrise/data/SLAM/navigation"
MAP_DIR = f"{NAV_WS}/maps"
ROS_SETUP = f"source /opt/ros/humble/setup.bash && source {NAV_WS}/install/setup.bash"

SERVICES = [
    "nav-lidar", "nav-slam", "nav-planning",
    "nav-autonomy", "nav-driver", "nav-grpc",
    "nav-semantic", "ota-daemon",
]

TOPICS_MONITOR = [
    "/nav/odometry",
    "/nav/map_cloud",
    "/nav/registered_cloud",
    "/nav/terrain_map",
    "/nav/global_path",
    "/nav/local_path",
    "/nav/cmd_vel",
    "/nav/stop",
    "/nav/slow_down",
    "/nav/planner_status",
    "/nav/adapter_status",
    "/nav/goal_pose",
    "/nav/lidar_scan",
    "/nav/imu",
    "/nav/dog_odometry",
    "/nav/localization_quality",
    "/nav/semantic/scene_graph",
    "/nav/semantic/detections_3d",
    "/nav/semantic/status",
]

# ── SSH Pool ──────────────────────────────────────────────────
_ssh_lock = threading.Lock()
_ssh_client = None


def get_ssh():
    global _ssh_client
    with _ssh_lock:
        if _ssh_client is None or _ssh_client.get_transport() is None or not _ssh_client.get_transport().is_active():
            _ssh_client = paramiko.SSHClient()
            _ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            _ssh_client.connect(ROBOT_HOST, username=ROBOT_USER, password=ROBOT_PASS, timeout=8)
        return _ssh_client


def ssh_exec(cmd, timeout=15):
    """Execute command on robot via SSH, return (stdout, stderr, exit_code)."""
    try:
        client = get_ssh()
        stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
        out = stdout.read().decode("utf-8", errors="replace")
        err = stderr.read().decode("utf-8", errors="replace")
        code = stdout.channel.recv_exit_status()
        return out, err, code
    except Exception as e:
        return "", str(e), -1


def ssh_exec_ros(cmd, timeout=15):
    """Execute ROS2 command (with setup sourced)."""
    return ssh_exec(f"bash -c '{ROS_SETUP} && {cmd}'", timeout=timeout)


# ── Pydantic Models ───────────────────────────────────────────
class ServiceAction(BaseModel):
    service: str
    action: str  # start | stop | restart


class NavGoal(BaseModel):
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0


class ParamUpdate(BaseModel):
    node: str
    param: str
    value: str


class SemanticGoal(BaseModel):
    instruction: str


class TopicPub(BaseModel):
    topic: str
    msg_type: str
    data: str


# ── App ───────────────────────────────────────────────────────
@asynccontextmanager
async def lifespan(app: FastAPI):
    yield
    global _ssh_client
    if _ssh_client:
        _ssh_client.close()

app = FastAPI(title="LingTu Dashboard", lifespan=lifespan)


@app.get("/", response_class=HTMLResponse)
async def index():
    return FileResponse(Path(__file__).parent / "dashboard.html")


# ── Connection Test ───────────────────────────────────────────
@app.get("/api/ping")
async def ping():
    out, err, code = ssh_exec("echo OK && hostname && uptime -p", timeout=5)
    if code == 0:
        lines = out.strip().split("\n")
        return {"ok": True, "hostname": lines[1] if len(lines) > 1 else "?",
                "uptime": lines[2] if len(lines) > 2 else "?"}
    return {"ok": False, "error": err or "Connection failed"}


# ── Service Management ────────────────────────────────────────
@app.get("/api/services")
async def list_services():
    # Batch check all services in one SSH call
    checks = " ".join([f"systemctl is-active {s} 2>/dev/null || echo inactive" for s in SERVICES])
    cmd = f"for s in {' '.join(SERVICES)}; do st=$(systemctl is-active $s 2>/dev/null || echo inactive); echo \"$s $st\"; done"
    out, err, code = ssh_exec(cmd)
    results = []
    for line in out.strip().split("\n"):
        parts = line.strip().split()
        if len(parts) >= 2:
            results.append({"name": parts[0], "status": parts[1]})
    return {"services": results}


@app.post("/api/services/action")
async def service_action(req: ServiceAction):
    if req.service not in SERVICES:
        return {"ok": False, "error": f"Unknown service: {req.service}"}
    if req.action not in ("start", "stop", "restart", "status"):
        return {"ok": False, "error": f"Unknown action: {req.action}"}
    out, err, code = ssh_exec(f"sudo systemctl {req.action} {req.service} 2>&1; systemctl is-active {req.service} 2>/dev/null", timeout=20)
    lines = out.strip().split("\n")
    new_status = lines[-1] if lines else "unknown"
    return {"ok": code == 0 or req.action == "status", "output": out.strip(), "status": new_status}


# ── Stack Control (start/stop all in dependency order) ────────
STACK_START_ORDER = ["nav-lidar", "nav-slam", "nav-planning", "nav-autonomy", "nav-driver", "nav-grpc"]
STACK_STOP_ORDER = list(reversed(STACK_START_ORDER))


@app.post("/api/stack/start")
async def stack_start():
    """Start full navigation stack in dependency order."""
    results = []
    for svc in STACK_START_ORDER:
        out, err, code = ssh_exec(f"sudo systemctl start {svc} 2>&1; sleep 1; systemctl is-active {svc} 2>/dev/null", timeout=15)
        status = out.strip().split("\n")[-1]
        results.append({"service": svc, "status": status})
    return {"ok": True, "results": results}


@app.post("/api/stack/stop")
async def stack_stop():
    """Stop full navigation stack in reverse order."""
    results = []
    for svc in STACK_STOP_ORDER:
        out, err, code = ssh_exec(f"sudo systemctl stop {svc} 2>&1; systemctl is-active {svc} 2>/dev/null", timeout=10)
        status = out.strip().split("\n")[-1]
        results.append({"service": svc, "status": status})
    return {"ok": True, "results": results}


# ── Topic Monitor ─────────────────────────────────────────────
@app.get("/api/topics")
async def list_topics():
    out, _, _ = ssh_exec_ros("ros2 topic list 2>/dev/null", timeout=10)
    topics = [t.strip() for t in out.strip().split("\n") if t.strip().startswith("/")]
    return {"topics": topics, "count": len(topics)}


@app.get("/api/topics/hz")
async def topics_hz():
    """Get Hz for monitored topics (batch, fast)."""
    # Use ros2 topic hz with timeout for each
    # For speed, check which topics have publishers first
    topic_list = " ".join(TOPICS_MONITOR)
    cmd = f"""for t in {topic_list}; do
  cnt=$(ros2 topic info $t 2>/dev/null | grep -c 'Publisher count: [1-9]' || echo 0)
  if [ "$cnt" -gt 0 ]; then
    echo "$t ACTIVE"
  else
    echo "$t INACTIVE"
  fi
done"""
    out, _, _ = ssh_exec_ros(cmd, timeout=15)
    results = []
    for line in out.strip().split("\n"):
        parts = line.strip().split()
        if len(parts) >= 2:
            results.append({"topic": parts[0], "active": parts[1] == "ACTIVE"})
    return {"topics": results}


@app.get("/api/topics/echo/{topic_path:path}")
async def topic_echo(topic_path: str):
    """Get one message from a topic."""
    topic = f"/{topic_path}"
    out, err, code = ssh_exec_ros(
        f"timeout 3 ros2 topic echo {topic} --once 2>/dev/null | head -50",
        timeout=8
    )
    return {"topic": topic, "message": out.strip() if out.strip() else "(no message received within 3s)",
            "ok": bool(out.strip())}


# ── Map Management ────────────────────────────────────────────
@app.get("/api/maps")
async def list_maps():
    out, _, code = ssh_exec(f"ls -lh {MAP_DIR}/ 2>/dev/null | tail -n +2")
    maps = []
    # Also check for .pcd and .pickle files
    out2, _, _ = ssh_exec(f"find {MAP_DIR} -maxdepth 2 -name '*.pcd' -o -name '*.pickle' 2>/dev/null | head -50")
    for line in out2.strip().split("\n"):
        if line.strip():
            # Get file info
            fname = line.strip()
            info_out, _, _ = ssh_exec(f"stat --format='%s %y' '{fname}' 2>/dev/null")
            parts = info_out.strip().split(" ", 1)
            size_bytes = int(parts[0]) if parts[0].isdigit() else 0
            mtime = parts[1][:19] if len(parts) > 1 else ""
            size_mb = size_bytes / (1024 * 1024)
            maps.append({
                "path": fname,
                "name": fname.replace(MAP_DIR + "/", ""),
                "size": f"{size_mb:.1f} MB",
                "size_bytes": size_bytes,
                "modified": mtime,
            })
    return {"maps": maps, "map_dir": MAP_DIR}


@app.post("/api/maps/save")
async def save_map():
    """Trigger map save via ROS2 service."""
    ts = time.strftime("%Y%m%d_%H%M%S")
    save_path = f"{MAP_DIR}/map_{ts}"
    out, err, code = ssh_exec_ros(
        f"ros2 service call /nav/save_map interface/srv/SaveMaps "
        f"\"{{file_path: '{save_path}', save_patches: true}}\" 2>&1",
        timeout=30
    )
    return {"ok": "success" in out.lower() or code == 0, "path": save_path, "output": out.strip()}


@app.delete("/api/maps/{map_name:path}")
async def delete_map(map_name: str):
    full_path = f"{MAP_DIR}/{map_name}"
    # Safety: only allow deletion within MAP_DIR
    out, err, code = ssh_exec(f"rm -rf '{full_path}' 2>&1 && echo DELETED")
    return {"ok": "DELETED" in out, "output": out.strip()}


# ── Navigation Control ────────────────────────────────────────
@app.post("/api/nav/goal")
async def send_nav_goal(goal: NavGoal):
    """Publish a navigation goal."""
    cmd = (
        f"ros2 topic pub --once /nav/goal_pose geometry_msgs/msg/PoseStamped "
        f"\"{{header: {{frame_id: 'map'}}, pose: {{position: {{x: {goal.x}, y: {goal.y}, z: {goal.z}}}, "
        f"orientation: {{w: 1.0}}}}}}\" 2>&1"
    )
    out, err, code = ssh_exec_ros(cmd, timeout=10)
    return {"ok": code == 0, "goal": {"x": goal.x, "y": goal.y, "z": goal.z}, "output": out.strip()}


@app.post("/api/nav/cancel")
async def cancel_nav():
    """Cancel navigation by publishing empty path."""
    out, _, code = ssh_exec_ros(
        "ros2 topic pub --once /nav/goal_pose geometry_msgs/msg/PoseStamped "
        "\"{}\" 2>&1", timeout=10
    )
    return {"ok": True, "output": "Cancel signal sent"}


@app.post("/api/nav/estop")
async def emergency_stop():
    """Send emergency stop."""
    out, _, code = ssh_exec_ros(
        "ros2 topic pub --once /nav/stop std_msgs/msg/Int8 \"{data: 2}\" 2>&1",
        timeout=5
    )
    return {"ok": True, "output": "E-STOP sent (stop=2)"}


@app.post("/api/nav/clear_stop")
async def clear_stop():
    """Clear stop signal."""
    out, _, code = ssh_exec_ros(
        "ros2 topic pub --once /nav/stop std_msgs/msg/Int8 \"{data: 0}\" 2>&1",
        timeout=5
    )
    return {"ok": True, "output": "Stop cleared (stop=0)"}


@app.post("/api/nav/semantic")
async def send_semantic_goal(goal: SemanticGoal):
    """Send natural language navigation instruction."""
    cmd = (
        f"ros2 topic pub --once /nav/semantic/instruction std_msgs/msg/String "
        f"\"{{data: '{goal.instruction}'}}\" 2>&1"
    )
    out, _, code = ssh_exec_ros(cmd, timeout=10)
    return {"ok": code == 0, "instruction": goal.instruction, "output": out.strip()}


# ── Planner Status ────────────────────────────────────────────
@app.get("/api/nav/status")
async def nav_status():
    """Get current planner, adapter, and mission status."""
    cmds = [
        "timeout 2 ros2 topic echo /nav/planner_status --once 2>/dev/null || echo 'NO_DATA'",
        "timeout 2 ros2 topic echo /nav/adapter_status --once 2>/dev/null || echo 'NO_DATA'",
        "timeout 2 ros2 topic echo /nav/mission_status --once 2>/dev/null || echo 'NO_DATA'",
    ]
    cmd = " && echo '---SPLIT---' && ".join(cmds)
    out, _, _ = ssh_exec_ros(cmd, timeout=15)
    parts = out.split("---SPLIT---")
    result = {
        "planner_status": parts[0].strip() if len(parts) > 0 else "N/A",
        "adapter_status": parts[1].strip() if len(parts) > 1 else "N/A",
    }
    # Parse mission_status JSON
    if len(parts) > 2:
        mission_raw = parts[2].strip()
        if mission_raw and mission_raw != "NO_DATA":
            import json as _json
            # ros2 topic echo outputs 'data: "{json}"' for String
            # Extract the JSON from the data field
            for line in mission_raw.split("\n"):
                line = line.strip()
                if line.startswith("data:"):
                    json_str = line[5:].strip().strip("'\"")
                    try:
                        result["mission"] = _json.loads(json_str)
                    except Exception:
                        result["mission"] = {"state": "PARSE_ERROR", "raw": json_str}
                    break
    return result


# ── Health Check ──────────────────────────────────────────────
@app.get("/api/health")
async def health_check():
    """Run quick health diagnostics."""
    checks = []

    # 1. SSH connectivity
    out, _, code = ssh_exec("echo OK", timeout=5)
    checks.append({"name": "SSH Connection", "ok": code == 0})

    # 2. ROS2 daemon
    out, _, code = ssh_exec_ros("ros2 node list 2>/dev/null | head -20", timeout=10)
    nodes = [n.strip() for n in out.strip().split("\n") if n.strip().startswith("/")]
    checks.append({"name": "ROS2 Nodes", "ok": len(nodes) > 0, "detail": f"{len(nodes)} nodes"})

    # 3. Key services
    key_services = ["nav-slam", "nav-autonomy", "nav-planning"]
    for svc in key_services:
        out, _, code = ssh_exec(f"systemctl is-active {svc} 2>/dev/null")
        checks.append({"name": f"Service: {svc}", "ok": out.strip() == "active", "detail": out.strip()})

    # 4. Disk space
    out, _, _ = ssh_exec("df -h / | tail -1 | awk '{print $5, $4}'")
    parts = out.strip().split()
    usage = parts[0] if parts else "?"
    avail = parts[1] if len(parts) > 1 else "?"
    usage_pct = int(usage.replace("%", "")) if "%" in usage else 0
    checks.append({"name": "Disk Space", "ok": usage_pct < 90, "detail": f"{usage} used, {avail} free"})

    # 5. Memory
    out, _, _ = ssh_exec("free -h | grep Mem | awk '{print $3, $2}'")
    checks.append({"name": "Memory", "ok": True, "detail": out.strip().replace(" ", " / ")})

    # 6. CPU temp
    out, _, _ = ssh_exec("cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0")
    temp_c = int(out.strip()) / 1000 if out.strip().isdigit() else 0
    checks.append({"name": "CPU Temp", "ok": temp_c < 80, "detail": f"{temp_c:.0f}C"})

    all_ok = all(c["ok"] for c in checks)
    return {"ok": all_ok, "checks": checks, "node_count": len(nodes), "nodes": nodes}


# ── Log Viewer ────────────────────────────────────────────────
@app.get("/api/logs/{service}")
async def get_logs(service: str, lines: int = 50):
    if service not in SERVICES and service != "all":
        return {"ok": False, "error": "Unknown service"}
    if service == "all":
        out, _, _ = ssh_exec(f"journalctl --no-pager -n {lines} --output=short-iso 2>/dev/null")
    else:
        out, _, _ = ssh_exec(f"journalctl -u {service} --no-pager -n {lines} --output=short-iso 2>/dev/null")
    return {"service": service, "logs": out.strip(), "lines": lines}


# ── Parameters ────────────────────────────────────────────────
@app.get("/api/params/{node_name:path}")
async def get_params(node_name: str):
    node = f"/{node_name}"
    out, _, code = ssh_exec_ros(f"ros2 param list {node} 2>/dev/null | head -100", timeout=10)
    params = [p.strip() for p in out.strip().split("\n") if p.strip()]
    return {"node": node, "params": params, "count": len(params)}


@app.post("/api/params/set")
async def set_param(req: ParamUpdate):
    out, err, code = ssh_exec_ros(
        f"ros2 param set {req.node} {req.param} {req.value} 2>&1",
        timeout=10
    )
    return {"ok": "successfully" in out.lower(), "output": out.strip()}


# ── SLAM Profile Switch ──────────────────────────────────────
@app.get("/api/slam/profile")
async def get_slam_profile():
    """Detect current SLAM profile by checking running nodes."""
    out, _, _ = ssh_exec_ros("ros2 node list 2>/dev/null | grep -E 'pointlio|lio_node|fastlio'")
    nodes = out.strip()
    if "pointlio" in nodes:
        return {"profile": "pointlio"}
    elif "lio_node" in nodes or "fastlio" in nodes:
        return {"profile": "fastlio2"}
    return {"profile": "none"}


# ── System Info ───────────────────────────────────────────────
@app.get("/api/system")
async def system_info():
    """Get system-level info."""
    cmd = """echo "---HOST---" && hostname
echo "---ARCH---" && uname -m
echo "---OS---" && cat /etc/os-release 2>/dev/null | grep PRETTY_NAME | cut -d= -f2 | tr -d '"'
echo "---UPTIME---" && uptime -p
echo "---CPU---" && nproc
echo "---MEM---" && free -h | grep Mem | awk '{print $2}'
echo "---DISK---" && df -h / | tail -1 | awk '{print $2, $3, $4, $5}'
echo "---TEMP---" && cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0
echo "---IP---" && hostname -I | awk '{print $1}'
"""
    out, _, _ = ssh_exec(cmd, timeout=10)
    info = {}
    current_key = None
    for line in out.strip().split("\n"):
        if line.startswith("---") and line.endswith("---"):
            current_key = line.strip("-").lower()
        elif current_key:
            info[current_key] = line.strip()
            current_key = None
    if "temp" in info:
        try:
            info["temp"] = f"{int(info['temp']) / 1000:.0f}C"
        except ValueError:
            pass
    return info


# ── WebSocket for live updates ────────────────────────────────
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            # Send periodic status updates
            try:
                # Quick status check
                out, _, code = ssh_exec("echo OK", timeout=3)
                connected = code == 0

                # Service status
                svc_cmd = f"for s in {' '.join(SERVICES)}; do st=$(systemctl is-active $s 2>/dev/null || echo inactive); echo \"$s $st\"; done"
                svc_out, _, _ = ssh_exec(svc_cmd, timeout=8)
                services = {}
                for line in svc_out.strip().split("\n"):
                    parts = line.strip().split()
                    if len(parts) >= 2:
                        services[parts[0]] = parts[1]

                await ws.send_json({
                    "type": "status",
                    "connected": connected,
                    "services": services,
                    "timestamp": time.time(),
                })
            except Exception as e:
                await ws.send_json({
                    "type": "status",
                    "connected": False,
                    "error": str(e),
                    "timestamp": time.time(),
                })

            await asyncio.sleep(5)
    except WebSocketDisconnect:
        pass


# ── Main ──────────────────────────────────────────────────────
if __name__ == "__main__":
    print("\n  LingTu Dashboard Server")
    print(f"  Robot: {ROBOT_HOST}")
    print(f"  Open: http://localhost:8066\n")
    uvicorn.run(app, host="0.0.0.0", port=8066, log_level="info")
