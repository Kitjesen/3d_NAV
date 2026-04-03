"""Cross-module integration tests for LingTu navigation system (tests 21-30)."""

import sys, os
sys.path.insert(0, "src")
for d in ["src/semantic/perception", "src/semantic/planner", "src/semantic/common"]:
    if os.path.isdir(d): sys.path.insert(0, d)
for k in ["MOONSHOT_API_KEY", "OPENAI_API_KEY", "ANTHROPIC_API_KEY", "DASHSCOPE_API_KEY"]:
    os.environ.pop(k, None)
import logging; logging.basicConfig(level=logging.ERROR)

import time
import threading
import numpy as np

# Capture warnings during build for test 30
_build_warnings = []
_orig_warn = logging.Logger.warning
def _capture_warn(self, msg, *args, **kwargs):
    try:
        formatted = str(msg) % args if args else str(msg)
    except Exception:
        formatted = str(msg)
    _build_warnings.append(formatted)
    _orig_warn(self, msg, *args, **kwargs)
logging.Logger.warning = _capture_warn

# Patch server start methods to avoid port binding / blocking in CI
import gateway.gateway_module as _gw_mod
import drivers.teleop_module as _tp_mod
import gateway.mcp_server as _mcp_mod
_gw_mod.GatewayModule.start = lambda self: None
_tp_mod.TeleopModule.start = lambda self: None
_mcp_mod.MCPServerModule.start = lambda self: None

from core.blueprints.full_stack import full_stack_blueprint
bp = full_stack_blueprint(
    robot="stub", slam_profile="none",
    enable_native=False, enable_semantic=True, enable_gateway=True,
)
system = bp.build()
system.start()
time.sleep(0.3)
logging.Logger.warning = _orig_warn

results = {}
def test(num, name, passed, detail=""):
    tag = "PASS" if passed else "FAIL"
    results[num] = passed
    msg = "Test %d: %s -- %s" % (num, tag, name)
    if detail and not passed:
        msg += "  [%s]" % detail
    print(msg, flush=True)

# ==== Test 20: Non-native stack builds Python autonomy chain ====
try:
    nav = system.get_module("NavigationModule")
    lp = system.get_module("LocalPlannerModule")
    pf = system.get_module("PathFollowerModule")
    wp_conns = [c for c in system.connections
                if c[0] == "NavigationModule" and c[1] == "waypoint"
                   and c[2] == "LocalPlannerModule" and c[3] == "waypoint"]
    path_conns = [c for c in system.connections
                  if c[0] == "LocalPlannerModule" and c[1] == "local_path"
                     and c[2] == "PathFollowerModule" and c[3] == "local_path"]
    cmd_conns = [c for c in system.connections
                 if c[0] == "PathFollowerModule" and c[1] == "cmd_vel"
                    and c[2] == "StubDogModule" and c[3] == "cmd_vel"]
    test(20, "Non-native stack uses Python autonomy chain",
         nav._enable_ros2_bridge is False
         and lp._backend == "simple"
         and pf._backend == "pid"
         and len(wp_conns) > 0
         and len(path_conns) > 0
         and len(cmd_conns) > 0,
         "nav_ros2=%s lp=%s pf=%s wires=%d/%d/%d" % (
             nav._enable_ros2_bridge, lp._backend, pf._backend,
             len(wp_conns), len(path_conns), len(cmd_conns)))
except Exception as e:
    test(20, "Non-native stack uses Python autonomy chain", False, str(e))

# ==== Test 21: SceneGraph fan-out ====
# PerceptionModule (scene_graph Out) is not in full_stack; DetectorModule
# publishes detections, not scene_graph.  So we simulate fan-out by creating
# a temporary Out[SceneGraph], wiring it to every In[SceneGraph] in the system,
# publishing once, and verifying all targets receive the message.
try:
    from core.msgs.semantic import SceneGraph
    from core.stream import Out as OutPort
    sg = SceneGraph(objects=[], relations=[], regions=[])

    expected_targets = {"SemanticPlannerModule", "SemanticMapperModule",
                        "EpisodicMemoryModule", "VisualServoModule"}
    # Collect In ports
    target_ports = {}
    for tname in expected_targets:
        if tname in system.modules:
            mod = system.modules[tname]
            if "scene_graph" in mod.ports_in:
                target_ports[tname] = mod.ports_in["scene_graph"]

    # Create a temporary Out and wire to all targets (simulating fan-out)
    fake_out = OutPort(SceneGraph, "scene_graph_test_source")
    for tname, in_port in target_ports.items():
        fake_out._add_callback(in_port._deliver)

    before = {tn: tp.msg_count for tn, tp in target_ports.items()}
    fake_out.publish(sg)
    time.sleep(0.05)

    received = 0
    for tname, bc in before.items():
        if target_ports[tname].msg_count > bc:
            received += 1

    test(21, "SceneGraph fan-out",
         received >= 2 and len(target_ports) >= 2,
         "delivered to %d/%d targets (%s)" % (received, len(target_ports),
                                               list(target_ports.keys())))
except Exception as e:
    test(21, "SceneGraph fan-out", False, str(e))

# ==== Test 22: Odometry fan-out ====
try:
    from core.msgs.nav import Odometry
    from core.msgs.geometry import Pose, Twist, Vector3, Quaternion
    odom = Odometry(
        pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
        twist=Twist(linear=Vector3(0.1, 0, 0), angular=Vector3(0, 0, 0)),
    )
    odom_conns = [c for c in system.connections if c[1] == "odometry" and c[3] == "odometry"]
    odom_targets = set(c[2] for c in odom_conns)
    expected_odom = {"NavigationModule", "SemanticMapperModule", "VectorMemoryModule"}
    found_odom = odom_targets & expected_odom

    odom_sources = set(c[0] for c in odom_conns)
    if odom_sources:
        src_name = list(odom_sources)[0]
        src_port = system.modules[src_name].ports_out["odometry"]
        before = {}
        for tname in found_odom:
            if tname in system.modules:
                before[tname] = system.modules[tname].ports_in["odometry"].msg_count
        src_port.publish(odom)
        time.sleep(0.05)
        received = 0
        for tname, bc in before.items():
            if system.modules[tname].ports_in["odometry"].msg_count > bc:
                received += 1
        test(22, "Odometry fan-out", received >= 2,
             "received=%d/%d, targets=%s" % (received, len(before), odom_targets))
    else:
        test(22, "Odometry fan-out", False, "No odometry source")
except Exception as e:
    test(22, "Odometry fan-out", False, str(e))

# ==== Test 23: on_system_modules VectorMemory ref ====
try:
    sp = system.get_module("SemanticPlannerModule")
    vm = system.get_module("VectorMemoryModule")
    has_ref = sp._vector_memory is not None
    is_vm = sp._vector_memory is vm
    test(23, "on_system_modules: _vector_memory is VectorMemoryModule",
         has_ref and is_vm, "has_ref=%s, is_vm=%s" % (has_ref, is_vm))
except Exception as e:
    test(23, "on_system_modules VectorMemory ref", False, str(e))

# ==== Test 24: Costmap chain ====
try:
    from nav.occupancy_grid_module import OccupancyGridModule
    from core.msgs.sensor import PointCloud2
    ogm = OccupancyGridModule(resolution=0.5, map_radius=5.0, z_min=0.1, z_max=2.0)
    ogm.setup()
    pts = np.array([[1.0, 1.0, 0.5], [1.5, 1.5, 0.8], [2.0, 2.0, 1.0], [-1.0, -1.0, 0.3]], dtype=np.float32)
    cloud = PointCloud2(points=pts)
    before_cost = ogm.costmap.msg_count
    ogm._on_cloud(cloud)
    after_cost = ogm.costmap.msg_count
    test(24, "Costmap chain: _on_cloud -> costmap published",
         after_cost > before_cost, "before=%d, after=%d" % (before_cost, after_cost))
except Exception as e:
    test(24, "Costmap chain", False, str(e))

# ==== Test 25: Safety stop wiring ====
try:
    stop_conns = [c for c in system.connections
                  if c[0] == "SafetyRingModule" and c[1] == "stop_cmd"]
    stop_targets = set(c[2] for c in stop_conns)
    has_driver = "StubDogModule" in stop_targets
    has_nav = "NavigationModule" in stop_targets
    test(25, "Safety stop: wired to driver + NavigationModule",
         has_driver and has_nav, "targets=%s" % stop_targets)
except Exception as e:
    test(25, "Safety stop wiring", False, str(e))

# ==== Test 26: Teleop priority ====
try:
    teleop = system.get_module("TeleopModule")
    teleop._on_joy({"lx": 0.5, "ly": 0.0, "az": 0.1})
    active_after = teleop._active
    teleop._last_joy_time = time.time() - teleop._release_timeout - 1.0
    teleop._check_idle()
    released = not teleop._active
    test(26, "Teleop priority: _on_joy active, check_idle releases",
         active_after and released,
         "active_after=%s, released=%s" % (active_after, released))
except Exception as e:
    test(26, "Teleop priority", False, str(e))

# ==== Test 27: Instruction fan-in ====
try:
    instr_conns = [c for c in system.connections
                   if c[3] == "instruction" and c[2] == "SemanticPlannerModule"]
    instr_sources = set(c[0] for c in instr_conns)
    has_gw = "GatewayModule" in instr_sources
    has_mcp = "MCPServerModule" in instr_sources
    test(27, "Instruction fan-in: Gateway + MCP -> SemanticPlanner",
         has_gw and has_mcp, "sources=%s" % instr_sources)
except Exception as e:
    test(27, "Instruction fan-in", False, str(e))

# ==== Test 28: goal_pose SemanticPlanner -> Navigation ====
try:
    from core.msgs.geometry import PoseStamped
    goal_conns = [c for c in system.connections
                  if c[0] == "SemanticPlannerModule" and c[1] == "goal_pose"
                     and c[2] == "NavigationModule" and c[3] == "goal_pose"]
    nav = system.get_module("NavigationModule")
    sp2 = system.get_module("SemanticPlannerModule")
    before_gp = nav.ports_in["goal_pose"].msg_count
    goal = PoseStamped(
        pose=Pose(position=Vector3(5.0, 3.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
        frame_id="map",
    )
    sp2.goal_pose.publish(goal)
    time.sleep(0.05)
    after_gp = nav.ports_in["goal_pose"].msg_count
    test(28, "goal_pose: SemanticPlanner -> NavigationModule",
         after_gp > before_gp and len(goal_conns) > 0,
         "wire=%d, before=%d, after=%d" % (len(goal_conns), before_gp, after_gp))
except Exception as e:
    test(28, "goal_pose fan-in", False, str(e))

# ==== Test 29: @skill MCP discovery ====
try:
    mcp = system.get_module("MCPServerModule")
    from gateway.mcp_server import TOOLS
    static_count = len(TOOLS)
    dynamic_count = len(mcp._dynamic_tools)
    total = static_count + dynamic_count
    test(29, "@skill MCP discovery: tool count > 0",
         total > 0, "static=%d, dynamic=%d, total=%d" % (static_count, dynamic_count, total))
except Exception as e:
    test(29, "@skill MCP discovery", False, str(e))

# ==== Test 30: Zero auto_wire ambiguity ====
try:
    ambiguity_warnings = [w for w in _build_warnings if "ambiguity" in w.lower()]
    test(30, "Zero auto_wire ambiguity warnings",
         len(ambiguity_warnings) == 0,
         "found %d: %s" % (len(ambiguity_warnings), ambiguity_warnings[:3]))
except Exception as e:
    test(30, "Zero auto_wire ambiguity", False, str(e))

# ==== Summary ====
passed = sum(1 for v in results.values() if v)
failed = sum(1 for v in results.values() if not v)
total = len(results)
print(flush=True)
print("=" * 60, flush=True)
print("Summary: %d/%d PASSED, %d/%d FAILED" % (passed, total, failed, total), flush=True)
print("=" * 60, flush=True)

# Stop system (may block on server threads, so use os._exit as fallback)
def _final_stop():
    time.sleep(5)
    os._exit(0 if failed == 0 else 1)
t = threading.Thread(target=_final_stop, daemon=True)
t.start()
try:
    system.stop()
except Exception:
    pass
sys.exit(0 if failed == 0 else 1)
