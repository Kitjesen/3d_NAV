"""Full-stack navigation blueprint — complete pluggable pipeline.

Every component is swappable via config arguments. One call starts everything.

Usage::

    # Real robot (S100P)
    system = full_stack_blueprint(
        robot="thunder", slam_profile="fastlio2",
        detector="bpu", encoder="mobileclip", llm="kimi",
    ).build()
    system.start()

    # Simulation
    system = full_stack_blueprint(
        robot="stub", detector="yoloe", llm="mock",
    ).build()

    # CI / testing
    system = full_stack_blueprint(
        robot="stub", enable_native=False, llm="mock",
    ).build()

Pipeline::

    AutonomyModule (C++: SLAM + Terrain + LocalPlanner + PathFollower)
                         ↕ DDS
    Camera → DetectorModule → EncoderModule
                         ↓ callback
    PerceptionService(tracker) → SceneGraph
                         ↓ transport (decoupled)
    SemanticPlannerModule ←→ LLMModule
                         ↓
    NavigationModule (plan + track + FSM)
                         ↓
    ThunderDriver → cmd_vel → Robot
                         ↓
    SafetyRingModule (reflex + eval + dialogue)
                         ↓
    GatewayModule (HTTP/WS/SSE) → App/Console
    MCPServerModule (16 MCP tools) → AI Agents
"""

from __future__ import annotations

import logging
import os
from typing import Any

from core.blueprint import Blueprint
from core.config import get_config
from core.registry import get, auto_select

_DEFAULT_SEMANTIC_DIR = os.path.join(os.path.expanduser("~"), ".nova", "semantic")

logger = logging.getLogger(__name__)


def _ensure_drivers_registered():
    try:
        import drivers.thunder.han_dog_module  # noqa: F401
    except ImportError:
        pass
    try:
        import core.blueprints.stub  # noqa: F401
    except ImportError:
        pass
    try:
        import drivers.sim.mujoco_driver_module  # noqa: F401
    except ImportError:
        pass
    try:
        import drivers.sim.ros2_sim_driver  # noqa: F401
    except ImportError:
        pass


def full_stack_blueprint(
    robot: str = "thunder",
    slam_profile: str = "fastlio2",
    detector: str = "yoloe",
    encoder: str = "mobileclip",
    llm: str = "kimi",
    planner: str = "astar",
    tomogram: str = "",
    gateway_port: int = 5050,
    enable_native: bool = True,
    enable_semantic: bool = True,
    enable_gateway: bool = True,
    enable_map_modules: bool = True,
    **config: Any,
) -> Blueprint:
    """Build the complete LingTu navigation stack.

    All components are pluggable via arguments. Modules communicate via
    In/Out ports with 3-tier decoupling (callback / transport / DDS).

    Args:
        robot: Driver ("thunder", "sim_mujoco", "stub", "auto")
        slam_profile: "fastlio2" or "pointlio"
        detector: "yoloe", "yolo_world", "bpu", "grounding_dino"
        encoder: "clip", "mobileclip"
        llm: "kimi", "openai", "claude", "qwen", "mock"
        planner: "astar", "pct"
        tomogram: path to tomogram .pickle for global planner
        gateway_port: FastAPI gateway port
        enable_native: include C++ AutonomyModule nodes
        enable_semantic: include perception + semantic planning modules
        enable_gateway: include HTTP gateway and MCP server
        enable_map_modules: include OccupancyGrid + ESDF + ElevationMap modules
        **config: extra config (dog_host, dog_port, etc.)
    """
    cfg = get_config()
    bp = Blueprint()

    # ── Layer 1: Robot driver (pluggable via registry) ───────────────────

    _ensure_drivers_registered()
    if robot == "auto":
        import platform
        arch = platform.machine().lower()
        robot = auto_select("driver", platform=arch) or "stub"
        logger.info("Auto-selected driver: %s", robot)

    DriverCls = get("driver", robot)
    bp.add(DriverCls,
           dog_host=config.get("dog_host", cfg.driver.dog_host),
           dog_port=config.get("dog_port", cfg.driver.dog_port))
    driver_name = DriverCls.__name__

    # ── Layer 1b: SLAM bridge (ROS2 → Python map_cloud + slam_odom) ───────
    # Bridges /nav/map_cloud from external SLAM (C++ ROS2 process) into
    # the Python pipeline.  Graceful no-op when rclpy unavailable.

    try:
        from slam.slam_bridge_module import SlamBridgeModule
        bp.add(SlamBridgeModule)
    except ImportError as e:
        logger.warning("SlamBridgeModule not available, skipping: %s", e)

    # ── Layer 2: C++ autonomy stack (NativeModule, DDS) ──────────────────

    if enable_native:
        from base_autonomy.modules.autonomy_module import add_autonomy_stack
        add_autonomy_stack(bp, backend="cmu", path_follower_backend="nav_core")

    # ── Layer 2: Python map modules (OccupancyGrid + ESDF + ElevationMap) ─
    # Consume map_cloud: Out[PointCloud] from the SLAM source (AutonomyModule
    # or SLAMModule).  auto_wire connects them by port name + type.
    #
    #   map_cloud ──┬── OccupancyGridModule ──► costmap → NavigationModule
    #               │                       └► occupancy_grid → ESDFModule
    #               └── ElevationMapModule  ──► elevation_map (consumers TBD)

    if enable_map_modules:
        try:
            from nav.occupancy_grid_module import OccupancyGridModule
            from nav.esdf_module import ESDFModule
            from nav.elevation_map_module import ElevationMapModule

            bp.add(OccupancyGridModule,
                   resolution=config.get("grid_resolution", 0.2),
                   map_radius=config.get("grid_radius", 30.0),
                   inflation_radius=config.get("inflation_radius", 0.5))
            bp.add(ESDFModule)
            bp.add(ElevationMapModule,
                   resolution=config.get("elev_resolution", 0.2),
                   map_radius=config.get("elev_radius", 15.0))
            logger.info("Map modules added: OccupancyGrid + ESDF + ElevationMap")
        except ImportError as e:
            logger.warning("Map modules not available, skipping: %s", e)

    # ── Layer 3: Perception (pluggable detector + encoder) ───────────────

    if enable_semantic:
        try:
            from semantic.perception.semantic_perception.detector_module import DetectorModule
            from semantic.perception.semantic_perception.encoder_module import EncoderModule

            bp.add(DetectorModule, detector=detector,
                   confidence=config.get("confidence", 0.3))
            bp.add(EncoderModule, encoder=encoder)
        except ImportError:
            logger.warning("Perception modules not available, skipping")

    # ── Layer 3b: Semantic mapper (KG + topological graph) ──────────────
    # Subscribes to scene_graph + odometry → drives RoomObjectKG + TopologySemGraph.
    # Publishes topo_summary (str) → SemanticPlannerModule for exploration context.

    semantic_save_dir = config.get("semantic_save_dir", _DEFAULT_SEMANTIC_DIR)

    if enable_semantic:
        try:
            from memory.modules.semantic_mapper_module import SemanticMapperModule
            bp.add(SemanticMapperModule, save_dir=semantic_save_dir)
            logger.info("SemanticMapperModule added (save_dir=%s)", semantic_save_dir)
        except ImportError as e:
            logger.warning("SemanticMapperModule not available, skipping: %s", e)

    # ── Layer 4: Semantic planning (unified module + LLM) ────────────────

    if enable_semantic:
        try:
            from semantic.planner.semantic_planner.semantic_planner_module import SemanticPlannerModule
            from semantic.planner.semantic_planner.llm_module import LLMModule

            bp.add(SemanticPlannerModule, save_dir=semantic_save_dir)
            bp.add(LLMModule, backend=llm)
        except ImportError:
            logger.warning("Semantic planner modules not available, skipping")

    # ── Layer 4b: Visual servo (bbox tracking + person following) ────────
    # Two channels: goal_pose (far) → NavigationModule, cmd_vel (near) → Driver

    _has_visual_servo = False
    if enable_semantic:
        try:
            from semantic.planner.semantic_planner.visual_servo_module import VisualServoModule
            bp.add(VisualServoModule)
            _has_visual_servo = True
        except ImportError as e:
            logger.warning("VisualServoModule not available, skipping: %s", e)

    # ── Layer 5: Navigation (unified plan + track + FSM) ─────────────────

    from nav.navigation_module import NavigationModule
    bp.add(NavigationModule, planner=planner, tomogram=tomogram,
           enable_ros2_bridge=not enable_native)

    # ── Layer 0: Safety ring (reflex + eval + dialogue) ──────────────────

    from nav.safety_ring_module import SafetyRingModule
    bp.add(SafetyRingModule)

    # ── Layer 6: Gateway (HTTP/WS/SSE + MCP server) ──────────────────────

    if enable_gateway:
        try:
            from gateway.gateway_module import GatewayModule
            bp.add(GatewayModule, port=gateway_port)
        except ImportError:
            logger.warning("GatewayModule not available, skipping")

        try:
            from gateway.mcp_server import MCPServerModule
            bp.add(MCPServerModule, port=8090)
        except ImportError:
            logger.warning("MCPServerModule not available, skipping")

    # ── Wiring: 3-tier decoupling ────────────────────────────────────────

    # Tier 1: Safety — direct callback (zero latency)
    bp.wire("SafetyRingModule", "stop_cmd", driver_name, "stop_signal")
    bp.wire("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal")

    # Tier 1b: Goal routing — sim_ros2 driver bridges ROS2 goal_pose
    if hasattr(DriverCls, 'goal_pose'):
        bp.wire(driver_name, "goal_pose", "NavigationModule", "goal_pose")

    # Tier 1c: Autonomy chain — explicit wires (must not silently break)
    if enable_native:
        bp.wire("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint")
        bp.wire("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map")
        bp.wire("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")
        bp.wire("PathFollowerModule", "cmd_vel", driver_name, "cmd_vel")

    # Tier 1d: Visual servo — dual channel (goal_pose + cmd_vel)
    if _has_visual_servo:
        bp.wire("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target")
        bp.wire("VisualServoModule", "goal_pose", "NavigationModule", "goal_pose")
        bp.wire("VisualServoModule", "nav_stop", "NavigationModule", "stop_signal")
        if enable_native:
            bp.wire("VisualServoModule", "cmd_vel", driver_name, "cmd_vel")

    # Tier 2: Everything else — auto_wire (direct callback)
    # goal_pose, odometry, instruction, etc. matched by (name, type)
    bp.auto_wire()

    return bp
