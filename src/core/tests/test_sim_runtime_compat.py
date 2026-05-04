import importlib.util
import math
import sys
import time
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from sim.engine.core.robot import RobotConfig

from core.blueprints.full_stack import full_stack_blueprint
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from drivers.sim.mujoco_driver_module import MujocoDriverModule


def test_default_nova_dog_resolves_real_robot_model():
    sim_root = Path(__file__).resolve().parents[3] / "sim"
    cfg = RobotConfig.default_nova_dog().resolve_paths(base_dir=str(sim_root))

    assert Path(cfg.robot_xml).name == "thunder_v3_lingtu.xml"
    assert Path(cfg.robot_xml).exists()
    assert cfg.policy_onnx == ""
    assert cfg.base_body_name == "base_link"
    assert cfg.lidar_body_name == "lidar_link"
    assert cfg.leg_act_offset == 0
    assert cfg.leg_joint_names[0] == "FR_hip_joint"


def test_default_nova_dog_resolves_paths_from_engine_core_default():
    sim_root = Path(__file__).resolve().parents[3] / "sim"
    cfg = RobotConfig.default_nova_dog().resolve_paths()

    assert Path(cfg.robot_xml).exists()
    assert Path(cfg.robot_xml).is_relative_to(sim_root)
    assert Path(cfg.robot_xml).name == "thunder_v3_lingtu.xml"
    assert "sim/sim" not in Path(cfg.robot_xml).as_posix()


def test_thunder_v3_urdf_xml_assets_are_current_and_resolvable():
    sim_root = Path(__file__).resolve().parents[3] / "sim"
    urdf_path = sim_root / "assets" / "urdf" / "thunder_v3.urdf"
    xml_path = sim_root / "assets" / "xml" / "thunder_v3.xml"
    compat_urdf_path = sim_root / "robot" / "thunder.urdf"

    assert urdf_path.exists()
    assert xml_path.exists()
    assert compat_urdf_path.exists()
    assert xml_path.read_bytes() == urdf_path.read_bytes()

    root = ET.parse(urdf_path).getroot()
    compat_root = ET.parse(compat_urdf_path).getroot()
    assert root.attrib["name"] == "thunder_v3"
    assert compat_root.attrib["name"] == "thunder_v3"

    masses = [float(m.attrib["value"]) for m in root.findall(".//mass")]
    assert sum(masses) == pytest.approx(48.79163, abs=1e-5)
    assert sum(1 for value in masses if value == pytest.approx(1.40377, abs=1e-5)) == 4

    joints = {j.attrib["name"]: j for j in root.findall("joint")}
    assert "FR_hip_joint" in joints
    assert "fr_hip_joint" not in joints
    assert joints["FR_hip_joint"].find("limit").attrib["effort"] == "120"
    assert joints["FR_hip_joint"].find("limit").attrib["velocity"] == "17.48"
    assert joints["FR_foot_joint"].attrib["type"] == "continuous"
    assert joints["FR_foot_joint"].find("limit") is None

    for mesh in root.findall(".//mesh"):
        mesh_path = (urdf_path.parent / mesh.attrib["filename"]).resolve()
        assert mesh_path.exists(), mesh.attrib["filename"]
    for mesh in compat_root.findall(".//mesh"):
        mesh_path = (compat_urdf_path.parent / mesh.attrib["filename"]).resolve()
        assert mesh_path.exists(), mesh.attrib["filename"]


def test_thunder_v3_mjcf_runtime_keeps_lingtu_sensor_and_control_contracts():
    sim_root = Path(__file__).resolve().parents[3] / "sim"
    upstream_path = sim_root / "assets" / "mjcf" / "thunder_v3_mujoco.xml"
    runtime_path = sim_root / "assets" / "mjcf" / "thunder_v3_lingtu.xml"

    upstream = ET.parse(upstream_path).getroot()
    runtime = ET.parse(runtime_path).getroot()

    assert upstream.attrib["model"] == "thunder_v3_mujoco"
    assert runtime.attrib["model"] == "thunder_v3_mujoco"
    assert runtime.find(".//body[@name='base_link']") is not None
    assert runtime.find(".//body[@name='lidar_link']") is not None
    assert runtime.find(".//camera[@name='front_camera']") is not None

    upstream_motors = upstream.findall("./actuator/motor")
    runtime_positions = runtime.findall("./actuator/position")
    assert len(upstream_motors) == 16
    assert len(runtime_positions) == 16
    assert {a.attrib["joint"] for a in runtime_positions} >= {
        "FR_hip_joint",
        "FL_foot_joint",
        "RR_calf_joint",
        "RL_foot_joint",
    }


def test_semantic_namespace_wrappers_expose_runtime_import_paths():
    assert importlib.util.find_spec("semantic_perception.instance_tracker") is not None
    assert importlib.util.find_spec("semantic_planner.llm_client") is not None

    # Canonical imports from core.utils
    from core.msgs import scene as scene_msgs
    from core.utils.robustness import retry
    from core.utils.sanitize import sanitize_position
    from core.utils.validation import validate_bgr
    from semantic.perception.semantic_perception.instance_tracker import InstanceTracker
    from semantic.perception.semantic_perception.tracked_objects import TrackedObject

    assert callable(sanitize_position)
    assert InstanceTracker is not None
    assert scene_msgs.TrackedObject is TrackedObject


class _FakeEngine:
    def __init__(
        self,
        robot_config,
        world_config,
        lidar_config,
        camera_configs,
        headless,
        drive_mode="policy",
    ):
        self.robot_config = robot_config
        self.world_config = world_config
        self.lidar_config = lidar_config
        self.camera_configs = camera_configs
        self.headless = headless
        self.drive_mode = drive_mode
        self.loaded_xml_path = ""
        self.reset_called = False

    def load(self, xml_path: str = "", **kwargs):
        self.loaded_xml_path = xml_path

    def reset(self):
        self.reset_called = True


def test_mujoco_driver_setup_uses_selected_scene_and_real_robot(monkeypatch):
    import sim.engine.mujoco.engine as mujoco_engine

    monkeypatch.setitem(sys.modules, "mujoco", types.SimpleNamespace(__version__="test"))
    monkeypatch.setattr(mujoco_engine, "MuJoCoEngine", _FakeEngine)

    driver = MujocoDriverModule(
        world="open_field",
        render=False,
        enable_camera=True,
    )
    driver.setup()

    expected_world = Path(__file__).resolve().parents[3] / "sim" / "worlds" / "open_field.xml"

    assert driver._engine is not None
    assert Path(driver._engine.loaded_xml_path) == expected_world
    assert Path(driver._engine.world_config.scene_xml) == expected_world
    assert Path(driver._engine.robot_config.robot_xml).name == "thunder_v3_lingtu.xml"
    assert Path(driver._engine.robot_config.robot_xml).exists()
    assert driver._engine.robot_config.base_body_name == "base_link"
    assert driver._engine.lidar_config.body_name == "lidar_link"
    assert driver._engine.drive_mode == "policy"
    assert driver._engine.reset_called is True
    assert len(driver._engine.camera_configs) == 1


def test_mujoco_driver_uses_scene_placeholder_start_pose(monkeypatch):
    import sim.engine.mujoco.engine as mujoco_engine

    monkeypatch.setitem(sys.modules, "mujoco", types.SimpleNamespace(__version__="test"))
    monkeypatch.setattr(mujoco_engine, "MuJoCoEngine", _FakeEngine)

    driver = MujocoDriverModule(
        world="building_scene",
        render=False,
        enable_camera=False,
    )
    driver.setup()

    assert driver._engine is not None
    assert driver._engine.robot_config.init_position == [2.0, 3.0, 0.5]


def test_mujoco_driver_prefers_thunder_policy_and_resolves_repo_relative_paths(monkeypatch, tmp_path):
    import drivers.sim.mujoco_driver_module as driver_mod

    sim_root = tmp_path / "sim"
    policy_dir = sim_root / "robots" / "nova_dog"
    policy_dir.mkdir(parents=True)
    thunder_policy = policy_dir / "thunder_policy.onnx"
    legacy_policy = policy_dir / "policy.onnx"
    thunder_policy.write_bytes(b"thunder")
    legacy_policy.write_bytes(b"legacy")

    monkeypatch.setattr(driver_mod, "_SIM_ROOT", sim_root)
    monkeypatch.setattr(driver_mod, "_POLICY_CANDIDATES", (thunder_policy, legacy_policy))

    driver = MujocoDriverModule(policy_path="")
    assert Path(driver._policy_path) == thunder_policy

    explicit = MujocoDriverModule(policy_path="sim/robots/nova_dog/thunder_policy.onnx")
    assert Path(explicit._policy_path) == thunder_policy.resolve()

    missing_explicit = driver_mod._resolve_sim_path("sim/robots/nova_dog/missing.onnx")
    assert Path(missing_explicit) == (sim_root / "robots" / "nova_dog" / "missing.onnx").resolve()

    missing_sim_relative = driver_mod._resolve_sim_path("robots/nova_dog/missing.onnx")
    assert Path(missing_sim_relative) == (
        sim_root / "robots" / "nova_dog" / "missing.onnx"
    ).resolve()


def test_mujoco_driver_kinematic_mode_disables_policy(monkeypatch):
    import sim.engine.mujoco.engine as mujoco_engine

    monkeypatch.setitem(sys.modules, "mujoco", types.SimpleNamespace(__version__="test"))
    monkeypatch.setattr(mujoco_engine, "MuJoCoEngine", _FakeEngine)

    driver = MujocoDriverModule(
        world="open_field",
        render=False,
        enable_camera=False,
        drive_mode="kinematic",
    )
    driver.setup()

    assert driver._engine is not None
    assert driver._engine.drive_mode == "kinematic"
    assert driver._engine.robot_config.policy_onnx == ""


def test_mujoco_driver_stop_signal_zero_clears_soft_stop_latch():
    driver = MujocoDriverModule(world="open_field", render=False, enable_camera=False)
    driver.cmd_vel.subscribe(driver._on_cmd_vel)
    driver.stop_signal.subscribe(driver._on_stop)

    driver.stop_signal._deliver(1)
    driver.cmd_vel._deliver(Twist(linear=Vector3(0.4, 0.0, 0.0)))
    assert driver._stopped is True
    assert driver._cmd_vx == 0.0

    driver.stop_signal._deliver(0)
    driver.cmd_vel._deliver(Twist(linear=Vector3(0.4, 0.0, 0.0)))
    assert driver._stopped is False
    assert driver._cmd_vx == pytest.approx(0.4)


def test_mujoco_policy_runner_pads_legacy_obs_for_single_frame_policy():
    from sim.engine.mujoco.robot_controller import OBS_DIM, PolicyRunner

    runner = object.__new__(PolicyRunner)
    runner._history_len = 1
    runner._policy_input_dim = 76

    obs = np.arange(OBS_DIM, dtype=np.float32)
    adapted = runner._adapt_obs_to_policy_input(obs)

    assert adapted.shape == (76,)
    assert np.allclose(adapted[:OBS_DIM], obs)
    assert np.allclose(adapted[OBS_DIM:], 0.0)


def test_mujoco_camera_preserves_metric_depth_output():
    from sim.engine.mujoco.camera import MuJoCoCamera

    raw = np.array([[0.4, 2.5, 25.0]], dtype=np.float32)
    depth = MuJoCoCamera._coerce_depth_meters(raw, near=0.1, far=10.0)

    assert np.allclose(depth, np.array([[0.4, 2.5, 10.0]], dtype=np.float32))



def test_mujoco_driver_default_robot_emits_lidar_points():
    pytest.importorskip("mujoco")

    driver = MujocoDriverModule(world="building_scene", render=False, enable_camera=False)
    driver.setup()
    try:
        assert driver._engine is not None
        pts = driver._engine.get_lidar_points()
        assert pts is not None
        assert len(pts) > 0
    finally:
        if driver._engine is not None:
            driver._engine.close()
            driver._engine = None


def test_mujoco_driver_kinematic_cmd_vel_moves_free_base():
    pytest.importorskip("mujoco")
    from sim.engine.core.engine import VelocityCommand

    driver = MujocoDriverModule(
        world="open_field",
        render=False,
        enable_camera=False,
        drive_mode="kinematic",
    )
    driver.setup()
    try:
        assert driver._engine is not None
        assert driver._engine.drive_mode == "kinematic"
        assert driver._engine.has_policy is False

        start = driver._engine.get_robot_state().position.copy()
        for _ in range(100):
            state = driver._engine.step(VelocityCommand(linear_x=0.5))

        moved = math.hypot(state.position[0] - start[0], state.position[1] - start[1])
        assert moved > 0.75
    finally:
        if driver._engine is not None:
            driver._engine.close()
            driver._engine = None


def test_sim_mujoco_full_stack_emits_costmap_and_plans_local_goal():
    pytest.importorskip("mujoco")

    system = full_stack_blueprint(
        robot="sim_mujoco",
        world="open_field",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        render=False,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        drive_mode="kinematic",
        waypoint_threshold=0.35,
        downsample_dist=0.5,
        run_startup_checks=False,
    ).build()
    driver = system.get_module("MujocoDriverModule")
    ogm = system.get_module("OccupancyGridModule")
    nav = system.get_module("NavigationModule")
    local_planner = system.get_module("LocalPlannerModule")
    path_follower = system.get_module("PathFollowerModule")
    mux = system.get_module("CmdVelMux")

    seen = {
        "costmap": 0,
        "waypoints": 0,
        "local_path": 0,
        "path_follower_cmd": 0,
        "mux_cmd": 0,
        "direct_fallback": 0,
    }
    odom = []
    ogm.costmap._add_callback(lambda _: seen.__setitem__("costmap", seen["costmap"] + 1))
    nav.waypoint._add_callback(lambda _: seen.__setitem__("waypoints", seen["waypoints"] + 1))
    nav.adapter_status._add_callback(
        lambda e: seen.__setitem__(
            "direct_fallback",
            seen["direct_fallback"] + (1 if e.get("event") == "direct_goal_fallback" else 0),
        )
    )
    local_planner.local_path._add_callback(
        lambda _: seen.__setitem__("local_path", seen["local_path"] + 1)
    )
    path_follower.cmd_vel._add_callback(
        lambda _: seen.__setitem__("path_follower_cmd", seen["path_follower_cmd"] + 1)
    )
    mux.driver_cmd_vel._add_callback(lambda _: seen.__setitem__("mux_cmd", seen["mux_cmd"] + 1))
    driver.odometry._add_callback(
        lambda m: odom.append((float(m.pose.position.x), float(m.pose.position.y), float(m.pose.position.z)))
    )

    system.start()
    try:
        deadline = time.time() + 6.0
        while time.time() < deadline and (seen["costmap"] == 0 or not odom):
            time.sleep(0.1)
        assert driver._engine is not None
        assert seen["costmap"] > 0
        assert nav._planner_svc.has_map is True

        start = odom[-1]
        x, y, _ = start
        goal_x = x + 2.5
        goal_y = y
        nav.goal_pose._deliver(
            PoseStamped(
                pose=Pose(
                    position=Vector3(goal_x, goal_y, 0.0),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
                frame_id="map",
                ts=time.time(),
            )
        )

        plan_deadline = time.time() + 10.0
        moved = 0.0
        dist_to_goal = math.hypot(goal_x - start[0], goal_y - start[1])
        while time.time() < plan_deadline:
            time.sleep(0.1)
            if odom:
                moved = math.hypot(odom[-1][0] - start[0], odom[-1][1] - start[1])
                dist_to_goal = math.hypot(goal_x - odom[-1][0], goal_y - odom[-1][1])
            if (
                seen["waypoints"] > 0
                and seen["local_path"] > 0
                and seen["path_follower_cmd"] > 0
                and seen["mux_cmd"] > 0
                and moved > 0.75
                and dist_to_goal < 1.75
            ):
                break

        assert seen["waypoints"] > 0
        assert seen["local_path"] > 0
        assert seen["path_follower_cmd"] > 0
        assert seen["mux_cmd"] > 0
        assert seen["direct_fallback"] == 0
        assert moved > 0.75
        assert dist_to_goal < 1.75
        assert nav._state in ("EXECUTING", "SUCCESS")
    finally:
        system.stop()


def test_sim_mujoco_full_stack_routes_autonomy_cmds_through_mux():
    pytest.importorskip("mujoco")

    system = full_stack_blueprint(
        robot="sim_mujoco",
        world="open_field",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        render=False,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        run_startup_checks=False,
    ).build()

    nav = system.get_module("NavigationModule")
    assert system.get_module("CmdVelMux") is not None
    recovery_edge = (
        "NavigationModule",
        "recovery_cmd_vel",
        "CmdVelMux",
        "recovery_cmd_vel",
    )
    if hasattr(nav, "recovery_cmd_vel"):
        assert recovery_edge in system.connections
    else:
        assert recovery_edge not in system.connections
    assert (
        "PathFollowerModule",
        "cmd_vel",
        "CmdVelMux",
        "path_follower_cmd_vel",
    ) in system.connections
    assert (
        "CmdVelMux",
        "driver_cmd_vel",
        "MujocoDriverModule",
        "cmd_vel",
    ) in system.connections
    assert (
        "CmdVelMux",
        "driver_cmd_vel",
        "SafetyRingModule",
        "cmd_vel",
    ) in system.connections
    assert (
        "PathFollowerModule",
        "cmd_vel",
        "MujocoDriverModule",
        "cmd_vel",
    ) not in system.connections


def test_full_stack_mux_wiring_tolerates_legacy_nav_without_recovery_cmd():
    from core.blueprint import Blueprint
    from core.blueprints.full_stack_wiring import apply_full_stack_wires
    from core.module import Module
    from core.stream import In, Out
    from nav.cmd_vel_mux_module import CmdVelMux

    class LegacyNavigationModule(Module, layer=5):
        stop_signal: In[int]

    class TestDriverModule(Module, layer=1):
        cmd_vel: In[Twist]
        stop_signal: In[int]

    class TestPathFollowerModule(Module, layer=5):
        cmd_vel: Out[Twist]

    class TestSafetyRingModule(Module, layer=0):
        stop_cmd: Out[int]
        cmd_vel: In[Twist]

    bp = Blueprint()
    bp.add(LegacyNavigationModule, alias="NavigationModule")
    bp.add(TestDriverModule, alias="MujocoDriverModule")
    bp.add(TestPathFollowerModule, alias="PathFollowerModule")
    bp.add(TestSafetyRingModule, alias="SafetyRingModule")
    bp.add(CmdVelMux)

    system = apply_full_stack_wires(
        bp,
        robot="sim_mujoco",
        driver_module="MujocoDriverModule",
        slam_profile="none",
        enable_semantic=False,
    ).build()

    assert (
        "NavigationModule",
        "recovery_cmd_vel",
        "CmdVelMux",
        "recovery_cmd_vel",
    ) not in system.connections
    assert (
        "PathFollowerModule",
        "cmd_vel",
        "CmdVelMux",
        "path_follower_cmd_vel",
    ) in system.connections
    assert (
        "CmdVelMux",
        "driver_cmd_vel",
        "MujocoDriverModule",
        "cmd_vel",
    ) in system.connections
    assert (
        "PathFollowerModule",
        "cmd_vel",
        "MujocoDriverModule",
        "cmd_vel",
    ) not in system.connections


def test_full_stack_wires_frontier_exploration_goal_to_navigation():
    system = full_stack_blueprint(
        robot="stub",
        slam_profile="none",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        enable_frontier=True,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        run_startup_checks=False,
    ).build()

    assert (
        "WavefrontFrontierExplorer",
        "exploration_goal",
        "NavigationModule",
        "goal_pose",
    ) in system.connections


def test_frontier_exploration_goal_reaches_navigation_planner():
    system = full_stack_blueprint(
        robot="stub",
        slam_profile="none",
        planner_backend="astar",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        enable_map_modules=False,
        enable_frontier=True,
        python_autonomy_backend="simple",
        python_path_follower_backend="pid",
        frontier_min_size=1,
        frontier_safe_distance=0.0,
        frontier_goal_timeout=0.5,
        frontier_rate=10.0,
        run_startup_checks=False,
    ).build()
    explorer = system.get_module("WavefrontFrontierExplorer")
    nav = system.get_module("NavigationModule")

    seen = {"exploration_goals": 0, "waypoints": 0, "paths": 0}
    explorer.exploration_goal._add_callback(
        lambda _: seen.__setitem__("exploration_goals", seen["exploration_goals"] + 1)
    )
    nav.waypoint._add_callback(lambda _: seen.__setitem__("waypoints", seen["waypoints"] + 1))
    nav.global_path._add_callback(lambda _: seen.__setitem__("paths", seen["paths"] + 1))

    grid = np.full((50, 50), -1, dtype=np.int16)
    grid[18:33, 18:33] = 0
    costmap = {
        "grid": grid,
        "resolution": 0.2,
        "origin": np.array([-5.0, -5.0], dtype=np.float32),
        "origin_x": -5.0,
        "origin_y": -5.0,
        "width": 50,
        "height": 50,
    }
    odom = Odometry(pose=Pose(0.0, 0.0, 0.0))

    system.start()
    try:
        explorer.odometry._deliver(odom)
        nav.odometry._deliver(odom)
        explorer.costmap._deliver(costmap)
        nav.costmap._deliver(costmap)

        assert explorer.begin_exploration() == "started"
        deadline = time.time() + 3.0
        while (
            time.time() < deadline
            and (seen["exploration_goals"] == 0 or seen["waypoints"] == 0)
        ):
            time.sleep(0.05)

        assert seen["exploration_goals"] > 0
        assert seen["paths"] > 0
        assert seen["waypoints"] > 0
        assert nav._state in ("EXECUTING", "SUCCESS")
    finally:
        explorer.end_exploration()
        system.stop()


def test_sim_scene_observer_emits_building_scene_stairs():
    from semantic.perception.semantic_perception.sim_scene_observer import SimSceneObserver

    class _Intrinsics:
        fx = 415.7
        fy = 415.7
        cx = 320.0
        cy = 240.0
        width = 640
        height = 480

    tf = np.eye(4, dtype=np.float32)
    tf[:3, :3] = np.array([
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)
    tf[:3, 3] = [2.0, 3.0, 0.5]
    observer = SimSceneObserver(world="building_scene")

    detections = observer.observe(tf, _Intrinsics(), text_prompt="stairs . goal")

    assert any(det.label == "stairs" for det in detections)


def test_sim_scene_observer_respects_live_forward_axis_convention():
    from semantic.perception.semantic_perception.sim_scene_observer import SimSceneObserver

    class _Intrinsics:
        fx = 415.7
        fy = 415.7
        cx = 320.0
        cy = 240.0
        width = 640
        height = 480

    tf = np.eye(4, dtype=np.float32)
    tf[:3, :3] = np.eye(3, dtype=np.float32)
    tf[:3, 3] = [2.0, 3.0, 0.5]
    observer = SimSceneObserver(world="building_scene")

    detections = observer.observe(tf, _Intrinsics(), text_prompt="stairs . goal")

    assert any(det.label == "stairs" for det in detections)
