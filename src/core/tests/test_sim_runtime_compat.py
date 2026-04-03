import importlib.util
import sys

import numpy as np
import pytest
import time
import types
from pathlib import Path

from drivers.sim.mujoco_driver_module import MujocoDriverModule
from core.blueprints.full_stack import full_stack_blueprint
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from sim.engine.core.robot import RobotConfig


def test_default_nova_dog_resolves_real_robot_model():
    sim_root = Path(__file__).resolve().parents[3] / "sim"
    cfg = RobotConfig.default_nova_dog().resolve_paths(base_dir=str(sim_root))

    assert Path(cfg.robot_xml).name == "robot_with_camera.xml"
    assert Path(cfg.robot_xml).exists()
    assert cfg.policy_onnx == ""
    assert cfg.base_body_name == "trunk"
    assert cfg.lidar_body_name == "trunk"


def test_semantic_namespace_wrappers_expose_runtime_import_paths():
    assert importlib.util.find_spec("semantic_common.validation") is not None
    assert importlib.util.find_spec("semantic_perception.instance_tracker") is not None
    assert importlib.util.find_spec("semantic_planner.llm_client") is not None

    from core.msgs import scene as scene_msgs
    from semantic.common.semantic_common import sanitize_position
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
    ):
        self.robot_config = robot_config
        self.world_config = world_config
        self.lidar_config = lidar_config
        self.camera_configs = camera_configs
        self.headless = headless
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
    assert Path(driver._engine.robot_config.robot_xml).name == "robot_with_camera.xml"
    assert Path(driver._engine.robot_config.robot_xml).exists()
    assert driver._engine.robot_config.base_body_name == "trunk"
    assert driver._engine.lidar_config.body_name == "trunk"
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


def test_sim_mujoco_full_stack_emits_costmap_and_plans_local_goal():
    pytest.importorskip("mujoco")

    system = full_stack_blueprint(
        robot="sim_mujoco",
        world="building_scene",
        slam_profile="none",
        detector="sim_scene",
        llm="mock",
        enable_native=False,
        enable_semantic=False,
        enable_gateway=False,
        render=False,
    ).build()
    driver = system.get_module("MujocoDriverModule")
    ogm = system.get_module("OccupancyGridModule")
    nav = system.get_module("NavigationModule")

    seen = {"costmap": 0, "waypoints": 0, "direct_fallback": 0}
    odom = []
    ogm.costmap._add_callback(lambda _: seen.__setitem__("costmap", seen["costmap"] + 1))
    nav.waypoint._add_callback(lambda _: seen.__setitem__("waypoints", seen["waypoints"] + 1))
    nav.adapter_status._add_callback(
        lambda e: seen.__setitem__(
            "direct_fallback",
            seen["direct_fallback"] + (1 if e.get("event") == "direct_goal_fallback" else 0),
        )
    )
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

        x, y, _ = odom[-1]
        nav.goal_pose._deliver(
            PoseStamped(
                pose=Pose(
                    position=Vector3(x + 2.0, y + 0.5, 0.0),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
                frame_id="map",
                ts=time.time(),
            )
        )

        plan_deadline = time.time() + 4.0
        while time.time() < plan_deadline and seen["waypoints"] == 0:
            time.sleep(0.1)

        assert seen["waypoints"] > 0
        assert seen["direct_fallback"] == 0
        assert nav._state == "EXECUTING"
    finally:
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
    tf[:3, :3] = np.array([
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)
    tf[:3, 3] = [2.0, 3.0, 0.5]
    observer = SimSceneObserver(world="building_scene")

    detections = observer.observe(tf, _Intrinsics(), text_prompt="stairs . goal")

    assert any(det.label == "stairs" for det in detections)
