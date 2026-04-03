import importlib.util
import sys
import types
from pathlib import Path

from drivers.sim.mujoco_driver_module import MujocoDriverModule
from sim.engine.core.robot import RobotConfig


def test_default_nova_dog_resolves_real_robot_model():
    sim_root = Path(__file__).resolve().parents[3] / "sim"
    cfg = RobotConfig.default_nova_dog().resolve_paths(base_dir=str(sim_root))

    assert Path(cfg.robot_xml).name == "robot_with_camera.xml"
    assert Path(cfg.robot_xml).exists()
    assert cfg.policy_onnx == ""


def test_semantic_namespace_wrappers_expose_runtime_import_paths():
    assert importlib.util.find_spec("semantic_common.validation") is not None
    assert importlib.util.find_spec("semantic_perception.instance_tracker") is not None
    assert importlib.util.find_spec("semantic_planner.llm_client") is not None

    from semantic_common import sanitize_position

    assert callable(sanitize_position)


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
    assert driver._engine.reset_called is True
    assert len(driver._engine.camera_configs) == 1
