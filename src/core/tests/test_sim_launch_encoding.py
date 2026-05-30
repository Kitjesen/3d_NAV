import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
SIM_NAV_LAUNCH = REPO_ROOT / "tests" / "planning" / "sim_navigation.launch.py"
LEGACY_MUJOCO_LAUNCH = REPO_ROOT / "sim" / "launch" / "sim.launch.py"
PRODUCT_MUJOCO_WORLD = REPO_ROOT / "sim" / "worlds" / "industrial_park_scene.xml"


def test_sim_navigation_launch_text_is_ascii_to_avoid_mojibake() -> None:
    text = SIM_NAV_LAUNCH.read_text(encoding="utf-8")

    assert text.isascii()


def test_legacy_mujoco_launch_text_is_ascii_to_avoid_mojibake() -> None:
    text = LEGACY_MUJOCO_LAUNCH.read_text(encoding="utf-8")

    assert text.isascii()


def test_product_mujoco_world_text_is_ascii_to_avoid_mojibake() -> None:
    text = PRODUCT_MUJOCO_WORLD.read_text(encoding="utf-8")

    assert text.isascii()


def test_sim_navigation_launch_keeps_path_follower_enabled() -> None:
    text = SIM_NAV_LAUNCH.read_text(encoding="utf-8")

    assert re.search(r"^\s*local_planner_node,", text, re.MULTILINE)
    assert re.search(r"^\s*path_follower_node,", text, re.MULTILINE)
