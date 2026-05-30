from __future__ import annotations

import pytest

from core.module import Module
from core.registry import register
from semantic.perception.semantic_perception.perception_module import PerceptionModule
from semantic.planner.semantic_planner.llm_module import LLMModule


class _RuntimeSwitchDetector:
    def load_model(self) -> None:
        pass

    def detect(self, _image, _prompt):
        return []


class _RuntimeSwitchEncoder:
    def load_model(self) -> None:
        pass

    def encode_text(self, _text):
        return []


@register("perception_detector", "runtime_switch_test_detector")
class _RuntimeSwitchDetectorProvider:
    @staticmethod
    def create(_module):
        return _RuntimeSwitchDetector()


@register("perception_encoder", "runtime_switch_test_encoder")
class _RuntimeSwitchEncoderProvider:
    @staticmethod
    def create(_module):
        return _RuntimeSwitchEncoder()


def test_motion_backend_reconfigure_is_unsupported_by_default():
    mod = Module()

    result = mod.reconfigure_backend("local_planner", "nav_core")

    assert result == {
        "ok": False,
        "category": "local_planner",
        "requested_backend": "nav_core",
        "reason": "backend_reconfigure_unsupported",
    }


@pytest.mark.parametrize(
    ("module_path", "class_name", "category", "backend"),
    [
        ("nav.navigation_module", "NavigationModule", "planner", "astar"),
        ("base_autonomy.modules.local_planner_module", "LocalPlannerModule", "local_planner", "cmu_py"),
        ("base_autonomy.modules.path_follower_module", "PathFollowerModule", "path_follower", "pid"),
        ("base_autonomy.modules.terrain_module", "TerrainModule", "terrain", "simple"),
        ("slam.slam_bridge_module", "SlamBridgeModule", "slam", "bridge"),
    ],
)
def test_motion_modules_reconfigure_backend_fails_closed(module_path, class_name, category, backend):
    module = __import__(module_path, fromlist=[class_name])
    cls = getattr(module, class_name)
    mod = cls()

    result = mod.reconfigure_backend(category, backend)

    assert result == {
        "ok": False,
        "category": category,
        "requested_backend": backend,
        "reason": "backend_reconfigure_unsupported",
    }


def test_perception_reconfigure_detector_rejects_unknown_backend():
    mod = PerceptionModule(detector_type="sim_scene", encoder_type="clip")

    result = mod.reconfigure_backend("detector", "missing_backend")

    assert result["ok"] is False
    assert result["category"] == "detector"
    assert result["requested_backend"] == "missing_backend"
    assert result["reason"] == "unknown_backend"


def test_perception_reconfigure_detector_updates_health_status():
    mod = PerceptionModule(detector_type="sim_scene", encoder_type="clip")
    mod._detector_type = "sim_scene"
    mod._detector = object()

    result = mod.reconfigure_backend("detector", "runtime_switch_test_detector")

    assert result["ok"] is True
    assert result["previous_backend"] == "sim_scene"
    assert result["backend"] == "runtime_switch_test_detector"
    health = mod.health()
    assert health["detector"]["configured_backend"] == "runtime_switch_test_detector"
    assert health["detector"]["backend"] == "runtime_switch_test_detector"
    assert health["detector"]["degraded"] is False


def test_perception_reconfigure_encoder_updates_health_status():
    mod = PerceptionModule(detector_type="sim_scene", encoder_type="clip")
    mod._encoder_type = "clip"
    mod._clip_encoder = object()

    result = mod.reconfigure_backend("encoder", "runtime_switch_test_encoder")

    assert result["ok"] is True
    assert result["previous_backend"] == "clip"
    assert result["backend"] == "runtime_switch_test_encoder"
    health = mod.health()
    assert health["encoder"]["configured_backend"] == "runtime_switch_test_encoder"
    assert health["encoder"]["backend"] == "runtime_switch_test_encoder"
    assert health["encoder"]["degraded"] is False


def test_llm_reconfigure_backend_rejects_unknown_backend():
    mod = LLMModule(backend="mock")

    result = mod.reconfigure_backend("llm", "missing_backend")

    assert result["ok"] is False
    assert result["category"] == "llm"
    assert result["requested_backend"] == "missing_backend"
    assert result["reason"] == "unknown_backend"


def test_llm_reconfigure_backend_rejects_in_flight_request():
    mod = LLMModule(backend="mock")
    mod._in_flight = 1

    result = mod.reconfigure_backend("llm", "mock")

    assert result["ok"] is False
    assert result["reason"] == "backend_reconfigure_in_flight"


def test_llm_reconfigure_backend_updates_health_status():
    mod = LLMModule(backend="mock")

    result = mod.reconfigure_backend("llm", "mock")

    assert result["ok"] is True
    assert result["previous_backend"] == "mock"
    assert result["backend"] == "mock"
    health = mod.health()
    assert health["llm"]["configured_backend"] == "mock"
    assert health["llm"]["backend"] == "mock"
    assert health["llm"]["degraded"] is False
