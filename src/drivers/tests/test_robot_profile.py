"""Tests for drivers.robot_profile — RobotProfile, registry, and YAML loading."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from drivers.robot_profile import (
    STUB_PROFILE,
    THUNDER_V3_PROFILE,
    ConnectionConfig,
    ControlLimits,
    MountSpec,
    PowerSpec,
    RobotKinematics,
    RobotProfile,
    SafetyLimits,
    get_profile,
    list_profiles,
    load_profiles_from_yaml,
    register_profile,
)

# ============================================================================
# THUNDER_V3_PROFILE
# ============================================================================

class TestThunderV3Profile:
    def test_identity(self):
        assert THUNDER_V3_PROFILE.id == "thunder_v3"
        assert THUNDER_V3_PROFILE.brand == "Nova"
        assert THUNDER_V3_PROFILE.model == "Dog V3"

    def test_kinematics_defaults(self):
        k = THUNDER_V3_PROFILE.kinematics
        assert k.locomotion_type == "quadruped"
        assert k.body_length == 1.0
        assert k.body_width == 0.6
        assert k.body_height == 0.5
        assert k.mass_kg == 55.0
        assert k.max_slope_deg == 30.0
        assert k.ground_clearance_m == 0.15

    def test_sensor_count(self):
        assert len(THUNDER_V3_PROFILE.sensors) == 3

    def test_sensor_lidar_extrinsics(self):
        lidar = THUNDER_V3_PROFILE.sensors[0]
        assert lidar.id == "lidar_front"
        assert lidar.type == "lidar"
        assert lidar.model == "Livox MID-360"
        assert lidar.x == -0.011
        assert lidar.y == -0.02329
        assert lidar.z == 0.04412

    def test_sensor_camera_intrinsics(self):
        cam = THUNDER_V3_PROFILE.sensors[1]
        assert cam.id == "camera_front"
        assert cam.model == "Depth Camera"
        assert cam.intrinsics["fx"] == 615.0
        assert cam.intrinsics["fy"] == 615.0
        assert cam.intrinsics["width"] == 640
        assert cam.intrinsics["height"] == 480

    def test_control_limits_defaults(self):
        c = THUNDER_V3_PROFILE.control_limits
        assert c.max_linear_speed == 1.0
        assert c.max_angular_speed == 1.0


# ============================================================================
# STUB_PROFILE
# ============================================================================

class TestStubProfile:
    def test_all_zeros(self):
        assert STUB_PROFILE.kinematics.locomotion_type == "none"
        assert STUB_PROFILE.kinematics.body_length == 0.0
        assert STUB_PROFILE.kinematics.mass_kg == 0.0
        assert STUB_PROFILE.control_limits.max_linear_speed == 0.0
        assert STUB_PROFILE.safety_limits.stop_distance == 0.0
        assert STUB_PROFILE.connection.protocol == "none"
        assert STUB_PROFILE.sensors == []


# ============================================================================
# from_config
# ============================================================================

class TestFromConfig:
    def test_minimal_config(self):
        """Minimal config returns profile with all defaults."""
        profile = RobotProfile.from_config({})
        assert profile.id == "robot_001"
        assert profile.kinematics.body_length == 1.0
        assert len(profile.sensors) == 2  # lidar + camera

    def test_full_config(self):
        """Config values propagate to the correct sub-dataclass fields."""
        config = {
            "robot": {"id": "my_bot", "hw_id": "dog_v4", "firmware_version": "2.0"},
            "geometry": {"vehicle_length": 1.2, "vehicle_width": 0.7, "mass_kg": 60.0},
            "speed": {"max_linear": 2.0, "max_angular": 1.5},
            "safety": {"stop_distance": 0.5, "tilt_limit_deg": 25.0},
            "driver": {"dog_host": "10.0.0.1", "dog_port": 13146, "protocol": "ros2"},
            "control": {"max_accel": 2.0},
            "lidar": {"offset_x": 0.1, "publish_freq": 20.0},
            "camera": {"fx": 800.0, "fy": 800.0, "width": 1280, "height": 720},
            "gnss": {"enabled": True, "model": "ZED-F9P"},
        }
        profile = RobotProfile.from_config(config)
        assert profile.id == "my_bot"
        assert profile.model == "dog_v4"
        assert profile.kinematics.body_length == 1.2
        assert profile.kinematics.mass_kg == 60.0
        assert profile.control_limits.max_linear_speed == 2.0
        assert profile.control_limits.max_angular_speed == 1.5
        assert profile.safety_limits.stop_distance == 0.5
        assert profile.safety_limits.tilt_limit_deg == 25.0
        assert profile.connection.host == "10.0.0.1"
        assert profile.connection.port == 13146
        assert profile.connection.protocol == "ros2"
        # 2 default sensors + 1 gnss = 3
        assert len(profile.sensors) == 3
        # Lidar position override
        lidar = profile.sensors[0]
        assert lidar.x == 0.1
        assert lidar.intrinsics["publish_freq"] == 20.0
        # Gnss added
        gnss = profile.sensors[2]
        assert gnss.type == "gnss"
        assert gnss.model == "ZED-F9P"


# ============================================================================
# merge_with
# ============================================================================

class TestMergeWith:
    def test_merge_overrides_non_empty(self):
        base = RobotProfile(
            id="base", brand="Nova", model="v1",
            kinematics=RobotKinematics(body_length=1.0),
            sensors=[MountSpec(id="lidar_front", type="lidar", frame_id="livox_frame")],
        )
        override = RobotProfile(
            id="override", brand="Nova", model="v2",
            kinematics=RobotKinematics(body_length=1.5),
        )
        merged = base.merge_with(override)
        assert merged.id == "override"
        assert merged.model == "v2"
        assert merged.kinematics.body_length == 1.5
        # Empty sensors in override keeps base sensors
        assert len(merged.sensors) == 1
        assert merged.sensors[0].id == "lidar_front"

    def test_merge_keeps_base_when_override_empty(self):
        base = RobotProfile(id="base", brand="Nova", description="base bot")
        override = RobotProfile(id="", brand="", description="")
        merged = base.merge_with(override)
        assert merged.id == "base"
        assert merged.brand == "Nova"
        assert merged.description == "base bot"

    def test_merge_replaces_sensors_when_non_empty(self):
        base = RobotProfile(
            id="base", sensors=[MountSpec(id="old", type="lidar", frame_id="f")],
        )
        override = RobotProfile(
            id="override", sensors=[MountSpec(id="new", type="camera", frame_id="f")],
        )
        merged = base.merge_with(override)
        assert len(merged.sensors) == 1
        assert merged.sensors[0].id == "new"


# ============================================================================
# Registry roundtrip
# ============================================================================

class TestRegistry:
    def setup_method(self):
        # Save and clear the internal registry
        from drivers.robot_profile import _PROFILES
        self._saved = dict(_PROFILES)
        _PROFILES.clear()

    def teardown_method(self):
        from drivers.robot_profile import _PROFILES
        _PROFILES.clear()
        _PROFILES.update(self._saved)

    def test_register_and_get(self):
        p = RobotProfile(id="test_bot")
        register_profile(p)
        assert get_profile("test_bot") is p

    def test_get_unknown_raises(self):
        with pytest.raises(KeyError, match="Unknown robot profile"):
            get_profile("nonexistent")

    def test_register_overwrites(self):
        p1 = RobotProfile(id="dup", brand="first")
        p2 = RobotProfile(id="dup", brand="second")
        register_profile(p1)
        register_profile(p2)
        assert get_profile("dup").brand == "second"

    def test_list_profiles_roundtrip(self):
        p = RobotProfile(id="alpha")
        register_profile(p)
        names = list_profiles()
        assert "alpha" in names


# ============================================================================
# load_profiles_from_yaml
# ============================================================================

class TestLoadFromYaml:
    def test_load_from_yaml(self, tmp_path: Path):
        config = {
            "robot": {"id": "yaml_bot", "hw_id": "dog_v5"},
            "geometry": {"vehicle_length": 1.3},
            "speed": {"max_linear": 1.5},
        }
        yaml_path = tmp_path / "robot_config.yaml"
        with open(yaml_path, "w") as f:
            yaml.dump(config, f)

        n = load_profiles_from_yaml(str(yaml_path))
        assert n == 1
        profile = get_profile("yaml_bot")
        assert profile.model == "dog_v5"
        assert profile.kinematics.body_length == 1.3
        assert profile.control_limits.max_linear_speed == 1.5

    def test_load_from_yaml_invalid_top_level(self, tmp_path: Path):
        yaml_path = tmp_path / "bad.yaml"
        with open(yaml_path, "w") as f:
            f.write("not_a_dict\n")

        with pytest.raises(ValueError, match="Expected a dict"):
            load_profiles_from_yaml(str(yaml_path))

    def test_load_from_yaml_file_not_found(self):
        with pytest.raises(FileNotFoundError):
            load_profiles_from_yaml("/nonexistent/path.yaml")
