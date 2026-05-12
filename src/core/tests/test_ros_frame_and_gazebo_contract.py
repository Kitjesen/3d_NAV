from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET
import math

import pytest

from sim.engine.bridge.gazebo_bridge import GazeboBridgeConfig
from sim.engine.bridge.gazebo_runtime_adapter import _transform_xyz


REPO_ROOT = Path(__file__).resolve().parents[3]


def _read(path: str) -> str:
    return (REPO_ROOT / path).read_text(encoding="utf-8", errors="ignore")


def test_ros_frame_contract_documents_body_base_link_alias():
    doc = _read("docs/architecture/ros_frame_contract.md")

    assert "map -> odom -> body" in doc
    assert "base_link == body" in doc
    assert "`world`" in doc
    assert "/nav/map_cloud" in doc
    assert "never body-relative points" in doc


def test_topic_contract_keeps_canonical_tf_frame_names():
    contract = _read("config/topic_contract.yaml")

    assert "map_frame: map" in contract
    assert "odom_frame: odom" in contract
    assert "body_frame: body" in contract
    assert "map_to_odom:" in contract
    assert "parent: map" in contract
    assert "child: odom" in contract
    assert "odom_to_body:" in contract
    assert "parent: odom" in contract
    assert "child: body" in contract


def test_sim_ros2_bridge_publishes_canonical_tf_and_cloud_frames():
    bridge = _read("sim/engine/bridge/ros2_bridge.py")

    assert 'tf.header.frame_id = "map"' in bridge
    assert 'tf.child_frame_id = "odom"' in bridge
    assert 'odom.header.frame_id = "odom"' in bridge
    assert 'odom.child_frame_id = "body"' in bridge
    assert 'msg.header.frame_id = "odom"' in bridge
    assert 'msg2.header.frame_id = "body"' in bridge
    assert 'img_msg.header.frame_id = "camera_link"' in bridge


def test_gazebo_bridge_config_exposes_lingtu_runtime_topics():
    cfg = GazeboBridgeConfig(world_name="test_world", robot_name="thunder")

    assert cfg.frames.body_alias_note == "base_link == body"
    assert cfg.required_lingtu_topics() == {
        "cmd_vel": "/nav/cmd_vel",
        "gazebo_cmd_vel_ros_input": "/lingtu/gazebo/cmd_vel",
        "odometry": "/nav/odometry",
        "map_cloud": "/nav/map_cloud",
        "registered_cloud": "/nav/registered_cloud",
        "color_image": "/camera/color/image_raw",
        "depth_image": "/camera/depth/image_raw",
        "camera_info": "/camera/color/camera_info",
    }
    assert cfg.raw_ros_topics() == {
        "odometry": "/lingtu/gazebo/raw/odometry",
        "lidar_points": "/lingtu/gazebo/raw/lidar_points",
        "lidar_scan": "/lingtu/gazebo/raw/lidar_scan",
        "color_image": "/lingtu/gazebo/raw/color_image",
        "depth_image": "/lingtu/gazebo/raw/depth_image",
        "camera_info": "/lingtu/gazebo/raw/camera_info",
    }
    assert cfg.gazebo_odometry == "/model/thunder/odometry"
    assert "test_world" in cfg.gazebo_lidar_points
    assert "test_world" in cfg.gazebo_lidar_scan
    assert "lidar_link" in cfg.gazebo_lidar_points
    assert "lidar_link" in cfg.gazebo_lidar_scan
    assert any("/lingtu/gazebo/cmd_vel" in spec for spec in cfg.ros_gz_bridge_specs())
    assert any("LaserScan" in spec for spec in cfg.ros_gz_bridge_specs())
    assert any("CameraInfo" in spec for spec in cfg.ros_gz_bridge_specs())
    remaps = " ".join(cfg.ros_remap_args())
    assert "/lingtu/gazebo/raw/lidar_points" in remaps
    assert "/lingtu/gazebo/raw/camera_info" in remaps


def test_gazebo_launch_is_optional_but_ros_native():
    launch = _read("launch/gazebo_simulation.launch.py")

    assert "ros_gz_sim" in launch
    assert "ros_gz_bridge" in launch
    assert "parameter_bridge" in launch
    assert "robot_model" in launch
    assert "spawn_robot" in launch
    assert "thunder_gazebo_proxy.sdf" in launch
    assert '"ros_gz_sim"' in launch
    assert '"create"' in launch
    assert "lingtu_gazebo_spawn_robot" in launch
    assert "sys.path.insert" in launch
    assert 'os.path.join(repo_root, "src")' in launch
    assert "headless" in launch
    assert "-r -s" in launch
    assert "sim.engine.bridge.gazebo_cmd_vel_adapter" in launch
    assert "sim.engine.bridge.gazebo_runtime_adapter" in launch
    assert "additional_env=adapter_env" in launch
    assert "cwd=repo_root" in launch
    assert "world -> map -> odom -> body" in launch
    assert "base_link aliased to body" in launch


def test_gazebo_default_world_is_repo_owned_sdf():
    world = _read("sim/worlds/lingtu_gazebo_empty.sdf")

    assert '<world name="lingtu_gazebo_empty">' in world
    assert 'filename="gz-sim-physics-system"' in world
    assert 'filename="gz-sim-user-commands-system"' in world
    assert 'filename="gz-sim-scene-broadcaster-system"' in world
    assert 'filename="gz-sim-sensors-system"' in world
    assert "<gravity>0 0 -9.81</gravity>" in world
    assert '<model name="ground_plane">' in world
    assert '<light name="sun" type="directional">' in world


def test_gazebo_proxy_model_matches_bridge_frame_and_topic_contract():
    model_path = REPO_ROOT / "sim/assets/sdf/thunder_gazebo_proxy.sdf"
    tree = ET.parse(model_path)
    root = tree.getroot()
    text = model_path.read_text(encoding="utf-8")
    link_names = {
        elem.attrib.get("name")
        for elem in root.findall(".//link")
    }

    assert {"base_link", "lidar_link", "camera_link"} <= link_names
    assert 'name="gz::sim::systems::DiffDrive"' in text
    assert 'type="gpu_lidar"' in text
    assert "<topic>/lingtu/gazebo/cmd_vel</topic>" in text
    assert "<odom_topic>/model/thunder/odometry</odom_topic>" in text
    assert "<frame_id>odom</frame_id>" in text
    assert "<child_frame_id>body</child_frame_id>" in text

    cfg = GazeboBridgeConfig(world_name="lingtu_gazebo_empty", robot_name="thunder")
    assert cfg.gazebo_lidar_scan in text
    assert cfg.gazebo_color_image in text
    assert cfg.gazebo_depth_image in text


def test_gazebo_cmd_vel_adapter_preserves_lingtu_stamped_command_boundary():
    adapter = _read("sim/engine/bridge/gazebo_cmd_vel_adapter.py")

    assert '"/nav/cmd_vel"' in adapter
    assert '"/lingtu/gazebo/cmd_vel"' in adapter
    assert "TwistStamped" in adapter
    assert "Twist" in adapter


def test_gazebo_runtime_adapter_normalizes_frames_and_avoids_control_publication():
    adapter = _read("sim/engine/bridge/gazebo_runtime_adapter.py")

    assert '"/nav/odometry"' not in adapter
    assert "lingtu_odometry" in adapter
    assert "lingtu_map_cloud" in adapter
    assert "lingtu_registered_cloud" in adapter
    assert 'out.header.frame_id = "odom"' in adapter
    assert 'out.child_frame_id = "body"' in adapter
    assert 'map_to_odom.header.frame_id = "map"' in adapter
    assert 'map_to_odom.child_frame_id = "odom"' in adapter
    assert 'body_to_lidar.header.frame_id = "body"' in adapter
    assert "body_to_lidar.child_frame_id = self._cfg.frames.lidar_frame" in adapter
    assert 'body_to_camera.header.frame_id = "body"' in adapter
    assert "body_to_camera.child_frame_id = self._cfg.frames.camera_frame" in adapter
    assert 'tf.header.frame_id = "odom"' in adapter
    assert 'tf.child_frame_id = "body"' in adapter
    assert "lidar_to_body_x" in adapter
    assert "point_cloud2.read_points" in adapter
    assert "LaserScan" in adapter
    assert "create_cloud_xyz32" in adapter
    assert "_on_scan" in adapter
    assert "_body_cloud_to_odom" in adapter
    assert "does not do geometric point transforms" not in adapter
    assert "create_subscription" in adapter
    assert '"/nav/cmd_vel"' not in adapter
    assert "create_publisher(Twist" not in adapter


def test_gazebo_runtime_adapter_point_transform_math():
    out = _transform_xyz(
        (1.0, 0.0, 0.0),
        translation=(0.28, 0.0, 0.20),
        rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    assert out == (1.28, 0.0, 0.20)

    qz_90 = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    rotated = _transform_xyz(
        (1.0, 0.0, 0.0),
        translation=(2.0, 3.0, 0.0),
        rotation_xyzw=qz_90,
    )
    assert rotated[0] == pytest.approx(2.0, abs=1e-6)
    assert rotated[1] == pytest.approx(4.0, abs=1e-6)
    assert rotated[2] == pytest.approx(0.0, abs=1e-6)


def test_tf_contract_smoke_is_read_only_and_checks_runtime_chain():
    smoke = _read("tests/integration/tf_contract_smoke.py")

    assert "map->odom->body" in smoke
    assert '"/nav/odometry"' in smoke
    assert '"/nav/map_cloud"' in smoke
    assert '"/nav/registered_cloud"' in smoke
    assert '"/camera/color/image_raw"' in smoke
    assert '"lingtu.gazebo_runtime_smoke.v1"' in smoke
    assert "--require-sensors" in smoke
    assert "--require-camera" in smoke
    assert "--json-out" in smoke
    assert "required_observations_ready" in smoke
    assert "lookup_transform(\"map\", \"odom\"" in smoke
    assert "lookup_transform(\"odom\", \"body\"" in smoke
    assert "lookup_transform(\"body\", \"lidar_link\"" in smoke
    assert "lookup_transform(\"body\", \"camera_link\"" in smoke
    assert "create_publisher" not in smoke
    assert '"/nav/cmd_vel"' not in smoke
    assert '"/nav/goal_pose"' not in smoke


def test_sim_navigation_launch_can_reuse_native_chain_with_gazebo_odometry():
    launch = _read("tests/planning/sim_navigation.launch.py")

    assert "use_sim_robot" in launch
    assert "use_terrain_passthrough" in launch
    assert "use_foxglove" in launch
    assert "IfCondition(use_sim_robot)" in launch
    assert "IfCondition(use_terrain_passthrough)" in launch
    assert "IfCondition(use_foxglove)" in launch
    assert "terrain_passthrough_node.py" in launch
    assert "Gazebo-only: map_cloud -> terrain_map topics" in launch
    assert "use_sim_robot_arg" in launch
    assert "use_terrain_passthrough_arg" in launch
    assert "use_foxglove_arg" in launch
    assert "_planner_python" in launch
    assert "os.path.exists(_venv_python)" in launch
    assert "source_global_planner_script" in launch
    assert "installed_global_planner_script" in launch
    assert "'legacy'" in launch
    assert "'global_planner.py'" in launch
    assert "prepare_pct_runtime" in _read(
        "src/global_planning/PCT_planner/planner/scripts/legacy/global_planner.py"
    )


def test_gazebo_nav_loop_gate_publishes_only_goal_and_checks_motion():
    smoke = _read("tests/integration/gazebo_nav_loop_smoke.py")
    gate = _read("sim/scripts/gazebo_runtime_gate.py")
    passthrough = _read("tests/planning/terrain_passthrough_node.py")

    assert '"lingtu.gazebo_nav_loop.v1"' in smoke
    assert '"/nav/goal_pose"' in smoke
    assert '"/nav/global_path"' in smoke
    assert '"/nav/local_path"' in smoke
    assert '"/nav/cmd_vel"' in smoke
    assert '"/nav/odometry"' in smoke
    assert "cmd_vel_sent_to_hardware" in smoke
    assert "goal_pub.publish(goal)" in smoke
    assert "create_publisher(PoseStamped, \"/nav/goal_pose\"" in smoke
    assert "create_publisher(Twist" not in smoke
    assert "--check-nav-loop" in gate
    assert "use_sim_robot:=false" in gate
    assert "use_terrain_passthrough:=true" in gate
    assert "gazebo_nav_loop_smoke.py" in gate
    assert "nav_loop" in gate
    assert '"/nav/map_cloud"' in passthrough
    assert '"/nav/terrain_map"' in passthrough
    assert '"/nav/terrain_map_ext"' in passthrough
