from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]


def _read(path: str) -> str:
    return (REPO_ROOT / path).read_text(encoding="utf-8")


def test_robot_localizer_service_matches_slam_bridge_topics():
    text = _read("docs/04-deployment/services/robot-localizer.service")

    assert "MAP_PATH=$${NAV_MAP_PATH:-/home/sunrise/data/nova/maps/active/map}" in text
    assert "-r map_cloud:=/nav/saved_map_cloud" in text
    assert "-r /localization_quality:=/nav/localization_quality" in text
    assert "-r global_relocalize:=/nav/global_relocalize" in text
    assert "-r map_cloud:=/nav/map_cloud" not in text


def test_s100p_localizer_template_matches_relocalize_api():
    text = _read("scripts/deploy/s100p/localizer.service")

    assert "MAP_PATH=$${NAV_MAP_PATH:-/home/sunrise/data/nova/maps/active/map}" in text
    assert "/home/sunrise/data/inovxio/data/maps/active/map" not in text
    assert "-r map_cloud:=/nav/saved_map_cloud" in text
    assert "-r /localization_quality:=/nav/localization_quality" in text
    assert "-r global_relocalize:=/nav/global_relocalize" in text
    assert "-r map_cloud:=/nav/map_cloud" not in text


def test_localizer_launch_profile_matches_topic_contract():
    text = _read("launch/profiles/localizer_icp.launch.py")

    assert 'default_value="/home/sunrise/data/nova/maps/active/map"' in text
    assert '("/cloud_registered", "/nav/registered_cloud")' in text
    assert '("/Odometry", "/nav/odometry")' in text
    assert '("map_cloud", "/nav/saved_map_cloud")' in text
    assert '("/localization_quality", "/nav/localization_quality")' in text
    assert '("global_relocalize", "/nav/global_relocalize")' in text
    assert '("map_cloud", "/nav/map_cloud")' not in text


def test_topic_contract_names_static_map_and_global_relocalize():
    text = _read("config/topic_contract.yaml")

    assert "saved_map_cloud: /nav/saved_map_cloud" in text
    assert "quality: /nav/localization_quality" in text
    assert "global_relocalize_service: /nav/global_relocalize" in text


def test_lingtu_doctor_reports_standard_quality_topic_with_legacy_hint():
    text = _read("scripts/lingtu")

    assert "/nav/localization_quality" in text
    assert "/localization_quality" in text
    assert "legacy; remap to /nav/localization_quality" in text


def test_lingtu_doctor_json_gates_runtime_readiness_freshness():
    text = _read("scripts/lingtu")

    assert "gateway.slam_stream" in text
    assert "gateway.localization_status" in text
    assert "add_dataflow_pressure_check" in text
    assert '"VoxelGridModule", "map_cloud", "p1"' in text
    assert "LINGTU_DOCTOR_MAX_DATAFLOW_P95_MS" in text
    assert "LINGTU_DOCTOR_MAX_DATAFLOW_DROP_RATIO" in text
    assert "callback pressure detected" in text
    assert "LINGTU_DOCTOR_MIN_SLAM_HZ" in text
    assert "LINGTU_DOCTOR_MAX_ODOM_AGE_MS" in text
    assert "LINGTU_DOCTOR_MAX_LOC_DIAG_AGE_MS" in text
    assert "LINGTU_DOCTOR_MAX_LOCALIZER_HEALTH_AGE_MS" in text
    assert "odom_age_ms" in text
    assert "diag_age_ms" in text
    assert "localizer_health_topic_age_ms" in text
    assert "map_cloud_fresh=false" in text


def test_lingtu_doctor_realtime_smoke_is_explicit_non_motion_gate():
    text = _read("scripts/lingtu")

    assert "Usage: lingtu doctor [--non-motion] [--json] [--strict] [--realtime]" in text
    assert "--realtime) realtime=1" in text
    assert "gateway.realtime_sse" in text
    assert "gateway.websocket_camera" in text
    assert "gateway.websocket_cloud" in text
    assert 'ws_smoke("/ws/camera", b"\\xff\\xd8\\xff"' in text
    assert 'ws_smoke("/ws/cloud", b"PCL"' in text


def test_lingtu_doctor_normalizes_structured_active_command_source():
    text = _read("scripts/lingtu")

    assert "def command_source_name(control):" in text
    assert 'source.get("name") or source.get("source") or source.get("owner") or "none"' in text
    assert 'isinstance(s,dict)' in text


def test_lingtu_slamcheck_writes_float_relocation_initial_pose():
    text = _read("scripts/lingtu")

    assert "initial_pose=$(printf '[%.6f,%.6f,0.0,0.0,0.0,%.6f]'" in text
    assert "SUPER_LIO_RELOCATION_INIT_POSE=${initial_pose:-[0.0,0.0,0.0,0.0,0.0,0.0]}" in text
    assert "SUPER_LIO_RELOCATION_INIT_POSE=[0,0,0.0,0.0,0.0,0]" not in text


def test_super_lio_relocation_service_uses_active_map_and_float_pose():
    text = _read("scripts/deploy/s100p/super_lio_relocation.service")

    assert "Environment=SUPER_LIO_RELOCATION_MAP_DIR=/home/sunrise/data/nova/maps/active" in text
    assert "Environment=SUPER_LIO_RELOCATION_INIT_POSE=[0.0,0.0,0.0,0.0,0.0,0.0]" in text
    assert '-p "lio.relocation.init_pose:=$${SUPER_LIO_RELOCATION_INIT_POSE}"' in text
    assert '-p "lio.map.save_map_dir:=$${EFFECTIVE_MAP_DIR}"' in text


def test_camera_deployment_retires_legacy_units():
    install = _read("docs/04-deployment/services/install.sh")
    readme = _read("docs/04-deployment/README.md")
    camera_service = _read("docs/04-deployment/services/robot-camera.service")

    assert "robot-camera.service" in install
    assert "camera.service orbbec-camera.service" in install
    assert 'systemctl mask "$legacy"' in install
    assert "duplicate /camera ROS node names" in install
    assert "Only `robot-camera.service` should own the Orbbec ROS nodes" in readme
    assert "KillMode=control-group" in camera_service
