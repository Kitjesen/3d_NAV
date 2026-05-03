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
