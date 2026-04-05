import json
import os
from pathlib import Path


def test_mid360_config_generated_from_robot_config(tmp_path, monkeypatch):
    # Avoid touching the real HOME on CI machines
    monkeypatch.setenv("HOME", str(tmp_path))

    from core.config import RobotConfig
    from core.utils.livox_config import build_mid360_config_dict, ensure_mid360_config_file

    cfg = RobotConfig()
    cfg.lidar.lidar_ip = "192.168.9.10"
    cfg.lidar.host_ip = "192.168.9.5"

    d = build_mid360_config_dict(cfg)
    assert d["lidar_configs"][0]["ip"] == "192.168.9.10"
    assert d["MID360"]["host_net_info"]["cmd_data_ip"] == "192.168.9.5"
    assert d["MID360"]["host_net_info"]["point_data_ip"] == "192.168.9.5"
    assert d["MID360"]["host_net_info"]["imu_data_ip"] == "192.168.9.5"

    out = ensure_mid360_config_file(cfg)
    assert out.endswith("MID360_config.json")
    assert Path(out).exists()
    parsed = json.loads(Path(out).read_text(encoding="utf-8"))
    assert parsed["lidar_configs"][0]["ip"] == "192.168.9.10"
    assert parsed["MID360"]["host_net_info"]["cmd_data_ip"] == "192.168.9.5"

