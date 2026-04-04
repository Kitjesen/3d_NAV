"""Livox MID-360 driver config generation.

Enterprise-grade rule: single source of truth.

We keep all network addresses in config/robot_config.yaml (RobotConfig.lidar),
and generate the Livox driver JSON at runtime for livox_ros_driver2 to consume.

This avoids manual edits to vendor JSON files and prevents config drift across
robots / deployments.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict

from core.config import RobotConfig


def build_mid360_config_dict(cfg: RobotConfig) -> Dict[str, Any]:
    """Return a MID360_config.json-compatible dict for livox_ros_driver2."""
    lidar_ip = cfg.lidar.lidar_ip
    host_ip = cfg.lidar.host_ip

    # Ports follow Livox reference defaults; override via cfg.raw['lidar'] if needed.
    raw_lidar = (cfg.raw or {}).get("lidar", {})
    ports = raw_lidar.get("livox_ports", {}) if isinstance(raw_lidar, dict) else {}

    def p(name: str, default: int) -> int:
        try:
            v = int(ports.get(name, default))
        except Exception:
            v = default
        return v

    cfg_dict: Dict[str, Any] = {
        "lidar_summary_info": {"lidar_type": 8},
        "MID360": {
            "lidar_net_info": {
                "cmd_data_port": p("cmd_data_port", 56100),
                "push_msg_port": p("push_msg_port", 56200),
                "point_data_port": p("point_data_port", 56300),
                "imu_data_port": p("imu_data_port", 56400),
                "log_data_port": p("log_data_port", 56500),
            },
            "host_net_info": {
                "cmd_data_ip": host_ip,
                "cmd_data_port": p("cmd_data_port_host", 56101),
                "push_msg_ip": host_ip,
                "push_msg_port": p("push_msg_port_host", 56201),
                "point_data_ip": host_ip,
                "point_data_port": p("point_data_port_host", 56301),
                "imu_data_ip": host_ip,
                "imu_data_port": p("imu_data_port_host", 56401),
                "log_data_ip": "",
                "log_data_port": p("log_data_port_host", 56501),
            },
        },
        "lidar_configs": [
            {
                "ip": lidar_ip,
                "pcl_data_type": int(raw_lidar.get("livox_pcl_data_type", 1)),
                "pattern_mode": int(raw_lidar.get("livox_pattern_mode", 0)),
                # Keep extrinsics at zero here. SLAM uses config/robot_config.yaml
                # as the unified TF source of truth.
                "extrinsic_parameter": {
                    "roll": 0.0,
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "x": 0,
                    "y": 0,
                    "z": 0,
                },
            }
        ],
    }
    return cfg_dict


def ensure_mid360_config_file(cfg: RobotConfig, out_dir: str | None = None) -> str:
    """Write a generated MID360_config.json and return its absolute path."""
    base = (
        Path(out_dir).expanduser()
        if out_dir
        else Path(os.path.expanduser("~/.lingtu/generated/livox"))
    )
    base.mkdir(parents=True, exist_ok=True)

    path = base / "MID360_config.json"
    data = build_mid360_config_dict(cfg)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return str(path)

