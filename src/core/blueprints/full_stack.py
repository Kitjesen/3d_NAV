"""Full-stack navigation blueprint - composable factory pattern.

Usage::

    # dimos-style one-liner per stack
    system = full_stack_blueprint(robot="thunder", slam_profile="localizer").build()
    system.start()

    # Or compose directly
    from core.blueprints.stacks import *
    system = autoconnect(
        driver("thunder", host="192.168.66.190"),
        slam("localizer"),
        maps(),
        perception("bpu"),
        memory(),
        planner("kimi"),
        navigation("astar"),
        safety(),
        gateway(5050),
    ).build()
"""

from __future__ import annotations

import logging
from typing import Any

from core.blueprint import Blueprint, autoconnect
from core.utils.calibration_check import run_calibration_check

from .full_stack_wiring import apply_full_stack_wires
from .stacks import (
    driver,
    exploration,
    gateway,
    lidar,
    maps,
    memory,
    navigation,
    perception,
    safety,
    sim_lidar,
    slam,
)
from .stacks import planner as planner_stack
from .stacks.driver import driver_name
from .stacks.memory import DEFAULT_SEMANTIC_DIR
from .stacks.slam import normalize_slam_profile

logger = logging.getLogger(__name__)


def full_stack_blueprint(
    robot: str = "thunder",
    slam_profile: str = "fastlio2",
    detector: str = "yoloe",
    encoder: str = "mobileclip",
    llm: str = "kimi",
    planner_backend: str = "astar",
    tomogram: str = "",
    gateway_port: int = 5050,
    teleop_port: int = 5050,  # teleop is now on /ws/teleop of the main gateway port
    enable_native: bool = True,
    enable_semantic: bool = True,
    enable_gateway: bool = True,
    enable_teleop: bool = True,
    enable_map_modules: bool = True,
    enable_rerun: bool = False,
    scene_xml: str = "",
    run_startup_checks: bool = True,
    manage_external_services: bool = True,
    # Legacy alias
    planner: str = "",
    **config: Any,
) -> Blueprint:
    """Build the complete LingTu navigation stack from composable factories.

    Each stack is a factory function returning a Blueprint.
    autoconnect() merges them and auto-wires by (port_name, msg_type).
    """
    planner_backend = planner or planner_backend
    slam_profile = normalize_slam_profile(slam_profile)
    semantic_save_dir = config.get("semantic_save_dir", DEFAULT_SEMANTIC_DIR)
    drv = driver_name(robot)

    needs_camera = enable_semantic or slam_profile not in ("", "none")
    needs_slam = slam_profile not in ("", "none")
    if run_startup_checks:
        calib = run_calibration_check(
            require_camera=needs_camera,
            require_slam=needs_slam,
        )
        if not calib.ok:
            raise RuntimeError(
                f"Calibration self-check failed ({len(calib.errors)} error(s)): "
                + "; ".join(calib.errors)
            )

    driver_config = dict(config)
    if enable_semantic and drv == "MujocoDriverModule":
        driver_config.setdefault("enable_camera", True)

    perception_config = dict(config)
    perception_config["_driver_cls_name"] = drv

    needs_lidar = slam_profile not in (
        "",
        "none",
        "bridge",
        "super_lio",
        "super_lio_relocation",
    )
    lidar_ip = config.get("lidar_ip")

    device_bp = Blueprint()
    try:
        import os
        from pathlib import Path

        devices_yaml = Path(__file__).resolve().parents[3] / "config" / "devices.yaml"
        if devices_yaml.exists():
            from core.devices import DeviceManager

            device_bp.add(
                DeviceManager,
                config_path=str(devices_yaml),
                enable_hotplug=os.environ.get("LINGTU_HOTPLUG", "0") == "1",
            )
    except Exception as exc:
        logging.getLogger(__name__).debug("DeviceManager not loaded: %s", exc)

    gnss_bp = Blueprint()
    try:
        from core.config import get_config

        gnss_cfg = get_config().raw.get("gnss", {})
        if gnss_cfg.get("enabled", False):
            from slam.gnss_bridge import GnssBridgeModule
            from slam.gnss_module import GnssModule

            gnss_bp.add(
                GnssModule,
                device_model=gnss_cfg.get("model", "WTRTK-980"),
                origin_lat=(gnss_cfg.get("origin") or {}).get("lat"),
                origin_lon=(gnss_cfg.get("origin") or {}).get("lon"),
                origin_alt=(gnss_cfg.get("origin") or {}).get("alt"),
                auto_init_origin=(gnss_cfg.get("origin") or {}).get("auto_init", True),
                min_sat_used=(gnss_cfg.get("quality") or {}).get("min_sat_used", 8),
                max_hdop=(gnss_cfg.get("quality") or {}).get("max_hdop", 2.5),
            )
            gnss_bp.add(
                GnssBridgeModule,
                device_id=gnss_cfg.get("device_id", "wtrtk980_main"),
            )
            rtcm_cfg = gnss_cfg.get("rtcm") or {}
            if rtcm_cfg.get("enabled", False):
                from slam.ntrip_client_module import NtripClientModule

                gnss_bp.add(
                    NtripClientModule,
                    enabled=True,
                    host=rtcm_cfg.get("ntrip_host", ""),
                    port=int(rtcm_cfg.get("ntrip_port", 2101)),
                    mount=rtcm_cfg.get("ntrip_mount", ""),
                    user=rtcm_cfg.get("ntrip_user", ""),
                    password=rtcm_cfg.get("ntrip_pass", ""),
                )
    except Exception as exc:
        logging.getLogger(__name__).debug("GNSS not loaded: %s", exc)

    bp = autoconnect(
        device_bp,
        driver(robot, **driver_config),
        lidar(ip=lidar_ip, enabled=needs_lidar),
        sim_lidar(scene_xml=scene_xml),
        slam(slam_profile, manage_services=manage_external_services),
        gnss_bp,
        maps(**config) if enable_map_modules else Blueprint(),
        perception(
            detector,
            encoder,
            manage_services=manage_external_services,
            **perception_config,
        )
        if enable_semantic
        else Blueprint(),
        memory(semantic_save_dir) if enable_semantic else Blueprint(),
        planner_stack(llm, semantic_save_dir) if enable_semantic else Blueprint(),
        navigation(planner_backend, tomogram, enable_native, **config),
        exploration(
            backend=config.get("exploration_backend", "none"),
            tare_scenario=config.get("tare_scenario", "forest"),
            auto_start=config.get("exploration_auto_start", True),
        ),
        safety(),
        gateway(
            gateway_port,
            teleop_port=teleop_port,
            enable_teleop=enable_teleop,
            enable_rerun=enable_rerun,
        )
        if enable_gateway
        else Blueprint(),
    )

    return apply_full_stack_wires(
        bp,
        robot=robot,
        driver_module=drv,
        slam_profile=slam_profile,
        scene_xml=scene_xml,
        enable_semantic=enable_semantic,
    )
