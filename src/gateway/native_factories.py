"""NativeModule factories for gateway C++ nodes.

Usage::

    from gateway.native_factories import grpc_gateway
    from core.config import get_config

    cfg = get_config()
    bp.add(grpc_gateway(cfg), alias="grpc")
"""

from __future__ import annotations

from typing import Optional

from core.config import RobotConfig, get_config
from core.native_install import DDS_ENV, exe
from core.native_module import NativeModule, NativeModuleConfig


def grpc_gateway(cfg: RobotConfig | None = None) -> NativeModule:
    """gRPC gateway — remote control + telemetry (C++)."""
    cfg = cfg or get_config()
    return NativeModule(NativeModuleConfig(
        executable=exe(cfg, "remote_monitoring", "grpc_gateway"),
        name="grpc_gateway",
        parameters={
            "grpc_port": cfg.raw.get("grpc", {}).get("port", 50051),
        },
        env=DDS_ENV,
    ))
