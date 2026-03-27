"""lingtu.core — Module orchestration framework core, built directly in src/.

Core components:
- transport — Pluggable transport layer (Transport, LocalTransport, SHM, DDS, Dual)
- msgs      — Unified message types (Vector3, Odometry, SceneGraph, ...)
- stream    — Typed data-flow ports (Out[T], In[T]) and transport abstraction
- module    — Module base class with automatic port scanning
- blueprint — Declarative orchestration blueprint (Blueprint, autoconnect, SystemHandle)
- config    — Unified configuration loader (config/robot_config.yaml)
- clock     — Switchable real-time / simulation clock
"""

from .stream import In, Out
from .transport import Transport, LocalTransport
from .module import Module, rpc
from .blueprint import Blueprint, SystemHandle, autoconnect
from .config import RobotConfig, get_config, load_config, reset_config
from .clock import Clock, clock
from .native_module import NativeModule, NativeModuleConfig

__all__ = [
    # transport
    "Transport", "LocalTransport",
    # stream
    "Out", "In",
    # module
    "Module", "rpc",
    # blueprint
    "Blueprint", "SystemHandle", "autoconnect",
    # config
    "RobotConfig", "load_config", "get_config", "reset_config",
    # clock
    "Clock", "clock",
    # native
    "NativeModule", "NativeModuleConfig",
]
