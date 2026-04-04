"""Structured logging for CLI runs."""

from __future__ import annotations

import logging
import sys
import time
from pathlib import Path

from .paths import logs_base_dir

# Loggers that produce high-volume INFO noise during startup.
# They are silenced on stderr but still written to the log file.
_QUIET_PREFIXES = (
    "root",                                     # open_clip / timm model loading
    "timm",
    "open_clip",
    "core.dds",                                 # DDS reader setup
    "core.service_manager",                     # systemd service start/stop
    "core.blueprint",                           # "System started: N modules"
    "slam.slam_bridge_module",                  # DDS transport choice
    "semantic.perception.semantic_perception.mobileclip_encoder",
    "semantic.perception.semantic_perception.encoder_module",
    "semantic.perception.semantic_perception.perception_module",
    "semantic.planner.semantic_planner.person_tracker",
    "drivers.sim.ros2_sim_driver",
    "gateway.mcp_server",
    "memory.modules.vector_memory_module",
    "memory.modules.semantic_mapper_module",
    "memory.knowledge.room_object_kg",
)

# Loggers whose WARNING/ERROR messages are also not useful on screen.
# These are "graceful degradation" notices — the system handles them automatically.
_MUTE_PREFIXES = (
    "semantic.planner.semantic_planner.llm_client",  # "API key not found"
    "base_autonomy.modules.terrain_module",           # "_nav_core not available, falling back"
    "base_autonomy.modules.local_planner_module",     # "_nav_core not available, falling back"
    "base_autonomy.modules.path_follower_module",     # "_nav_core not available, falling back"
)


class _StderrFilter(logging.Filter):
    """Allow only lingtu-important messages through to stderr."""

    def filter(self, record: logging.LogRecord) -> bool:
        name = record.name
        # Always show ERROR and above
        if record.levelno >= logging.ERROR:
            return True
        # Fully mute noisy loggers
        for p in _MUTE_PREFIXES:
            if name == p or name.startswith(p + "."):
                return False
        # Quiet loggers: only show WARNING+
        for p in _QUIET_PREFIXES:
            if name == p or name.startswith(p + "."):
                return record.levelno >= logging.WARNING
        return True


def setup_logging(level: str, profile_name: str) -> str:
    """Stderr (filtered) + full per-run file. Returns log directory path."""
    ts = time.strftime("%Y%m%d_%H%M%S")
    log_dir = logs_base_dir() / f"{ts}_{profile_name}"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "lingtu.log"

    stderr_h = logging.StreamHandler(sys.stderr)
    stderr_h.setFormatter(logging.Formatter(
        "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    ))
    stderr_h.addFilter(_StderrFilter())

    file_h = logging.FileHandler(str(log_file), encoding="utf-8")
    file_h.setFormatter(logging.Formatter(
        "%(asctime)s\t%(levelname)s\t%(name)s\t%(message)s",
    ))

    root = logging.getLogger()
    root.setLevel(getattr(logging, level.upper(), logging.INFO))
    root.addHandler(stderr_h)
    root.addHandler(file_h)

    return str(log_dir)
