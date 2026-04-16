"""NativeModule factory for the TARE exploration planner (CMU).

Launches the `tare_planner` C++ ROS2 node from the `third_party/tare_planner`
submodule. The binary must be built beforehand via
``scripts/build/build_tare.sh`` so the colcon install prefix layout matches
what ``core.native_install.exe`` expects.

Topic contract is fixed via remappings so the TARE node slots into LingTu's
existing DDS naming (``/nav/*``) without patching upstream code.
"""

from __future__ import annotations

from typing import Any

from core.config import RobotConfig, get_config
from core.native_install import DDS_ENV, exe, share
from core.native_module import NativeModule, NativeModuleConfig


# ── Topic remappings: TARE default  →  LingTu /nav/* ─────────────────────────
# TARE's upstream topic names are terse (`/terrain_map`, `/state_estimation`,
# `/registered_scan`). We remap them onto LingTu's canonical ``/nav/*``
# namespace so the rest of the stack is unaware a third-party node is running.
_TARE_REMAPS = {
    # Subs (sensor/SLAM inputs)
    "/terrain_map":       "/nav/terrain_map",
    "/terrain_map_ext":   "/nav/terrain_map_ext",
    "/state_estimation":  "/nav/odometry",
    "/registered_scan":   "/nav/registered_cloud",
    "/overall_map":       "/nav/map_cloud",
    # Pubs (goal / path outputs)
    "/way_point":         "/exploration/way_point",
    "/exploration_path":  "/exploration/path",
    "/runtime_breakdown": "/exploration/runtime_breakdown",
    "/runtime":           "/exploration/runtime",
    "/exploration_finish": "/exploration/finish",
    "/start_exploration": "/exploration/start",
}


def tare_explorer(cfg: RobotConfig | None = None,
                   scenario: str = "forest") -> NativeModule:
    """TARE hierarchical exploration planner (CMU RSS 2021).

    Args:
        cfg: robot config (defaults to :func:`core.config.get_config`).
        scenario: one of the pre-tuned TARE scenario configs. Options
            (from upstream ``config/`` directory) are ``campus``, ``forest``,
            ``garage``, ``indoor``, ``matterport``, ``tunnel``. LingTu's
            outdoor quadruped typically uses ``forest``.

    The scenario file path can be overridden in ``robot_config.yaml`` under
    ``exploration.tare_config``.
    """
    cfg = cfg or get_config()
    default_yaml = f"{scenario}.yaml"
    config_path = cfg.raw.get("exploration", {}).get(
        "tare_config",
        share(cfg, "tare_planner", "config", default_yaml),
    )
    return NativeModule(NativeModuleConfig(
        executable=exe(cfg, "tare_planner", "tare_planner_node"),
        name="tare_explorer",
        parameters={
            "config_path": config_path,
            # Keep TARE idle until LingTu publishes the start signal
            "auto_start": False,
        },
        remappings=_TARE_REMAPS,
        env=DDS_ENV,
        auto_restart=True,
        max_restarts=3,
    ))
