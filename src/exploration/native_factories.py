"""NativeModule factory for the TARE exploration planner (CMU).

Launches the ``tare_planner`` C++ ROS2 node vendored in-tree at
``src/exploration/tare_planner``. Build with ``scripts/build/build_tare.sh``
(which auto-invokes ``fetch_ortools.sh`` on first run). The binary lands
under the LingTu colcon install prefix where ``core.native_install.exe``
resolves it.

Topic remappings below put the TARE node in LingTu's ``/nav/*`` and
``/exploration/*`` namespaces so the rest of the stack is unaware a
third-party node is running.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from core.config import RobotConfig, get_config
from core.native_install import DDS_ENV, exe, share
from core.native_module import NativeModule, NativeModuleConfig
from core.runtime_interface import TOPICS, adapter_remappings

# Topic remappings: TARE default names -> LingTu runtime contract.
# TARE's upstream topic names are terse (`/terrain_map`, `/state_estimation`,
# `/registered_scan`). We remap them onto LingTu's canonical ``/nav/*``
# namespace so the rest of the stack is unaware a third-party node is running.
TARE_REMAPS = {
    **adapter_remappings("tare"),
    # Subs (sensor/SLAM inputs)
    "/overall_map":       TOPICS.map_cloud,
    # Pubs (goal / path outputs)
    "/global_path_full":  "/exploration/global_path_full",
    "/global_path":       "/exploration/global_path",
    "/local_path":        "/exploration/local_path",
    "/old_global_path":   "/exploration/old_global_path",
    "/to_nearest_global_subspace_path": "/exploration/to_nearest_global_subspace_path",
    "/exploration_path":  "/exploration/path",
    "/runtime_breakdown": "/exploration/runtime_breakdown",
    "/runtime":           "/exploration/runtime",
    "/exploration_finish": "/exploration/finish",
    "/start_exploration": "/exploration/start",
}


def _shared_library_on_path(name: str, path_value: str) -> bool:
    for item in path_value.split(":"):
        if item and (Path(item) / name).exists():
            return True
    return False


def _tare_native_env() -> dict[str, str]:
    """Return a native-node env that does not force an unavailable RMW."""
    env: dict[str, str] = {}
    ld_library_path = DDS_ENV.get("LD_LIBRARY_PATH", "")
    if ld_library_path:
        env["LD_LIBRARY_PATH"] = ld_library_path
    rmw = DDS_ENV.get("RMW_IMPLEMENTATION", "")
    if rmw and _shared_library_on_path(f"lib{rmw}.so", ld_library_path):
        env["RMW_IMPLEMENTATION"] = rmw
    return env


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
    configured_path = cfg.raw.get("exploration", {}).get(
        "tare_config",
        share(cfg, "tare_planner", "config", default_yaml),
    )
    config_path = str(configured_path)
    if not Path(config_path).is_file():
        source_config = Path(__file__).resolve().parent / "tare_planner" / "config" / default_yaml
        if source_config.is_file():
            config_path = str(source_config)
    params: dict[str, Any] = {
        # Keep TARE idle until LingTu publishes the start signal. This must
        # override scenario YAML files, which set kAutoStart=true upstream.
        "kAutoStart": False,
    }
    if not Path(config_path).is_file():
        # Preserve the selected path in health/logging even when the share tree
        # is incomplete; setup() will still fail loudly if the executable is
        # missing, and the node will run with compiled defaults if launched.
        params["config_path"] = config_path

    return NativeModule(NativeModuleConfig(
        executable=exe(cfg, "tare_planner", "tare_planner_node"),
        name="tare_explorer",
        parameter_files=[config_path] if Path(config_path).is_file() else [],
        parameters=params,
        remappings=TARE_REMAPS,
        env=_tare_native_env(),
        auto_restart=True,
        max_restarts=3,
    ))
