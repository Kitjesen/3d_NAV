"""AutonomyModule — unified C++ autonomy stack as one Module.

Manages 4 C++ NativeModule nodes as a single unit:
  terrain_analysis     — ground estimation + obstacle detection
  terrain_analysis_ext — connectivity + 2.5D height map
  local_planner        — obstacle avoidance + path scoring
  path_follower        — Pure Pursuit waypoint tracking → cmd_vel

One Module to add to Blueprint, internally manages all 4 C++ processes.

Usage::

    system = autoconnect(
        AutonomyModule.blueprint(),  # starts all 4 C++ nodes
        NavigationModule.blueprint(planner="astar"),
        ThunderDriver.blueprint(dog_host="192.168.66.190"),
        ...
    ).build()
"""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from core.module import Module
from core.stream import Out
from core.config import RobotConfig, get_config
from core.native_module import NativeModule
from core.native_factories import (
    terrain_analysis,
    terrain_analysis_ext,
    local_planner,
    path_follower,
)
from core.registry import register

logger = logging.getLogger(__name__)


@register("autonomy", "default", description="Unified C++ autonomy stack (terrain+planner)")
class AutonomyModule(Module, layer=2):
    """Unified autonomy stack — manages 4 C++ NativeModule processes.

    No In/Out data ports — C++ nodes communicate via DDS topics directly.
    This Module only manages lifecycle (start/stop/health).
    """

    alive: Out[bool]

    def __init__(self, cfg: Optional[RobotConfig] = None, **kw):
        super().__init__(**kw)
        self._cfg = cfg or get_config()
        self._nodes: Dict[str, NativeModule] = {}

    def setup(self):
        """Create all 4 NativeModule instances."""
        self._nodes = {
            "terrain": terrain_analysis(self._cfg),
            "terrain_ext": terrain_analysis_ext(self._cfg),
            "local_planner": local_planner(self._cfg),
            "path_follower": path_follower(self._cfg),
        }
        # Setup each (validates binary exists)
        for name, node in self._nodes.items():
            try:
                node.setup()
            except (FileNotFoundError, PermissionError) as e:
                logger.warning("AutonomyModule: %s setup failed: %s", name, e)

    def start(self):
        """Start all 4 C++ processes."""
        super().start()
        started = 0
        for name, node in self._nodes.items():
            try:
                node.start()
                started += 1
                logger.info("AutonomyModule: %s started", name)
            except Exception as e:
                logger.error("AutonomyModule: %s start failed: %s", name, e)
        self.alive.publish(started == len(self._nodes))
        logger.info("AutonomyModule: %d/%d C++ nodes started", started, len(self._nodes))

    def stop(self):
        """Stop all C++ processes (SIGTERM → SIGKILL)."""
        for name, node in reversed(list(self._nodes.items())):
            try:
                node.stop()
                logger.info("AutonomyModule: %s stopped", name)
            except Exception as e:
                logger.warning("AutonomyModule: %s stop error: %s", name, e)
        self._nodes.clear()
        self.alive.publish(False)
        super().stop()

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        node_health = {}
        for name, node in self._nodes.items():
            h = node.health()
            native = h.get("native", {})
            node_health[name] = {
                "running": native.get("running", False),
                "pid": native.get("pid"),
                "restart_count": native.get("restart_count", 0),
            }
        info["autonomy"] = {
            "nodes": node_health,
            "total": len(self._nodes),
            "running": sum(1 for h in node_health.values() if h["running"]),
        }
        return info
