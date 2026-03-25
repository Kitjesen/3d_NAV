"""LingTu navigation system blueprint -- one line to wire everything.

Wires all converted hive Modules into a complete navigation stack:

    L0  SafetyModule          -- Ring 1 reflex safety aggregator
    L1  ThunderDriver          -- quadruped gRPC bridge (cmd_vel -> Walk)
    L2  EvaluatorModule       -- Ring 2 closed-loop execution evaluator
    L5  PathAdapterModule     -- global path -> waypoint tracker
    L5  MissionArcModule      -- mission lifecycle FSM
    L6  DialogueModule        -- Ring 3 user-facing dialogue state

Auto-wire connects ports that share (name, msg_type). Explicit wires
handle the cases where port names differ but carry the same signal:

    SafetyModule.stop_cmd (int) -> ThunderDriver.stop_signal (int)
    SafetyModule.stop_cmd (int) -> MissionArcModule.stop_signal (int)

Usage::

    from core.blueprints.navigation import navigation_blueprint

    system = navigation_blueprint(dog_host="192.168.66.190").build()
    system.start()
"""

from __future__ import annotations

import sys
import os
from typing import Any

# Ensure src/ is importable
_src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _src_dir not in sys.path:
    sys.path.insert(0, _src_dir)

from core.blueprint import Blueprint


def navigation_blueprint(**config: Any) -> Blueprint:
    """Build the full LingTu navigation stack blueprint.

    All module imports are deferred so this file stays importable even if
    optional dependencies (grpc, numpy, etc.) are missing at import time.

    Args:
        dog_host: gRPC address for HanDog CMS (default "127.0.0.1")
        dog_port: gRPC port (default 13145)
        max_replan_count: mission replan budget (default 3)
        Any other key is silently ignored.

    Returns:
        A ready-to-build Blueprint with auto_wire + explicit cross-name wires.
    """
    from drivers.thunder.han_dog_module import ThunderDriver
    from nav.rings.nav_rings.safety_module import SafetyModule
    from nav.rings.nav_rings.evaluator_module import EvaluatorModule
    from nav.rings.nav_rings.dialogue_module import DialogueModule
    from global_planning.pct_adapters.src.path_adapter_module import PathAdapterModule
    from global_planning.pct_adapters.src.mission_arc_module import MissionArcModule

    bp = Blueprint()

    # -- Register modules (layer order is for readability only) ----------------

    bp.add(SafetyModule)
    bp.add(ThunderDriver,
           dog_host=config.get("dog_host", "127.0.0.1"),
           dog_port=config.get("dog_port", 13145))
    bp.add(EvaluatorModule)
    bp.add(PathAdapterModule)
    bp.add(MissionArcModule,
           max_replan_count=config.get("max_replan_count", 3))
    bp.add(DialogueModule)

    # -- Tier 1: Safety-critical — direct callback (zero latency) --------------
    bp.wire("SafetyModule", "stop_cmd",
            "ThunderDriver", "stop_signal")
    bp.wire("SafetyModule", "stop_cmd",
            "MissionArcModule", "stop_signal")

    # -- Tier 2: Control loop — direct callback (low latency, tight coupling) --
    # Odometry, adapter_status, safety_state stay as auto_wire callbacks.
    # These are lightweight messages at 10-50Hz.

    # -- Auto-wire remaining connections by (name, msg_type) match -------------
    bp.auto_wire()

    return bp
