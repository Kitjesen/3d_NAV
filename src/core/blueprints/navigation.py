"""LingTu navigation blueprint — ThunderDriver + new module architecture.

Usage::

    from core.blueprints.navigation import navigation_blueprint
    system = navigation_blueprint(dog_host="192.168.66.190").build()
    system.start()
"""

from __future__ import annotations
from typing import Any
from core.blueprint import Blueprint


def navigation_blueprint(**config: Any) -> Blueprint:
    """Build navigation stack: ThunderDriver + Navigation + SafetyRing."""
    from drivers.thunder.han_dog_module import ThunderDriver
    from nav.navigation_module import NavigationModule
    from nav.safety_ring_module import SafetyRingModule

    bp = Blueprint()
    bp.add(ThunderDriver,
           dog_host=config.get("dog_host", "127.0.0.1"),
           dog_port=config.get("dog_port", 13145))
    bp.add(NavigationModule, planner=config.get("planner", "astar"))
    bp.add(SafetyRingModule)

    bp.wire("SafetyRingModule", "stop_cmd", "ThunderDriver", "stop_signal")
    bp.auto_wire()
    return bp
