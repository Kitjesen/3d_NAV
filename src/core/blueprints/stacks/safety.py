"""Safety stack: SafetyRing + Geofence."""

from __future__ import annotations

from core.blueprint import Blueprint
from core.blueprints.stacks._registry import optional_stack_module, stack_module


def safety() -> Blueprint:
    """Safety reflex + geofence boundary enforcement + cmd_vel arbitration."""
    bp = Blueprint()

    SafetyRingModule = stack_module(
        "safety",
        "ring",
        seed_group="safety",
        fallback="nav.safety_ring_module.SafetyRingModule",
    )
    bp.add(SafetyRingModule, alias="SafetyRingModule")

    CmdVelMux = stack_module(
        "safety",
        "cmd_vel_mux",
        seed_group="safety",
        fallback="nav.cmd_vel_mux_module.CmdVelMux",
    )
    bp.add(CmdVelMux, alias="CmdVelMux")

    GeofenceManagerModule = optional_stack_module(
        "safety",
        "geofence",
        seed_group="safety",
        fallback="nav.services.nav_services.geofence_manager_module.GeofenceManagerModule",
    )
    if GeofenceManagerModule is not None:
        bp.add(GeofenceManagerModule, alias="GeofenceManagerModule")

    return bp
