"""Safety stack: SafetyRing + Geofence."""

from __future__ import annotations

from core.blueprint import Blueprint


def safety() -> Blueprint:
    """Safety reflex + geofence boundary enforcement + cmd_vel arbitration."""
    bp = Blueprint()

    from nav.safety_ring_module import SafetyRingModule
    bp.add(SafetyRingModule)

    from nav.cmd_vel_mux_module import CmdVelMux
    bp.add(CmdVelMux)

    try:
        from nav.services.nav_services.geofence_manager_module import GeofenceManagerModule
        bp.add(GeofenceManagerModule)
    except ImportError:
        pass

    return bp
