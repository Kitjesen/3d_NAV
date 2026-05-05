"""Route registration helpers for GatewayModule."""

from .assets import mount_dashboard_assets
from .app import register_app_routes
from .auth import register_auth_routes
from .camera import register_camera_routes
from .commands import register_command_routes
from .diagnostics import register_diagnostic_routes
from .maps import map_lifecycle_payload, register_map_routes
from .operations import register_operation_routes
from .realtime import register_realtime_routes
from .session import register_session_routes
from .status import register_status_routes

__all__ = [
    "mount_dashboard_assets",
    "register_app_routes",
    "register_auth_routes",
    "register_camera_routes",
    "register_command_routes",
    "register_diagnostic_routes",
    "map_lifecycle_payload",
    "register_map_routes",
    "register_operation_routes",
    "register_realtime_routes",
    "register_session_routes",
    "register_status_routes",
]
