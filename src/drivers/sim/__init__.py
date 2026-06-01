"""Simulation and test drivers -- no hardware required."""

import time
from typing import Any


def build_sim_robot_state() -> dict[str, Any]:
    """Return a standard robot-state dict shared by all sim drivers.

    Both ``MujocoDriverModule`` and ``ROS2SimDriverModule`` publish
    identical robot state dictionaries.  This helper ensures they stay
    in sync when fields change.
    """
    return {
        "standing": True,
        "enabled": True,
        "emergency": False,
        "connected": True,
        "battery_voltage": 0.0,
        "battery_soc": 0.0,
        "current_gait": "trot",
        "timestamp": time.time(),
    }


from .stub import StubConnection  # noqa: E402
