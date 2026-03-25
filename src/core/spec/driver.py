"""Protocol interfaces for the Driver layer (L1).

Defines the minimal contract a robot hardware driver must satisfy so
that higher layers (planning, safety) can interact with the physical
platform without caring about the concrete implementation.

Port type annotations (``In[Twist]``, ``Out[Odometry]``) are expressed
as string literals to avoid hard dependencies on the ``msgs`` module.
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class RobotDriver(Protocol):
    """Robot hardware driver interface.

    Concrete implementations: HanDogBridge (gRPC ↔ brainstem CMS),
    GenericRobotDriver (serial), simulation stubs, etc.

    At minimum a driver must expose typed stream ports for velocity
    commands, odometry feedback, and a liveness flag.
    """

    cmd_vel: Any      # In[Twist]  -- velocity command input
    odometry: Any     # Out[Odometry] -- odometry output
    alive: Any        # Out[bool]  -- heartbeat / liveness flag
