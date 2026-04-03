"""core.msgs.robot — Quadruped robot state types.

Types for joint states, battery, foot forces, and composite robot state.
Matches the ROS2 interface/msg definitions used by the brainstem CMS gRPC API.

Classes:
    JointState    — 16-DOF joint positions, velocities, efforts
    BatteryState  — SOC, voltage, current, temperature, status
    FootForces    — 4-leg ground reaction forces
    RobotState    — Composite: joints + battery + foot forces + IMU
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List

from .geometry import Quaternion, Vector3
from .sensor import Imu


NUM_JOINTS = 16
NUM_LEGS = 4

# Joint ordering (matches brainstem CMS proto Matrix4):
# [0-2]  FR: hip, thigh, calf
# [3-5]  FL: hip, thigh, calf
# [6-8]  RR: hip, thigh, calf
# [9-11] RL: hip, thigh, calf
# [12-15] FR_foot, FL_foot, RR_foot, RL_foot

JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "FR_foot", "FL_foot", "RR_foot", "RL_foot",
]

LEG_NAMES = ["FR", "FL", "RR", "RL"]


@dataclass
class JointState:
    """16-DOF joint state for quadruped robot.

    All arrays are length 16, ordered per JOINT_NAMES.
    """

    positions: List[float] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    velocities: List[float] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    efforts: List[float] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    ts: float = field(default_factory=time.time)

    def leg(self, leg_idx: int) -> Dict[str, List[float]]:
        """Get joint values for one leg (0=FR, 1=FL, 2=RR, 3=RL).

        Returns dict with 3-element lists for hip, thigh, calf.
        """
        i = leg_idx * 3
        return {
            "positions": self.positions[i:i+3],
            "velocities": self.velocities[i:i+3],
            "efforts": self.efforts[i:i+3],
        }

    def to_dict(self) -> Dict[str, Any]:
        return {
            "positions": list(self.positions),
            "velocities": list(self.velocities),
            "efforts": list(self.efforts),
            "ts": self.ts,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> JointState:
        return cls(
            positions=d.get("positions", [0.0] * NUM_JOINTS),
            velocities=d.get("velocities", [0.0] * NUM_JOINTS),
            efforts=d.get("efforts", [0.0] * NUM_JOINTS),
            ts=d.get("ts", 0.0),
        )

    def __repr__(self) -> str:
        return f"JointState(16 DOF, ts={self.ts:.3f})"


@dataclass
class BatteryState:
    """Robot battery state."""

    percentage: int = 0
    voltage: float = 0.0
    current: float = 0.0
    temperature: List[int] = field(default_factory=lambda: [0, 0])
    status: int = 0
    cycle_count: int = 0
    ts: float = field(default_factory=time.time)

    # Status constants
    NORMAL: int = 0
    CHARGING: int = 1
    LOW: int = 2
    CRITICAL: int = 3

    @property
    def is_low(self) -> bool:
        return self.status >= self.LOW

    @property
    def is_critical(self) -> bool:
        return self.status >= self.CRITICAL

    def to_dict(self) -> Dict[str, Any]:
        return {
            "percentage": self.percentage,
            "voltage": self.voltage,
            "current": self.current,
            "temperature": list(self.temperature),
            "status": self.status,
            "cycle_count": self.cycle_count,
            "ts": self.ts,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> BatteryState:
        return cls(
            percentage=d.get("percentage", 0),
            voltage=d.get("voltage", 0.0),
            current=d.get("current", 0.0),
            temperature=d.get("temperature", [0, 0]),
            status=d.get("status", 0),
            cycle_count=d.get("cycle_count", 0),
            ts=d.get("ts", 0.0),
        )

    def __repr__(self) -> str:
        return (f"BatteryState({self.percentage}%, {self.voltage:.1f}V, "
                f"status={self.status})")


@dataclass
class FootForces:
    """4-leg ground reaction forces (FR, FL, RR, RL)."""

    forces: List[float] = field(default_factory=lambda: [0.0] * NUM_LEGS)
    ts: float = field(default_factory=time.time)

    @property
    def fr(self) -> float: return self.forces[0]
    @property
    def fl(self) -> float: return self.forces[1]
    @property
    def rr(self) -> float: return self.forces[2]
    @property
    def rl(self) -> float: return self.forces[3]

    @property
    def total(self) -> float:
        return sum(self.forces)

    @property
    def in_contact(self) -> List[bool]:
        """Which feet are in ground contact (force > threshold)."""
        return [f > 5.0 for f in self.forces]

    def to_dict(self) -> Dict[str, Any]:
        return {"forces": list(self.forces), "ts": self.ts}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> FootForces:
        return cls(
            forces=d.get("forces", [0.0] * NUM_LEGS),
            ts=d.get("ts", 0.0),
        )

    def __repr__(self) -> str:
        return (f"FootForces(FR={self.fr:.1f}, FL={self.fl:.1f}, "
                f"RR={self.rr:.1f}, RL={self.rl:.1f})")


@dataclass
class RobotState:
    """Composite robot state — joints + battery + foot forces + IMU.

    Matches the ROS2 interface/msg/RobotState used by the legacy bridge.
    """

    joints: JointState = field(default_factory=JointState)
    battery: BatteryState = field(default_factory=BatteryState)
    foot_forces: FootForces = field(default_factory=FootForces)
    imu: Imu = field(default_factory=Imu)
    ts: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "joints": self.joints.to_dict(),
            "battery": self.battery.to_dict(),
            "foot_forces": self.foot_forces.to_dict(),
            "imu": {
                "orientation": self.imu.orientation.to_dict(),
                "angular_velocity": self.imu.angular_velocity.to_dict(),
                "linear_acceleration": self.imu.linear_acceleration.to_dict(),
            },
            "ts": self.ts,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> RobotState:
        return cls(
            joints=JointState.from_dict(d.get("joints", {})),
            battery=BatteryState.from_dict(d.get("battery", {})),
            foot_forces=FootForces.from_dict(d.get("foot_forces", {})),
            ts=d.get("ts", 0.0),
        )

    def __repr__(self) -> str:
        contact = sum(self.foot_forces.in_contact)
        return (f"RobotState(battery={self.battery.percentage}%, "
                f"feet_contact={contact}/4, ts={self.ts:.3f})")
