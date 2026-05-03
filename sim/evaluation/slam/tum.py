"""TUM trajectory IO for SLAM simulation evaluation."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, sqrt
from pathlib import Path
from typing import Iterable, Optional


@dataclass(frozen=True)
class TumPose:
    """One pose sample in TUM RGB-D trajectory format."""

    timestamp: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    frame_id: str = "map"

    def __post_init__(self) -> None:
        values = (
            self.timestamp,
            self.x,
            self.y,
            self.z,
            self.qx,
            self.qy,
            self.qz,
            self.qw,
        )
        if not all(isfinite(v) for v in values):
            raise ValueError("TUM pose values must be finite")
        q_norm = sqrt(self.qx * self.qx + self.qy * self.qy + self.qz * self.qz + self.qw * self.qw)
        if q_norm <= 0.0:
            raise ValueError("TUM pose quaternion must be non-zero")
        if abs(q_norm - 1.0) > 1e-9:
            object.__setattr__(self, "qx", self.qx / q_norm)
            object.__setattr__(self, "qy", self.qy / q_norm)
            object.__setattr__(self, "qz", self.qz / q_norm)
            object.__setattr__(self, "qw", self.qw / q_norm)

    @classmethod
    def from_odometry(
        cls,
        odometry: object,
        *,
        timestamp: Optional[float] = None,
        frame_id: Optional[str] = None,
    ) -> "TumPose":
        """Create a TUM pose from a LingTu-style odometry message."""

        pose = getattr(odometry, "pose")
        position = getattr(pose, "position")
        orientation = getattr(pose, "orientation")
        ts = timestamp if timestamp is not None else getattr(odometry, "ts")
        fid = frame_id if frame_id is not None else getattr(odometry, "frame_id", "map")
        return cls(
            timestamp=float(ts),
            x=float(getattr(position, "x")),
            y=float(getattr(position, "y")),
            z=float(getattr(position, "z")),
            qx=float(getattr(orientation, "x")),
            qy=float(getattr(orientation, "y")),
            qz=float(getattr(orientation, "z")),
            qw=float(getattr(orientation, "w")),
            frame_id=str(fid),
        )

    def to_line(self, precision: int = 9) -> str:
        fmt = f"{{:.{precision}f}}"
        return " ".join(
            [
                fmt.format(self.timestamp),
                fmt.format(self.x),
                fmt.format(self.y),
                fmt.format(self.z),
                fmt.format(self.qx),
                fmt.format(self.qy),
                fmt.format(self.qz),
                fmt.format(self.qw),
            ]
        )


def parse_tum_line(line: str, *, line_number: Optional[int] = None) -> TumPose:
    """Parse one non-empty TUM trajectory line."""

    parts = line.strip().split()
    if len(parts) != 8:
        where = f" on line {line_number}" if line_number is not None else ""
        raise ValueError(f"expected 8 TUM fields{where}, got {len(parts)}")
    try:
        values = [float(part) for part in parts]
    except ValueError as exc:
        where = f" on line {line_number}" if line_number is not None else ""
        raise ValueError(f"invalid numeric TUM value{where}") from exc
    return TumPose(*values)


def read_tum_trajectory(path: str | Path) -> list[TumPose]:
    """Read a TUM trajectory file, ignoring blank lines and comments."""

    trajectory_path = Path(path)
    poses: list[TumPose] = []
    with trajectory_path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            poses.append(parse_tum_line(stripped, line_number=line_number))
    return poses


def write_tum_trajectory(path: str | Path, poses: Iterable[TumPose]) -> None:
    """Write a TUM trajectory file."""

    trajectory_path = Path(path)
    trajectory_path.parent.mkdir(parents=True, exist_ok=True)
    with trajectory_path.open("w", encoding="utf-8", newline="\n") as handle:
        for pose in poses:
            handle.write(pose.to_line())
            handle.write("\n")
