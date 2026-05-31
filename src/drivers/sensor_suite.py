"""SensorSuite — typed sensor-mount registry with body-frame transforms.

Manages the set of sensors mounted on the robot. Each sensor is described by
its mount extrinsics (position + orientation) relative to the robot body
frame, plus optional metadata like frame_id and resolution.

Usage::

    from drivers.sensor_suite import SensorMount, SensorSuite, SensorType

    sensors = [
        SensorMount("lidar", SensorType.LIDAR, "livox_frame",
                    x=-0.011, y=-0.02329, z=0.04412),
        SensorMount("camera_front", SensorType.CAMERA, "camera_frame",
                    x=0.15, y=0.0, z=0.45, roll=0.0, pitch=0.0, yaw=0.0),
        SensorMount("gnss", SensorType.GNSS, "gnss_frame",
                    x=0.0, y=0.0, z=0.45),
    ]
    suite = SensorSuite(sensors)

    T = suite.T_body_sensor("lidar")          # 4x4 body → lidar
    T_12 = suite.T_sensor1_sensor2("lidar", "camera_front")  # lidar → camera
    frames = suite.frame_ids()                # {"lidar": "livox_frame", ...}
    issues = suite.validate()                 # [] if valid
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from enum import Enum

# ── SensorType ────────────────────────────────────────────────────────────────


class SensorType(str, Enum):
    LIDAR = "lidar"
    CAMERA = "camera"
    DEPTH_CAMERA = "depth_camera"
    IMU = "imu"
    GNSS = "gnss"


# ── SensorMount ──────────────────────────────────────────────────────────────


@dataclass
class SensorMount:
    """A single sensor with its mount extrinsics relative to the robot body frame.

    Attributes:
        sensor_id:  Unique identifier (e.g. ``"lidar"``, ``"camera_front"``).
        sensor_type:  ``SensorType`` enum member.
        frame_id:  TF frame name for this sensor (e.g. ``"livox_frame"``).
        x, y, z:  Position offset from body origin in metres.
        roll, pitch, yaw:  Orientation (Euler XYZ, radians) relative to body.
        fps:  Nominal publish frequency (Hz), optional.
        resolution:  (width, height) in pixels, for cameras.
        model:  Sensor model string (e.g. ``"Livox MID-360"``).
        extra:  Any additional sensor-specific metadata.
    """

    sensor_id: str
    sensor_type: SensorType
    frame_id: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    fps: float | None = None
    resolution: tuple[int, int] | None = None
    model: str | None = None
    extra: dict[str, Any] = field(default_factory=dict)


# ── SensorSuite ──────────────────────────────────────────────────────────────


class SensorSuite:
    """Typed registry of robot-mounted sensors with body-frame transform methods.

    Each sensor's mount extrinsics (position + euler angles) define the
    rigid transform **from the robot body frame to the sensor frame**:

        T_body_sensor = [R(rpy) | t]
                        [   0    | 1]

    where ``t = [x, y, z]`` is the sensor origin in body coordinates and
    ``R`` is the rotation matrix from ``roll, pitch, yaw`` (XYZ extrinsic Euler).
    """

    def __init__(self, sensors: list[SensorMount]):
        """Initialise the suite with an ordered list of mounted sensors.

        Args:
            sensors: Iterable of ``SensorMount`` instances describing all
                     sensors on the robot.
        """
        self._sensors = sensors
        self._by_id: dict[str, SensorMount] = {}
        for s in self._sensors:
            # Last occurrence wins on duplicate keys (validate() flags duplicates).
            self._by_id[s.sensor_id] = s

    # ── Accessors ──────────────────────────────────────────────────────────

    def get(self, sensor_id: str) -> SensorMount:
        """Look up a sensor by its unique identifier.

        Raises ``KeyError`` if the sensor is not registered.
        """
        if sensor_id not in self._by_id:
            raise KeyError(
                f"sensor '{sensor_id}' not found in SensorSuite "
                f"(available: {sorted(self._by_id)})"
            )
        return self._by_id[sensor_id]

    def by_type(self, sensor_type: SensorType) -> list[SensorMount]:
        """Return all sensors of a given type."""
        return [s for s in self._sensors if s.sensor_type == sensor_type]

    @property
    def sensors(self) -> list[SensorMount]:
        """Ordered list of all registered sensors (read-only)."""
        return list(self._sensors)

    @property
    def count(self) -> int:
        """Number of registered sensors."""
        return len(self._sensors)

    # ── Body-frame transforms ──────────────────────────────────────────────

    def T_body_sensor(self, sensor_id: str) -> np.ndarray:
        """Return the 4x4 homogeneous transform **body → sensor**.

        This is a rigid-body transform built from the sensor's mount extrinsics
        (position ``x, y, z`` and Euler XYZ ``roll, pitch, yaw``).

        Args:
            sensor_id:  The sensor identifier.

        Returns:
            ``np.ndarray`` with shape ``(4, 4)``.
        """
        mount = self.get(sensor_id)
        return self._extrinsics_to_matrix(mount)

    def T_sensor1_sensor2(self, id1: str, id2: str) -> np.ndarray:
        """Return the 4x4 homogeneous transform **sensor1 → sensor2**.

        Computed as::

            T_s1_s2 = inv(T_body_s1) @ T_body_s2

        Args:
            id1:  Source sensor identifier.
            id2:  Target sensor identifier.

        Returns:
            ``np.ndarray`` with shape ``(4, 4)``.
        """
        T_b_s1 = self.T_body_sensor(id1)
        T_b_s2 = self.T_body_sensor(id2)
        return np.linalg.inv(T_b_s1) @ T_b_s2

    # ── Frame helpers ──────────────────────────────────────────────────────

    def frame_ids(self) -> dict[str, str]:
        """Return ``{sensor_id: frame_id, ...}`` for all sensors."""
        return {s.sensor_id: s.frame_id for s in self._sensors}

    # ── Validation ─────────────────────────────────────────────────────────

    def validate(self) -> list[str]:
        """Run structural validation and return a list of issues.

        Checks performed:

        * Empty ``sensor_id``.
        * Duplicate ``sensor_id`` values.
        * Empty ``frame_id``.

        An empty list means the suite is well-formed.
        """
        issues: list[str] = []
        seen_ids: set[str] = set()
        for s in self._sensors:
            if not s.sensor_id:
                issues.append("sensor entry has empty sensor_id")
                continue
            if s.sensor_id in seen_ids:
                issues.append(f"duplicate sensor_id '{s.sensor_id}'")
            seen_ids.add(s.sensor_id)
            if not s.frame_id:
                issues.append(f"sensor '{s.sensor_id}' has empty frame_id")
        return issues

    # ── Internal helpers ───────────────────────────────────────────────────

    @staticmethod
    def _extrinsics_to_matrix(mount: SensorMount) -> np.ndarray:
        """Build a 4x4 homogeneous **body → sensor** transform.

        Given a sensor mounted at position ``t = [x, y, z]`` (in body frame)
        with rotation ``R`` (roll-pitch-yaw), a point in body frame is
        mapped to sensor frame as::

            p_sensor = R @ (p_body - t) = R @ p_body - R @ t

        The 4x4 matrix is therefore ``[R | -R@t; 0 | 1]``.
        """
        roll, pitch, yaw = mount.roll, mount.pitch, mount.yaw
        rot = _euler_to_rot(roll, pitch, yaw)
        t = np.array([mount.x, mount.y, mount.z], dtype=np.float64)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = rot
        T[:3, 3] = -rot @ t
        return T


# ── Euler-angle helpers ──────────────────────────────────────────────────────


def _euler_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build rotation matrix from roll-pitch-yaw (XYZ intrinsic / ZYX extrinsic) Euler angles (radians). Pure numpy."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    return np.array([
        [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
        [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr],
        [-sp,      cp * sr,                  cp * cr],
    ])


# ── Build from robot_config.yaml ─────────────────────────────────────────────


def sensor_suite_from_config(config_path: str | None = None) -> SensorSuite:
    """Build a ``SensorSuite`` from ``robot_config.yaml``.

    Reads the ``lidar``, ``camera``, and ``gnss`` sections and creates
    ``SensorMount`` entries for each.

    Args:
        config_path:  Path to robot_config.yaml.  ``None`` = resolve via
                      ``core.config.get_config()``.

    Returns:
        A ``SensorSuite`` populated from the config file.
    """
    if config_path is not None:
        import yaml
        with open(config_path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
    else:
        from core.config import get_config
        cfg = get_config()

    sensors: list[SensorMount] = []

    # ── LiDAR ──────────────────────────────────────────────────────────────
    if "lidar" in cfg:
        lid = cfg["lidar"]
        sensors.append(SensorMount(
            sensor_id="lidar",
            sensor_type=SensorType.LIDAR,
            frame_id=lid.get("frame_id", "livox_frame"),
            x=lid.get("offset_x", 0.0),
            y=lid.get("offset_y", 0.0),
            z=lid.get("offset_z", 0.0),
            roll=lid.get("roll", 0.0),
            pitch=lid.get("pitch", 0.0),
            yaw=lid.get("yaw", 0.0),
            fps=lid.get("publish_freq"),
            model=lid.get("model"),
        ))

    # ── Camera ─────────────────────────────────────────────────────────────
    if "camera" in cfg:
        cam = cfg["camera"]
        sensors.append(SensorMount(
            sensor_id="camera_front",
            sensor_type=SensorType.CAMERA,
            frame_id=cam.get("frame_id", "camera_frame"),
            x=cam.get("position_x", 0.0),
            y=cam.get("position_y", 0.0),
            z=cam.get("position_z", 0.0),
            roll=cam.get("roll", 0.0),
            pitch=cam.get("pitch", 0.0),
            yaw=cam.get("yaw", 0.0),
            resolution=(cam.get("width", 640), cam.get("height", 480)),
            fps=cam.get("fps"),
            model=cam.get("model"),
        ))

    # ── GNSS ───────────────────────────────────────────────────────────────
    if "gnss" in cfg:
        gnss = cfg["gnss"]
        ant = gnss.get("antenna_offset", {})
        sensors.append(SensorMount(
            sensor_id="gnss",
            sensor_type=SensorType.GNSS,
            frame_id="gnss_frame",
            x=ant.get("x", 0.0),
            y=ant.get("y", 0.0),
            z=ant.get("z", 0.0),
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            fps=gnss.get("fps", 50.0),  # WTRTK-980 default rate
            model=gnss.get("model", "WTRTK-980"),
        ))

    return SensorSuite(sensors)


# ── Convenience instance ─────────────────────────────────────────────────────


def default_sensor_suite() -> SensorSuite:
    """Return the ``SensorSuite`` loaded from the project default config.

    Shorthand for ``sensor_suite_from_config(None)``.
    """
    return sensor_suite_from_config(None)
