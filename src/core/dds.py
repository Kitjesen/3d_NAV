"""Lightweight DDS layer — subscribes to ROS2 topics without rclpy.

Uses cyclonedds with IDL types that match ROS2 DDS type names exactly.
Zero ROS2 dependency: `pip install cyclonedds` is all you need.

NOTE: Do NOT add `from __future__ import annotations` — cyclonedds IdlStruct
needs real type objects at class definition time, not string annotations.

Usage::

    from core.dds import ROS2TopicReader

    reader = ROS2TopicReader()
    reader.on_odometry("/nav/odometry", lambda o: print(o.pose.pose.position.x))
    reader.on_pointcloud2("/nav/map_cloud", lambda pc: print(pc.width))
    reader.spin_background()
"""

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

logger = logging.getLogger(__name__)

# ── ROS2 message types as cyclonedds IDL structs ────────────────────────────
# typename must match ROS2 DDS type name exactly for subscription to work.

try:
    from cyclonedds.idl import IdlStruct, types
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.topic import Topic
    from cyclonedds.sub import DataReader
    from cyclonedds.qos import Qos, Policy
    from cyclonedds.util import duration

    _HAS_CYCLONEDDS = True

    # ── std_msgs ──

    @dataclass
    class DDS_Time(IdlStruct):
        sec: types.int32
        nanosec: types.uint32

    @dataclass
    class DDS_Header(IdlStruct):
        stamp: DDS_Time
        frame_id: str

    # ── geometry_msgs ──

    @dataclass
    class DDS_Point(IdlStruct):
        x: types.float64
        y: types.float64
        z: types.float64

    @dataclass
    class DDS_Quaternion(IdlStruct):
        x: types.float64
        y: types.float64
        z: types.float64
        w: types.float64

    @dataclass
    class DDS_Pose(IdlStruct):
        position: DDS_Point
        orientation: DDS_Quaternion

    @dataclass
    class DDS_PoseWithCovariance(IdlStruct):
        pose: DDS_Pose
        covariance: types.array[types.float64, 36]

    @dataclass
    class DDS_Vector3(IdlStruct):
        x: types.float64
        y: types.float64
        z: types.float64

    @dataclass
    class DDS_Twist(IdlStruct):
        linear: DDS_Vector3
        angular: DDS_Vector3

    @dataclass
    class DDS_TwistWithCovariance(IdlStruct):
        twist: DDS_Twist
        covariance: types.array[types.float64, 36]

    @dataclass
    class DDS_PoseStamped(IdlStruct, typename="geometry_msgs::msg::dds_::PoseStamped_"):
        header: DDS_Header
        pose: DDS_Pose

    # ── nav_msgs ──

    @dataclass
    class DDS_Odometry(IdlStruct, typename="nav_msgs::msg::dds_::Odometry_"):
        header: DDS_Header
        child_frame_id: str
        pose: DDS_PoseWithCovariance
        twist: DDS_TwistWithCovariance

    @dataclass
    class DDS_MapMetaData(IdlStruct):
        map_load_time: DDS_Time
        resolution: types.float32
        width: types.uint32
        height: types.uint32
        origin: DDS_Pose

    @dataclass
    class DDS_OccupancyGrid(IdlStruct, typename="nav_msgs::msg::dds_::OccupancyGrid_"):
        header: DDS_Header
        info: DDS_MapMetaData
        data: types.sequence[types.int8]

    @dataclass
    class DDS_Path(IdlStruct, typename="nav_msgs::msg::dds_::Path_"):
        header: DDS_Header
        poses: types.sequence[DDS_PoseStamped]

    # ── sensor_msgs ──

    @dataclass
    class DDS_PointField(IdlStruct):
        name: str
        offset: types.uint32
        datatype: types.uint8
        count: types.uint32

    @dataclass
    class DDS_PointCloud2(IdlStruct, typename="sensor_msgs::msg::dds_::PointCloud2_"):
        header: DDS_Header
        height: types.uint32
        width: types.uint32
        fields: types.sequence[DDS_PointField]
        is_bigendian: bool
        point_step: types.uint32
        row_step: types.uint32
        data: types.sequence[types.uint8]
        is_dense: types.bool

    @dataclass
    class DDS_Image(IdlStruct, typename="sensor_msgs::msg::dds_::Image_"):
        header: DDS_Header
        height: types.uint32
        width: types.uint32
        encoding: str
        is_bigendian: types.uint8
        step: types.uint32
        data: types.sequence[types.uint8]

    # ── tf2_msgs ──

    @dataclass
    class DDS_TransformStamped_Translation(IdlStruct):
        x: types.float64
        y: types.float64
        z: types.float64

    @dataclass
    class DDS_Transform(IdlStruct):
        translation: DDS_TransformStamped_Translation
        rotation: DDS_Quaternion

    @dataclass
    class DDS_TransformStamped(IdlStruct):
        header: DDS_Header
        child_frame_id: str
        transform: DDS_Transform

    @dataclass
    class DDS_TFMessage(IdlStruct, typename="tf2_msgs::msg::dds_::TFMessage_"):
        transforms: types.sequence[DDS_TransformStamped]

except ImportError:
    _HAS_CYCLONEDDS = False


# ── DDSReader ────────────────────────────────────────────────────────────────

# Map of ROS2 message type shortnames → (DDS IDL class, DDS topic prefix)
_MSG_TYPES: dict[str, Any] = {}
if _HAS_CYCLONEDDS:
    _MSG_TYPES = {
        "odometry": DDS_Odometry,
        "pointcloud2": DDS_PointCloud2,
        "image": DDS_Image,
        "occupancygrid": DDS_OccupancyGrid,
        "path": DDS_Path,
        "tfmessage": DDS_TFMessage,
    }


class DDSReader:
    """Lightweight DDS reader for ROS2 topics — no rclpy needed."""

    def __init__(self, domain_id: int = 0):
        self._domain_id = domain_id
        self._subs: list[dict] = []
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._dp = None

    def subscribe(self, ros2_topic: str, dds_type, callback: Callable):
        """Register a typed subscription."""
        self._subs.append({
            "ros2_topic": ros2_topic,
            "dds_topic": "rt" + ros2_topic,
            "dds_type": dds_type,
            "callback": callback,
            "reader": None,
        })

    def start(self) -> bool:
        if not _HAS_CYCLONEDDS:
            logger.warning("DDSReader: cyclonedds not available")
            return False
        try:
            self._dp = DomainParticipant(domain_id=self._domain_id)
            qos = Qos(Policy.Reliability.Reliable(duration(seconds=1)))
            for sub in self._subs:
                try:
                    topic = Topic(self._dp, sub["dds_topic"], sub["dds_type"])
                    sub["reader"] = DataReader(self._dp, topic, qos=qos)
                    logger.info("DDSReader: %s → %s", sub["ros2_topic"], sub["dds_topic"])
                except Exception as e:
                    logger.warning("DDSReader: failed %s: %s", sub["ros2_topic"], e)
            self._running = True
            return True
        except Exception as e:
            logger.warning("DDSReader: start failed: %s", e)
            return False

    def spin_background(self) -> None:
        if not self._running:
            if not self.start():
                return
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None

    def _spin_loop(self) -> None:
        while self._running:
            for sub in self._subs:
                reader = sub.get("reader")
                if reader is None:
                    continue
                try:
                    samples = reader.take(N=10)
                    for sample in samples:
                        if sample is not None:
                            sub["callback"](sample)
                except Exception as e:
                    logger.debug("DDSReader poll %s: %s", sub["ros2_topic"], e)
            time.sleep(0.005)


class ROS2TopicReader(DDSReader):
    """Convenience reader with typed subscribe helpers."""

    def on_odometry(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_Odometry, callback)

    def on_pointcloud2(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_PointCloud2, callback)

    def on_image(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_Image, callback)

    def on_occupancy_grid(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_OccupancyGrid, callback)

    def on_path(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_Path, callback)

    def on_tf(self, topic: str, callback: Callable):
        if _HAS_CYCLONEDDS:
            self.subscribe(topic, DDS_TFMessage, callback)
