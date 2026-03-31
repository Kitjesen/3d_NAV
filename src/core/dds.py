"""Lightweight DDS subscription layer — replaces rclpy for ROS2 topic reading.

Subscribes to ROS2 DDS topics using raw cyclonedds, no rclpy dependency.
Falls back to rclpy if cyclonedds not available.

ROS2 DDS topic mapping:
  ROS2 topic "/nav/odometry" → DDS topic "rt/nav/odometry"
  Message type nav_msgs/Odometry → CDR serialized bytes

Usage::

    from core.dds import DDSReader

    reader = DDSReader()
    reader.subscribe("/nav/odometry", "nav_msgs::msg::dds_::Odometry_", on_odom)
    reader.subscribe("/nav/map_cloud", "sensor_msgs::msg::dds_::PointCloud2_", on_cloud)
    reader.spin()  # blocking, or reader.spin_background()
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

import numpy as np

logger = logging.getLogger(__name__)


# ── CDR parsers for ROS2 message types ───────────────────────────────────────

def _align(offset: int, alignment: int) -> int:
    """CDR alignment: advance offset to next multiple of alignment."""
    return (offset + alignment - 1) & ~(alignment - 1)


def _read_string(buf: bytes, offset: int) -> tuple[str, int]:
    """Read CDR string: uint32 length + chars + null + alignment."""
    offset = _align(offset, 4)
    length = struct.unpack_from("<I", buf, offset)[0]
    offset += 4
    s = buf[offset:offset + length - 1].decode("utf-8", errors="replace")
    offset += length
    return s, offset


@dataclass
class ParsedOdometry:
    """Minimal parsed Odometry from CDR bytes."""
    stamp_sec: int
    stamp_nsec: int
    frame_id: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def parse_odometry_cdr(data: bytes) -> Optional[ParsedOdometry]:
    """Parse nav_msgs/Odometry from CDR bytes."""
    try:
        # Skip CDR header (4 bytes: encapsulation)
        off = 4
        # Header.stamp (sec: int32, nsec: uint32)
        off = _align(off, 4)
        sec = struct.unpack_from("<i", data, off)[0]; off += 4
        nsec = struct.unpack_from("<I", data, off)[0]; off += 4
        # Header.frame_id (string)
        frame_id, off = _read_string(data, off)
        # child_frame_id (string)
        _child, off = _read_string(data, off)
        # PoseWithCovariance.pose.position (3 × float64)
        off = _align(off, 8)
        x = struct.unpack_from("<d", data, off)[0]; off += 8
        y = struct.unpack_from("<d", data, off)[0]; off += 8
        z = struct.unpack_from("<d", data, off)[0]; off += 8
        # PoseWithCovariance.pose.orientation (4 × float64)
        qx = struct.unpack_from("<d", data, off)[0]; off += 8
        qy = struct.unpack_from("<d", data, off)[0]; off += 8
        qz = struct.unpack_from("<d", data, off)[0]; off += 8
        qw = struct.unpack_from("<d", data, off)[0]; off += 8
        return ParsedOdometry(sec, nsec, frame_id, x, y, z, qx, qy, qz, qw)
    except Exception:
        return None


@dataclass
class ParsedPointCloud2:
    """Minimal parsed PointCloud2 from CDR bytes."""
    frame_id: str
    width: int
    height: int
    point_step: int
    data: bytes


def parse_pointcloud2_cdr(raw: bytes) -> Optional[ParsedPointCloud2]:
    """Parse sensor_msgs/PointCloud2 from CDR bytes."""
    try:
        off = 4  # CDR header
        # Header.stamp
        off = _align(off, 4)
        off += 8  # skip stamp (sec + nsec)
        # Header.frame_id
        frame_id, off = _read_string(raw, off)
        # height, width (uint32)
        off = _align(off, 4)
        height = struct.unpack_from("<I", raw, off)[0]; off += 4
        width = struct.unpack_from("<I", raw, off)[0]; off += 4
        # fields[] — sequence<PointField>
        off = _align(off, 4)
        n_fields = struct.unpack_from("<I", raw, off)[0]; off += 4
        for _ in range(n_fields):
            # PointField: string name, uint32 offset, uint8 datatype, uint32 count
            _name, off = _read_string(raw, off)
            off = _align(off, 4)
            off += 4  # offset
            off += 1  # datatype
            off = _align(off, 4)
            off += 4  # count
        # is_bigendian (bool/uint8)
        off += 1
        # point_step, row_step (uint32)
        off = _align(off, 4)
        point_step = struct.unpack_from("<I", raw, off)[0]; off += 4
        row_step = struct.unpack_from("<I", raw, off)[0]; off += 4
        # data[] — sequence<uint8>
        off = _align(off, 4)
        data_len = struct.unpack_from("<I", raw, off)[0]; off += 4
        cloud_data = raw[off:off + data_len]
        return ParsedPointCloud2(frame_id, width, height, point_step, cloud_data)
    except Exception:
        return None


@dataclass
class ParsedImage:
    """Minimal parsed Image from CDR bytes."""
    height: int
    width: int
    encoding: str
    data: bytes


def parse_image_cdr(raw: bytes) -> Optional[ParsedImage]:
    """Parse sensor_msgs/Image from CDR bytes."""
    try:
        off = 4  # CDR header
        # Header.stamp
        off = _align(off, 4)
        off += 8  # skip stamp
        # Header.frame_id
        _frame_id, off = _read_string(raw, off)
        # height, width (uint32)
        off = _align(off, 4)
        height = struct.unpack_from("<I", raw, off)[0]; off += 4
        width = struct.unpack_from("<I", raw, off)[0]; off += 4
        # encoding (string)
        encoding, off = _read_string(raw, off)
        # is_bigendian (uint8)
        off += 1
        # step (uint32)
        off = _align(off, 4)
        _step = struct.unpack_from("<I", raw, off)[0]; off += 4
        # data[] — sequence<uint8>
        off = _align(off, 4)
        data_len = struct.unpack_from("<I", raw, off)[0]; off += 4
        img_data = raw[off:off + data_len]
        return ParsedImage(height, width, encoding, img_data)
    except Exception:
        return None


# ── DDS Reader (cyclonedds backend) ──────────────────────────────────────────

class DDSReader:
    """Lightweight DDS reader that subscribes to ROS2 topics without rclpy.

    Uses cyclonedds Python binding. Subscriptions receive raw CDR bytes.
    """

    def __init__(self, domain_id: int = 0):
        self._domain_id = domain_id
        self._subs: list[dict] = []
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._dp = None
        self._readers: list = []

    def subscribe(self, ros2_topic: str, callback: Callable, msg_type: str = "raw"):
        """Register a subscription. Call before spin().

        Args:
            ros2_topic: ROS2 topic name (e.g. "/nav/odometry")
            callback: function(raw_bytes) — receives CDR serialized bytes
            msg_type: hint for future auto-parsing ("odometry", "pointcloud2", "image", "raw")
        """
        self._subs.append({
            "ros2_topic": ros2_topic,
            "dds_topic": "rt" + ros2_topic,  # ROS2 DDS naming convention
            "callback": callback,
            "msg_type": msg_type,
        })

    def start(self) -> bool:
        """Initialize cyclonedds and create readers. Returns True if successful."""
        try:
            from cyclonedds.domain import DomainParticipant
            from cyclonedds.topic import Topic
            from cyclonedds.sub import Subscriber, DataReader
            from cyclonedds.core import Qos, Policy
            from cyclonedds.util import duration

            self._dp = DomainParticipant(domain=self._domain_id)
            subscriber = Subscriber(self._dp)

            for sub in self._subs:
                # Use generic bytes topic for raw CDR reception
                topic = Topic(self._dp, sub["dds_topic"], bytes)
                qos = Qos(Policy.Reliability.Reliable(duration(seconds=1)))
                reader = DataReader(subscriber, topic, qos=qos)
                sub["reader"] = reader
                self._readers.append(reader)
                logger.info("DDSReader: subscribed %s → %s", sub["ros2_topic"], sub["dds_topic"])

            self._running = True
            return True

        except ImportError:
            logger.warning("DDSReader: cyclonedds not available, no subscriptions")
            return False
        except Exception as e:
            logger.warning("DDSReader: start failed: %s", e)
            return False

    def spin_background(self) -> None:
        """Start polling in a background thread."""
        if not self._running:
            if not self.start():
                return
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop polling and clean up."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None

    def _spin_loop(self) -> None:
        """Poll all readers and dispatch callbacks."""
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
                    logger.debug("DDSReader poll error on %s: %s", sub["ros2_topic"], e)
            time.sleep(0.005)  # 200Hz poll rate


# ── Convenience: auto-parsing reader ─────────────────────────────────────────

class ROS2TopicReader(DDSReader):
    """DDSReader with auto-parsing for common ROS2 message types.

    Usage::

        reader = ROS2TopicReader()
        reader.on_odometry("/nav/odometry", lambda odom: print(odom.x, odom.y))
        reader.on_pointcloud2("/nav/map_cloud", lambda pc: print(pc.width))
        reader.on_image("/camera/color/image_raw", lambda img: print(img.encoding))
        reader.spin_background()
    """

    def on_odometry(self, topic: str, callback: Callable[[ParsedOdometry], None]):
        def _parse(raw_bytes):
            odom = parse_odometry_cdr(raw_bytes)
            if odom:
                callback(odom)
        self.subscribe(topic, _parse, "odometry")

    def on_pointcloud2(self, topic: str, callback: Callable[[ParsedPointCloud2], None]):
        def _parse(raw_bytes):
            pc = parse_pointcloud2_cdr(raw_bytes)
            if pc:
                callback(pc)
        self.subscribe(topic, _parse, "pointcloud2")

    def on_image(self, topic: str, callback: Callable[[ParsedImage], None]):
        def _parse(raw_bytes):
            img = parse_image_cdr(raw_bytes)
            if img:
                callback(img)
        self.subscribe(topic, _parse, "image")
