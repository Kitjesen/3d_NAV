"""Livox CustomMsg DDS IDL types for cyclonedds.

Mirrors the livox_ros_driver2 message layout exactly so Python can
subscribe to /lidar/scan without rclpy.

NOTE: No `from __future__ import annotations` — cyclonedds IdlStruct
requires real type objects at class-definition time.
"""

from dataclasses import dataclass

try:
    from cyclonedds.idl import IdlStruct, types

    # Reuse header from core.dds if available, else define inline.
    try:
        from core.dds import DDS_Header
    except ImportError:
        @dataclass
        class _DDS_Time(IdlStruct):
            sec: types.int32
            nanosec: types.uint32

        @dataclass
        class DDS_Header(IdlStruct):
            stamp: _DDS_Time
            frame_id: str

    @dataclass
    class LivoxPoint(IdlStruct):
        """Single point in a Livox CustomMsg frame."""
        offset_time: types.uint32    # ns offset from timebase
        x: types.float32
        y: types.float32
        z: types.float32
        reflectivity: types.uint8
        tag: types.uint8             # return type flags
        line: types.uint8            # scan line index

    @dataclass
    class LivoxCustomMsg(
        IdlStruct,
        typename="livox_ros_driver2::msg::dds_::CustomMsg_",
    ):
        """livox_ros_driver2/CustomMsg — one full LiDAR scan."""
        header: DDS_Header
        timebase: types.uint64       # absolute ns timestamp of first point
        point_num: types.uint32
        lidar_id: types.uint8
        rsvd: types.array[types.uint8, 3]
        points: types.sequence[LivoxPoint]

    HAS_LIVOX_IDL = True

except ImportError:
    HAS_LIVOX_IDL = False
    LivoxCustomMsg = None
    LivoxPoint = None
