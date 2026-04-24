"""TAREExplorerModule — bridges the TARE ROS2 node (CMU) into LingTu.

Architecture
------------
The TARE planner runs as a separate C++ ROS2 node (started via
``exploration.native_factories.tare_explorer``). This module subscribes to
its DDS output and re-publishes it through the framework's In/Out ports so
the rest of LingTu stays unaware a third-party node exists.

Port contract
-------------
Output port ``exploration_goal: Out[PoseStamped]`` matches
``WavefrontFrontierExplorer`` exactly — letting ``autoconnect`` wire it
straight into ``NavigationModule.goal_pose`` without manual wires, and
letting you hot-swap the exploration backend without other modules noticing.

DDS topic contract (after the TARE remappings in ``native_factories``):
    /exploration/way_point      PointStamped  — next goal (primary output)
    /exploration/path           Path          — full exploration path
    /exploration/runtime        Float32       — per-cycle latency
    /exploration/finish         Bool          — exploration done

Start control: we publish a bool to ``/exploration/start`` when
``start()`` is called, telling TARE to begin planning.
"""

from __future__ import annotations

import logging
import threading
import time as _time
from typing import Any

from core.module import Module, skill
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from core.registry import register
from core.stream import In, Out

logger = logging.getLogger(__name__)


@register("exploration", "tare", description="CMU TARE hierarchical exploration bridge")
class TAREExplorerModule(Module, layer=5):
    """DDS → Module bridge for the TARE planner.

    Outputs mirror ``WavefrontFrontierExplorer`` so this Module is a
    drop-in replacement under ``autoconnect``. Keeps the existing
    ``exploration_goal: Out[PoseStamped]`` contract, and adds per-TARE
    diagnostics (``exploration_path``, ``tare_stats``).
    """

    exploration_goal: Out[PoseStamped]     # → NavigationModule.goal_pose
    exploration_path: Out[list]            # for visualization
    exploring:        Out[bool]            # activity indicator
    tare_stats:       Out[dict]            # per-cycle runtime + health
    alive:            Out[bool]

    def __init__(
        self,
        way_point_topic:  str = "/exploration/way_point",
        path_topic:       str = "/exploration/path",
        runtime_topic:    str = "/exploration/runtime",
        finish_topic:     str = "/exploration/finish",
        start_topic:      str = "/exploration/start",
        way_point_timeout_s: float = 15.0,
        auto_start: bool = True,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._way_point_topic = way_point_topic
        self._path_topic = path_topic
        self._runtime_topic = runtime_topic
        self._finish_topic = finish_topic
        self._start_topic = start_topic
        self._way_point_timeout_s = way_point_timeout_s
        self._auto_start = auto_start

        self._reader = None
        self._rclpy_node = None
        self._publisher = None

        # Runtime state
        self._last_waypoint_ts: float = 0.0
        self._last_runtime_ms: float = 0.0
        self._last_finish: bool = False
        self._waypoint_count: int = 0
        self._started_exploration: bool = False
        self._shutdown = threading.Event()
        self._watchdog_thread: threading.Thread | None = None

    # ── lifecycle ────────────────────────────────────────────────────────

    def preflight(self) -> str | None:
        """Verify that at least one DDS backend is reachable before the system starts."""
        has_cyclonedds = False
        has_rclpy = False
        try:
            import cyclonedds  # noqa: F401
            has_cyclonedds = True
        except ImportError:
            pass
        try:
            import rclpy  # noqa: F401
            has_rclpy = True
        except ImportError:
            pass
        if not has_cyclonedds and not has_rclpy:
            return (
                "TAREExplorerModule requires cyclonedds or rclpy, but neither is installed. "
                "Install cyclonedds-python or source a ROS2 workspace."
            )
        return None

    def setup(self) -> None:
        if self._try_cyclonedds():
            return
        if self._try_rclpy():
            return
        # preflight() guarantees at least one package is importable, so reaching
        # here means runtime init failed despite the package being present.
        logger.warning(
            "TAREExplorerModule: DDS backend import succeeded but init failed; "
            "no waypoints will be produced."
        )

    def start(self) -> None:
        super().start()
        self.alive.publish(self._reader is not None or self._rclpy_node is not None)
        self._shutdown.clear()
        # Watchdog: detect TARE silence and publish health state
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True, name="tare-watchdog")
        self._watchdog_thread.start()
        if self._auto_start:
            self._publish_start_signal(True)
            self._started_exploration = True
        self.exploring.publish(self._started_exploration)

    def stop(self) -> None:
        self._shutdown.set()
        if self._watchdog_thread and self._watchdog_thread.is_alive():
            self._watchdog_thread.join(timeout=2.0)
        # Signal TARE to stop before we tear down
        try:
            self._publish_start_signal(False)
        except Exception:
            pass
        if self._reader is not None:
            try:
                self._reader.stop()
            except Exception:
                pass
            self._reader = None
        if self._rclpy_node is not None:
            try:
                self._rclpy_node.destroy_node()
            except Exception:
                pass
            self._rclpy_node = None
        super().stop()

    # ── DDS transports ───────────────────────────────────────────────────

    def _try_cyclonedds(self) -> bool:
        """Preferred path: lightweight cyclonedds subscribe + publish."""
        try:
            from core.dds import DDSReader
            # Local DDS IDL types needed for TARE's topic set.
            dds_types = self._load_tare_dds_types()
            if dds_types is None:
                return False
            (DDS_PointStamped, DDS_Path, DDS_Float32, DDS_Bool) = dds_types

            reader = DDSReader(domain_id=0)
            reader.subscribe(self._way_point_topic, DDS_PointStamped,
                              self._on_dds_waypoint)
            reader.subscribe(self._path_topic, DDS_Path, self._on_dds_path)
            reader.subscribe(self._runtime_topic, DDS_Float32,
                              self._on_dds_runtime)
            reader.subscribe(self._finish_topic, DDS_Bool,
                              self._on_dds_finish)
            if not reader.start():
                return False
            reader.spin_background()
            self._reader = reader
            # We reuse the reader's internal DDS participant for publishing
            # the start signal; falling back to rclpy publish otherwise.
            try:
                from cyclonedds.domain import DomainParticipant
                from cyclonedds.pub import DataWriter, Publisher
                from cyclonedds.topic import Topic
                dp = DomainParticipant(0)
                self._cyclonedds_publisher = DataWriter(
                    Publisher(dp),
                    Topic(dp, self._start_topic, DDS_Bool),
                )
                self._cyclonedds_bool_type = DDS_Bool
            except Exception as e:
                logger.debug("TARE: cyclonedds publisher init failed: %s", e)
                self._cyclonedds_publisher = None
            logger.info("TAREExplorerModule: using cyclonedds")
            return True
        except ImportError:
            return False
        except Exception as e:
            logger.warning("TAREExplorerModule: cyclonedds init failed: %s", e)
            return False

    def _try_rclpy(self) -> bool:
        """Fallback: rclpy subscribe + publish via shared executor."""
        try:
            from geometry_msgs.msg import PointStamped as ROS2PointStamped
            from nav_msgs.msg import Path as ROS2Path
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from std_msgs.msg import Bool as ROS2Bool
            from std_msgs.msg import Float32 as ROS2Float32

            from core.ros2_context import ensure_rclpy, get_shared_executor

            ensure_rclpy()
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
            node = Node("tare_bridge")
            get_shared_executor().add_node(node)
            node.create_subscription(
                ROS2PointStamped, self._way_point_topic,
                self._on_rclpy_waypoint, qos)
            node.create_subscription(
                ROS2Path, self._path_topic,
                self._on_rclpy_path, qos)
            node.create_subscription(
                ROS2Float32, self._runtime_topic,
                self._on_rclpy_runtime, qos)
            node.create_subscription(
                ROS2Bool, self._finish_topic,
                self._on_rclpy_finish, qos)
            self._publisher = node.create_publisher(
                ROS2Bool, self._start_topic, qos)
            self._rclpy_bool_type = ROS2Bool
            self._rclpy_node = node
            logger.info("TAREExplorerModule: using rclpy fallback")
            return True
        except (ImportError, Exception) as e:
            logger.debug("TAREExplorerModule: rclpy unavailable: %s", e)
            return False

    @staticmethod
    def _load_tare_dds_types():
        """Build the four TARE-specific DDS IDL types we need. Returns None
        when cyclonedds isn't installed (stub mode)."""
        try:
            from dataclasses import dataclass as dds_dc

            from cyclonedds.idl import IdlStruct, types
        except ImportError:
            return None

        @dds_dc
        class DDS_Time(IdlStruct):
            sec: types.int32
            nanosec: types.uint32

        @dds_dc
        class DDS_Header(IdlStruct):
            stamp: DDS_Time
            frame_id: str

        @dds_dc
        class DDS_Point(IdlStruct):
            x: types.float64
            y: types.float64
            z: types.float64

        @dds_dc
        class DDS_Quaternion(IdlStruct):
            x: types.float64
            y: types.float64
            z: types.float64
            w: types.float64

        @dds_dc
        class DDS_Pose(IdlStruct):
            position: DDS_Point
            orientation: DDS_Quaternion

        @dds_dc
        class DDS_PoseStamped(IdlStruct):
            header: DDS_Header
            pose: DDS_Pose

        @dds_dc
        class DDS_PointStamped(
            IdlStruct, typename="geometry_msgs::msg::dds_::PointStamped_"
        ):
            header: DDS_Header
            point: DDS_Point

        @dds_dc
        class DDS_Path(
            IdlStruct, typename="nav_msgs::msg::dds_::Path_"
        ):
            header: DDS_Header
            poses: types.sequence[DDS_PoseStamped]

        @dds_dc
        class DDS_Float32(
            IdlStruct, typename="std_msgs::msg::dds_::Float32_"
        ):
            data: types.float32

        @dds_dc
        class DDS_Bool(IdlStruct, typename="std_msgs::msg::dds_::Bool_"):
            data: bool

        return (DDS_PointStamped, DDS_Path, DDS_Float32, DDS_Bool)

    # ── Callbacks: cyclonedds path ───────────────────────────────────────

    def _on_dds_waypoint(self, msg) -> None:
        try:
            p = msg.point
            frame = msg.header.frame_id or "map"
            self._emit_waypoint(p.x, p.y, p.z, frame_id=frame)
        except Exception as e:
            logger.debug("TARE dds waypoint error: %s", e)

    def _on_dds_path(self, msg) -> None:
        try:
            pts = [
                {"x": float(ps.pose.position.x),
                 "y": float(ps.pose.position.y),
                 "z": float(ps.pose.position.z)}
                for ps in msg.poses
            ]
            self.exploration_path.publish(pts)
        except Exception as e:
            logger.debug("TARE dds path error: %s", e)

    def _on_dds_runtime(self, msg) -> None:
        try:
            self._last_runtime_ms = float(msg.data)
        except Exception:
            pass

    def _on_dds_finish(self, msg) -> None:
        try:
            self._last_finish = bool(msg.data)
        except Exception:
            pass

    # ── Callbacks: rclpy path ────────────────────────────────────────────

    def _on_rclpy_waypoint(self, msg) -> None:
        try:
            frame = msg.header.frame_id or "map"
            self._emit_waypoint(float(msg.point.x), float(msg.point.y),
                                  float(msg.point.z), frame_id=frame)
        except Exception as e:
            logger.warning("TARE rclpy waypoint error: %s", e)

    def _on_rclpy_path(self, msg) -> None:
        try:
            pts = [
                {"x": float(ps.pose.position.x),
                 "y": float(ps.pose.position.y),
                 "z": float(ps.pose.position.z)}
                for ps in msg.poses
            ]
            self.exploration_path.publish(pts)
        except Exception as e:
            logger.debug("TARE rclpy path error: %s", e)

    def _on_rclpy_runtime(self, msg) -> None:
        self._last_runtime_ms = float(msg.data)

    def _on_rclpy_finish(self, msg) -> None:
        self._last_finish = bool(msg.data)

    # ── Emit ──────────────────────────────────────────────────────────────

    def _emit_waypoint(self, x: float, y: float, z: float,
                        frame_id: str = "map") -> None:
        """Convert a TARE PointStamped to LingTu PoseStamped and publish."""
        self._last_waypoint_ts = _time.time()
        self._waypoint_count += 1
        pose = PoseStamped(
            pose=Pose(
                position=Vector3(x=x, y=y, z=z),
                # TARE only provides a point; yaw decided downstream from
                # travel direction / path tangent. Identity here is fine
                # because NavigationModule derives heading from path delta.
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            frame_id=frame_id,
        )
        self.exploration_goal.publish(pose)

    def _publish_start_signal(self, enable: bool) -> None:
        """Send the bool start/stop signal to the TARE node."""
        try:
            if self._reader is not None and getattr(self, "_cyclonedds_publisher", None):
                msg = self._cyclonedds_bool_type(data=bool(enable))
                self._cyclonedds_publisher.write(msg)
                return
            if self._publisher is not None:
                msg = self._rclpy_bool_type()
                msg.data = bool(enable)
                self._publisher.publish(msg)
        except Exception as e:
            logger.debug("TARE start signal publish failed: %s", e)

    # ── Watchdog / diagnostics ───────────────────────────────────────────

    def _watchdog_loop(self) -> None:
        while not self._shutdown.wait(1.0):
            self._publish_stats()

    def _publish_stats(self) -> None:
        now = _time.time()
        wp_age = (now - self._last_waypoint_ts
                  if self._last_waypoint_ts > 0 else float("inf"))
        healthy = (wp_age < self._way_point_timeout_s
                   and (self._reader is not None or self._rclpy_node is not None))
        self.tare_stats.publish({
            "alive": self._reader is not None or self._rclpy_node is not None,
            "started": self._started_exploration,
            "healthy": healthy,
            "waypoint_count": self._waypoint_count,
            "waypoint_age_s": wp_age,
            "last_runtime_ms": self._last_runtime_ms,
            "finished": self._last_finish,
        })

    # ── Skills ───────────────────────────────────────────────────────────

    @skill
    def start_tare_exploration(self) -> str:
        """Send the start signal to the TARE planner. Use this to resume
        exploration after calling ``stop_tare_exploration``. Re-issues are
        idempotent."""
        import json
        self._publish_start_signal(True)
        self._started_exploration = True
        self.exploring.publish(True)
        return json.dumps({"status": "started"})

    @skill
    def stop_tare_exploration(self) -> str:
        """Pause TARE exploration. The planner keeps running but stops
        emitting new waypoints until ``start_tare_exploration`` is called."""
        import json
        self._publish_start_signal(False)
        self._started_exploration = False
        self.exploring.publish(False)
        return json.dumps({"status": "stopped"})

    @skill
    def get_tare_status(self) -> str:
        """Return TARE exploration state: waypoint count, last waypoint age,
        per-cycle runtime, and whether exploration is marked finished."""
        import json
        now = _time.time()
        return json.dumps({
            "alive": self._reader is not None or self._rclpy_node is not None,
            "started": self._started_exploration,
            "waypoint_count": self._waypoint_count,
            "waypoint_age_s": (now - self._last_waypoint_ts
                               if self._last_waypoint_ts > 0 else None),
            "last_runtime_ms": self._last_runtime_ms,
            "finished": self._last_finish,
        })
