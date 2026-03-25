"""core.transport.ros2_mixin — ROS2 Transport Mixin.

Lets any ROS2 Node transparently use multiple transport backends.

Usage:
    class MyNode(TransportMixin, Node):
        def __init__(self):
            super().__init__('my_node')
            self.init_transport()

            # High-bandwidth topics (images/pointclouds) — can use SHM/DUAL
            self._img_pub = self.create_fast_publisher(
                '/camera/color/image_raw', buffer_size=4*1024*1024
            )
            self._img_pub.publish(rgb_bytes)

            # Low-bandwidth topics use normal ROS2 DDS
            self._status_pub = self.create_publisher(String, '/status', 10)

Design principles:
  - Default DDS (fully compatible with existing code)
  - transport_strategy parameter controls high-bandwidth topic transport
  - SHM path writes bytes/numpy directly to shared memory (~200us vs DDS ~1300us)
  - DUAL mode writes to both SHM + DDS (low latency + RViz/ros2 topic echo compatible)
"""

import logging
from typing import Any, Callable, Dict

from .abc import Publisher, Subscriber, TopicConfig, TransportABC, TransportStrategy
from .factory import create_transport

logger = logging.getLogger(__name__)


class TransportMixin:
    """Transport layer Mixin for ROS2 Nodes.

    After inheriting, the Node gains:
      - init_transport()         initialize transport (reads transport_strategy param)
      - create_fast_publisher()  create high-speed publisher (SHM/DUAL/DDS)
      - create_fast_subscriber() create high-speed subscriber
      - shutdown_transport()     clean up transport resources
    """

    def init_transport(self, default_strategy: str = "dds") -> None:
        """Initialize transport layer. Must be called after Node.__init__().

        Args:
            default_strategy: default transport strategy (dds/shm/dual/auto)
        """
        self.declare_parameter("transport_strategy", default_strategy)
        strategy_str = self.get_parameter("transport_strategy").value

        try:
            self._transport_strategy = TransportStrategy(strategy_str)
        except ValueError:
            self.get_logger().warning(
                f"Unknown transport strategy '{strategy_str}', falling back to DDS"
            )
            self._transport_strategy = TransportStrategy.DDS

        # DDS requires ros_node parameter (i.e. self)
        ros_node = self if self._transport_strategy in (
            TransportStrategy.DDS, TransportStrategy.DUAL
        ) else None

        try:
            self._transport: TransportABC = create_transport(
                self._transport_strategy, ros_node=ros_node
            )
        except Exception as e:
            self.get_logger().warning(
                f"Failed to create {strategy_str} transport: {e}, falling back to DDS"
            )
            self._transport_strategy = TransportStrategy.DDS
            self._transport = create_transport(TransportStrategy.DDS, ros_node=self)

        self._transport_pubs: Dict[str, Publisher] = {}
        self._transport_subs: Dict[str, Subscriber] = {}

        self.get_logger().info(
            f"Transport initialized: {self._transport.name}"
        )

    def create_fast_publisher(
        self,
        topic_name: str,
        msg_type: Any = None,
        buffer_size: int = 4 * 1024 * 1024,
        qos_depth: int = 10,
        reliable: bool = False,
    ) -> Publisher:
        """Create a high-speed publisher.

        In SHM/DUAL mode uses shared memory fast path.
        In DDS mode equivalent to a normal ROS2 publisher.

        Args:
            topic_name: ROS2 topic name
            msg_type:   ROS2 message type (required for DDS/DUAL mode)
            buffer_size: SHM buffer size (default 4MB, enough for 1080p RGB)
            qos_depth:  QoS queue depth
            reliable:   whether to use RELIABLE QoS

        Returns:
            Publisher instance (call .publish(msg) to publish)
        """
        topic = TopicConfig(
            name=topic_name,
            msg_type=msg_type,
            strategy=self._transport_strategy,
            buffer_size=buffer_size,
            qos_depth=qos_depth,
            reliable=reliable,
        )
        pub = self._transport.create_publisher(topic)
        self._transport_pubs[topic_name] = pub
        self.get_logger().debug(
            f"Fast publisher created: {topic_name} [{self._transport.name}]"
        )
        return pub

    def create_fast_subscriber(
        self,
        topic_name: str,
        callback: Callable,
        msg_type: Any = None,
        buffer_size: int = 4 * 1024 * 1024,
        qos_depth: int = 10,
        reliable: bool = False,
    ) -> Subscriber:
        """Create a high-speed subscriber.

        Args:
            topic_name: ROS2 topic name
            callback:   message callback (SHM mode receives bytes, DDS receives ROS2 msg)
            msg_type:   ROS2 message type
            buffer_size: SHM buffer size
            qos_depth:  QoS queue depth
            reliable:   whether to use RELIABLE QoS

        Returns:
            Subscriber instance
        """
        topic = TopicConfig(
            name=topic_name,
            msg_type=msg_type,
            strategy=self._transport_strategy,
            buffer_size=buffer_size,
            qos_depth=qos_depth,
            reliable=reliable,
        )
        sub = self._transport.create_subscriber(topic, callback)
        self._transport_subs[topic_name] = sub
        self.get_logger().debug(
            f"Fast subscriber created: {topic_name} [{self._transport.name}]"
        )
        return sub

    @property
    def transport_name(self) -> str:
        """Current transport backend name."""
        return getattr(self, '_transport', None) and self._transport.name or "uninitialized"

    def shutdown_transport(self) -> None:
        """Clean up transport resources. Call before Node.destroy_node()."""
        if hasattr(self, '_transport') and self._transport is not None:
            self._transport.close()
            self._transport_pubs.clear()
            self._transport_subs.clear()
            self.get_logger().info("Transport shut down")
