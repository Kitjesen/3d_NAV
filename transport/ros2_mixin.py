"""
ROS2 Transport Mixin — 让任意 ROS2 Node 透明使用多传输后端

用法:
    class MyNode(TransportMixin, Node):
        def __init__(self):
            super().__init__('my_node')
            self.init_transport()

            # 高带宽话题 (图像/点云) — 可走 SHM/DUAL
            self._img_pub = self.create_fast_publisher(
                '/camera/color/image_raw', buffer_size=4*1024*1024
            )
            self._img_pub.publish(rgb_bytes)

            # 低带宽话题照常用 ROS2 DDS
            self._status_pub = self.create_publisher(String, '/status', 10)

设计原则:
  - 默认 DDS (完全兼容现有代码)
  - transport_strategy 参数控制高带宽话题传输方式
  - SHM 路径对 bytes/numpy 直接写共享内存 (~200μs vs DDS ~1300μs)
  - DUAL 模式同时写 SHM + DDS (低延迟 + RViz/ros2 topic echo 兼容)
"""

import logging
from typing import Any, Callable, Dict, Optional

from .core import Publisher, Subscriber, TopicConfig, Transport, TransportStrategy
from .factory import create_transport

logger = logging.getLogger(__name__)


class TransportMixin:
    """ROS2 Node 的传输层 Mixin。

    继承此类后，Node 获得:
      - init_transport()        初始化传输层 (读取 transport_strategy 参数)
      - create_fast_publisher() 创建高速发布者 (SHM/DUAL/DDS)
      - create_fast_subscriber() 创建高速订阅者
      - shutdown_transport()    清理传输资源
    """

    def init_transport(self, default_strategy: str = "dds") -> None:
        """初始化传输层。须在 Node.__init__() 之后调用。

        Args:
            default_strategy: 默认传输策略 (dds/shm/dual/auto)
        """
        # 声明 ROS2 参数
        self.declare_parameter("transport_strategy", default_strategy)
        strategy_str = self.get_parameter("transport_strategy").value

        try:
            self._transport_strategy = TransportStrategy(strategy_str)
        except ValueError:
            self.get_logger().warning(
                f"Unknown transport strategy '{strategy_str}', falling back to DDS"
            )
            self._transport_strategy = TransportStrategy.DDS

        # 创建传输实例
        # DDS 需要 ros_node 参数 (即 self)
        ros_node = self if self._transport_strategy in (
            TransportStrategy.DDS, TransportStrategy.DUAL
        ) else None

        try:
            self._transport: Transport = create_transport(
                self._transport_strategy, ros_node=ros_node
            )
        except Exception as e:
            self.get_logger().warning(
                f"Failed to create {strategy_str} transport: {e}, falling back to DDS"
            )
            self._transport_strategy = TransportStrategy.DDS
            self._transport = create_transport(TransportStrategy.DDS, ros_node=self)

        # 跟踪已创建的发布者/订阅者
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
        """创建高速发布者。

        SHM/DUAL 模式下走共享内存快速路径。
        DDS 模式下等价于普通 ROS2 publisher。

        Args:
            topic_name: ROS2 话题名
            msg_type: ROS2 消息类型 (DDS/DUAL 模式需要)
            buffer_size: SHM 缓冲区大小 (默认 4MB，足够 1080p RGB)
            qos_depth: QoS 队列深度
            reliable: 是否使用 RELIABLE QoS

        Returns:
            Publisher 实例 (调用 .publish(msg) 发布)
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
        """创建高速订阅者。

        Args:
            topic_name: ROS2 话题名
            callback: 消息回调 (SHM 模式收到 bytes，DDS 模式收到 ROS2 msg)
            msg_type: ROS2 消息类型
            buffer_size: SHM 缓冲区大小
            qos_depth: QoS 队列深度
            reliable: 是否使用 RELIABLE QoS

        Returns:
            Subscriber 实例
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
        """当前传输后端名称。"""
        return getattr(self, '_transport', None) and self._transport.name or "uninitialized"

    def shutdown_transport(self) -> None:
        """清理传输资源。在 Node.destroy_node() 前调用。"""
        if hasattr(self, '_transport') and self._transport is not None:
            self._transport.close()
            self._transport_pubs.clear()
            self._transport_subs.clear()
            self.get_logger().info("Transport shut down")
