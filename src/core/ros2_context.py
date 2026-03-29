"""Shared rclpy context — one executor for all ROS2 nodes.

Prevents multiple nodes from each running their own executor,
which causes DDS callback starvation on ROS2 Humble.

Usage:
    from core.ros2_context import get_shared_executor, ensure_rclpy

    ensure_rclpy()
    node = Node("my_node")
    get_shared_executor().add_node(node)
    # Executor spin thread is auto-started on first add_node
"""

import logging
import threading
from typing import Optional

logger = logging.getLogger(__name__)

_lock = threading.Lock()
_executor = None
_spin_thread: Optional[threading.Thread] = None
_running = False


def ensure_rclpy() -> None:
    """Initialize rclpy if not already done."""
    try:
        import rclpy
        if not rclpy.ok():
            rclpy.init()
    except ImportError:
        pass


def get_shared_executor():
    """Get the singleton SingleThreadedExecutor. Auto-starts spin thread."""
    global _executor, _spin_thread, _running
    with _lock:
        if _executor is None:
            from rclpy.executors import SingleThreadedExecutor
            ensure_rclpy()
            _executor = SingleThreadedExecutor()
            _running = True
            _spin_thread = threading.Thread(
                target=_spin_loop, daemon=True, name="ros2_shared_spin")
            _spin_thread.start()
            logger.info("Shared ROS2 executor started")
    return _executor


def shutdown_shared_executor() -> None:
    """Shutdown the shared executor (call on system stop)."""
    global _executor, _spin_thread, _running
    with _lock:
        _running = False
        if _executor:
            _executor.shutdown()
            _executor = None
        if _spin_thread:
            _spin_thread.join(timeout=3)
            _spin_thread = None


def _spin_loop() -> None:
    import rclpy
    try:
        while _running and rclpy.ok():
            _executor.spin_once(timeout_sec=0.001)
    except Exception:
        pass
