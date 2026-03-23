"""lingtu.core — 模块编排框架核心，直接在 src/ 内构建。

核心组件:
- msgs     — 统一消息类型 (Vector3, Odometry, SceneGraph, ...)
- stream   — 类型化数据流端口 (Out[T], In[T]) 与传输抽象 (Transport)
- module   — 自动端口扫描的 Module 基类
- blueprint — 声明式编排蓝图 (Blueprint, autoconnect, SystemHandle)
"""

from .stream import In, LocalTransport, Out, Transport
from .module import Module
from .blueprint import Blueprint, SystemHandle, autoconnect

__all__ = [
    # stream
    "Transport", "LocalTransport", "Out", "In",
    # module
    "Module",
    # blueprint
    "Blueprint", "SystemHandle", "autoconnect",
]
