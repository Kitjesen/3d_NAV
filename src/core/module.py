"""lingtu.core.module — dimos 风格 Module 基类，自动扫描 In/Out 端口。

使用方式::

    class PerceptionModule(Module):
        scene_graph: Out[SceneGraph]
        detections: Out[Detection3D]
        image: In[Image]

        def setup(self):
            self.image.subscribe(self._on_image)

        def _on_image(self, img: Image):
            # 处理图像 → 发布场景图
            self.scene_graph.publish(sg)

端口通过 type hints 自动发现和实例化。Module.__init__ 会扫描所有
In[T] / Out[T] 类型标注，创建对应端口对象并注册到 ports_in / ports_out。

生命周期: __init__ → setup() → start() → [运行中] → stop()
"""

from __future__ import annotations

import logging
import sys
from typing import Any, Dict, List, Optional, get_args, get_origin, get_type_hints

from .stream import In, Out

logger = logging.getLogger(__name__)


class Module:
    """模块基类 — LingTu 编排框架的核心构件。

    子类声明 ``In[T]`` / ``Out[T]`` 类型标注，基类自动扫描并实例化端口。
    支持 layer 标签用于依赖图层级校验。

    Attributes:
        _config: 构造时传入的配置字典
        _ports_in: 输入端口注册表 {name: In[T]}
        _ports_out: 输出端口注册表 {name: Out[T]}
        _running: 模块运行状态
        _layer: 所属层级 (L0-L6)，用于依赖校验
    """

    # 类级别层级标签，子类可覆盖
    _layer: Optional[int] = None

    # -- dimos 风格 __init_subclass__: 在类定义时设置 None 占位 -----------

    def __init_subclass__(cls, layer: Optional[int] = None, **kwargs: Any) -> None:
        """子类定义钩子 — 设置 In/Out 类级别占位属性。

        dimos 需要这个是因为 Dask Actor proxy 在类级别查找属性。
        我们保留此模式以确保 hasattr() 在类级别就能发现端口。
        """
        super().__init_subclass__(**kwargs)

        if layer is not None:
            cls._layer = layer

        # 收集 MRO 中所有模块的命名空间，确保前向引用能解析
        globalns: Dict[str, Any] = {}
        for c in reversed(cls.__mro__):
            mod = sys.modules.get(c.__module__)
            if mod:
                globalns.update(mod.__dict__)

        try:
            hints = get_type_hints(cls, globalns=globalns, include_extras=True)
        except (NameError, AttributeError, TypeError):
            hints = {}

        for name, ann in hints.items():
            origin = get_origin(ann)
            if origin in (In, Out):
                if not hasattr(cls, name) or getattr(cls, name) is None:
                    setattr(cls, name, None)

    # -- 实例初始化 -------------------------------------------------------

    def __init__(self, **config: Any) -> None:
        self._config = config
        self._ports_in: Dict[str, In[Any]] = {}
        self._ports_out: Dict[str, Out[Any]] = {}
        self._running = False
        self._closed = False
        self._closed_lock = __import__("threading").Lock()

        # 扫描类型标注，实例化端口
        globalns: Dict[str, Any] = {}
        for c in reversed(type(self).__mro__):
            mod = sys.modules.get(c.__module__)
            if mod:
                globalns.update(mod.__dict__)

        try:
            hints = get_type_hints(type(self), globalns=globalns, include_extras=True)
        except (NameError, AttributeError, TypeError):
            hints = {}

        for name, ann in hints.items():
            origin = get_origin(ann)
            if origin is Out:
                inner = get_args(ann)[0] if get_args(ann) else Any
                port = Out(name, inner)
                setattr(self, name, port)
                self._ports_out[name] = port
            elif origin is In:
                inner = get_args(ann)[0] if get_args(ann) else Any
                port = In(name, inner)
                setattr(self, name, port)
                self._ports_in[name] = port

    # -- 生命周期 ---------------------------------------------------------

    def setup(self) -> None:
        """配置阶段 — 注册订阅者、加载资源。子类覆盖。"""
        pass

    def start(self) -> None:
        """启动模块。"""
        self._running = True
        logger.debug("Module %s started", type(self).__name__)

    def stop(self) -> None:
        """Stop module. Idempotent — safe to call multiple times.

        Breaks reference cycles (Module → Out → callback → Module) so the
        garbage collector can reclaim the entire module graph after shutdown.
        """
        with self._closed_lock:
            if self._closed:
                return
            self._closed = True
        self._running = False
        self._cleanup_refs()
        logger.debug("Module %s stopped", type(self).__name__)

    def _cleanup_refs(self) -> None:
        """Break reference cycles by clearing all port callbacks and transports."""
        for port in self._ports_out.values():
            port._clear_callbacks()
        for port in self._ports_in.values():
            port._clear_subscriber()

    # -- 端口访问 ---------------------------------------------------------

    @property
    def ports_in(self) -> Dict[str, In[Any]]:
        """只读输入端口字典副本。"""
        return dict(self._ports_in)

    @property
    def ports_out(self) -> Dict[str, Out[Any]]:
        """只读输出端口字典副本。"""
        return dict(self._ports_out)

    @property
    def all_ports(self) -> Dict[str, Any]:
        """所有端口 (In + Out)。"""
        return {**self._ports_in, **self._ports_out}

    @property
    def running(self) -> bool:
        return self._running

    @property
    def layer(self) -> Optional[int]:
        return self._layer

    # -- Blueprint 工厂 ---------------------------------------------------

    @classmethod
    def blueprint(cls, **kwargs: Any) -> "Blueprint":
        """创建仅包含本模块的 Blueprint。"""
        from .blueprint import Blueprint
        return Blueprint().add(cls, **kwargs)

    # -- 信息 -------------------------------------------------------------

    def port_summary(self) -> Dict[str, Any]:
        """返回端口摘要信息，用于调试和健康检查。"""
        return {
            "module": type(self).__name__,
            "layer": self._layer,
            "running": self._running,
            "ports_in": {
                name: {
                    "type": port.msg_type.__name__,
                    "msg_count": port.msg_count,
                    "connected": port.connected,
                }
                for name, port in self._ports_in.items()
            },
            "ports_out": {
                name: {
                    "type": port.msg_type.__name__,
                    "msg_count": port.msg_count,
                    "callbacks": port.callback_count,
                }
                for name, port in self._ports_out.items()
            },
        }

    def __repr__(self) -> str:
        layer_str = f", L{self._layer}" if self._layer is not None else ""
        return (
            f"{type(self).__name__}("
            f"in={list(self._ports_in.keys())}, "
            f"out={list(self._ports_out.keys())}"
            f"{layer_str})"
        )
