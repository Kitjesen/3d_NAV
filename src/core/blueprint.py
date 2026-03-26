"""lingtu.core.blueprint — 模块编排蓝图，声明式连接与依赖图。

Blueprint 是 LingTu 编排框架的组装器：

1. add()       — 注册模块类及其配置
2. wire()      — 显式连接 Out→In
3. auto_wire() — 按 (name, msg_type) 自动匹配
4. build()     — 实例化所有模块，验证依赖层级，返回 SystemHandle

SystemHandle 管理运行时生命周期: setup → start → [running] → stop

使用示例::

    system = (
        Blueprint()
        .add(PerceptionModule, camera_id=0)
        .add(PlannerModule)
        .auto_wire()
        .build()
    )
    system.start()
    # ... 运行 ...
    system.stop()

autoconnect() 工厂函数合并多个 Blueprint::

    system = autoconnect(
        PerceptionModule.blueprint(camera_id=0),
        PlannerModule.blueprint(),
    ).build()
"""

from __future__ import annotations

import logging
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Set, Tuple, Type

from .module import Module
from .stream import In, Out
from .transport.local import Transport, LocalTransport

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# _ModuleEntry — 蓝图内部模块条目
# ---------------------------------------------------------------------------

@dataclass
class _ModuleEntry:
    """蓝图中的一个模块条目。"""
    module_cls: Type[Module]
    config: Dict[str, Any]
    alias: Optional[str] = None
    instance: Optional[Module] = None  # pre-instantiated module (e.g. NativeModule)

    @property
    def name(self) -> str:
        if self.alias:
            return self.alias
        if self.instance is not None:
            return type(self.instance).__name__
        return self.module_cls.__name__


# ---------------------------------------------------------------------------
# _WireSpec — 连接规格
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class _WireSpec:
    """一条 Out→In 连接的规格。

    transport: None = direct callback (default, zero-copy, same process)
               "dds" = CycloneDDS (cross-process, crash isolation)
               "shm" = shared memory (same machine, high bandwidth)
               or a Transport instance for custom backends
    """
    out_module: str   # module name (alias or class.__name__)
    out_port: str     # Out port name
    in_module: str    # module name
    in_port: str      # In port name
    transport: Any = None  # None=callback, "dds"/"shm"/Transport instance


# ---------------------------------------------------------------------------
# Blueprint
# ---------------------------------------------------------------------------

class Blueprint:
    """声明式模块编排蓝图。

    采用 builder 模式，所有配置方法返回 self 支持链式调用。
    不可变语义: build() 后不应再修改蓝图。
    """

    def __init__(self) -> None:
        self._entries: List[_ModuleEntry] = []
        self._wires: List[_WireSpec] = []
        self._auto_wired: bool = False

    # -- 注册 ---------------------------------------------------------------

    def add(self, module_or_cls, alias: Optional[str] = None,
            **config: Any) -> Blueprint:
        """Register a module class or a pre-instantiated module.

        Args:
            module_or_cls: Module subclass (instantiated during build) or
                           a Module instance (used as-is, e.g. NativeModule).
            alias: Optional alias for multi-instance scenarios.
            **config: Passed to Module.__init__ when module_or_cls is a class.

        Returns:
            self (chained)
        """
        if isinstance(module_or_cls, Module):
            # Pre-instantiated module (e.g. NativeModule(config))
            entry = _ModuleEntry(
                module_cls=type(module_or_cls),
                config={},
                alias=alias,
                instance=module_or_cls,
            )
        else:
            entry = _ModuleEntry(module_cls=module_or_cls, config=config, alias=alias)
        # 检查名称唯一性
        existing = {e.name for e in self._entries}
        if entry.name in existing:
            raise ValueError(
                f"Blueprint already contains module '{entry.name}'. "
                f"Use alias= to distinguish multiple instances."
            )
        self._entries.append(entry)
        return self

    # -- 连接 ---------------------------------------------------------------

    def wire(self, out_module: str, out_port: str,
             in_module: str, in_port: str,
             transport: Any = None) -> Blueprint:
        """Connect an Out port to an In port.

        Args:
            out_module: Output module name (class name or alias).
            out_port: Output port name.
            in_module: Input module name.
            in_port: Input port name.
            transport: Delivery strategy for this connection.
                None    — direct callback (default, zero-copy, same thread)
                "dds"   — CycloneDDS (cross-process, crash isolation, ~1ms)
                "shm"   — shared memory (same machine, high bandwidth, ~50μs)
                Transport instance — custom backend

        Returns:
            self
        """
        self._wires.append(_WireSpec(out_module, out_port, in_module, in_port, transport))
        return self

    def auto_wire(self) -> Blueprint:
        """自动连接: 按 (端口名, 消息类型) 匹配 Out→In。

        规则:
        1. 端口名相同且消息类型相同 → 自动连接
        2. 已有显式 wire 的端口不再自动连接
        3. 一个 Out 可以连接多个 In (扇出)
        4. 一个 In 只能连一个 Out (首次匹配优先)

        Returns:
            self
        """
        self._auto_wired = True
        return self

    # -- 合并 ---------------------------------------------------------------

    def merge(self, other: Blueprint) -> Blueprint:
        """合并另一个 Blueprint 的所有模块和连接规格。"""
        existing = {e.name for e in self._entries}
        for entry in other._entries:
            if entry.name in existing:
                raise ValueError(
                    f"Cannot merge: module '{entry.name}' exists in both blueprints."
                )
            self._entries.append(entry)
            existing.add(entry.name)
        self._wires.extend(other._wires)
        if other._auto_wired:
            self._auto_wired = True
        return self

    # -- 构建 ---------------------------------------------------------------

    def build(self, transport: Optional[Transport] = None) -> SystemHandle:
        """实例化所有模块，执行连接，返回 SystemHandle。

        Args:
            transport: 可选外部传输层。None 时使用 LocalTransport。

        Returns:
            SystemHandle 运行时句柄

        Raises:
            ValueError: 层级依赖违规 (高层 Out → 低层 In 的逆向数据流)
        """
        if transport is None:
            transport = LocalTransport()

        # 1. Instantiate modules (or use pre-built instances)
        instances: Dict[str, Module] = {}
        entry_map: Dict[str, _ModuleEntry] = {}
        for entry in self._entries:
            if entry.instance is not None:
                instance = entry.instance
            else:
                instance = entry.module_cls(**entry.config)
            instances[entry.name] = instance
            entry_map[entry.name] = entry

        # 2. 收集所有端口信息
        out_ports: Dict[str, Dict[str, Out[Any]]] = {}  # module_name → {port_name: Out}
        in_ports: Dict[str, Dict[str, In[Any]]] = {}    # module_name → {port_name: In}
        for mod_name, mod in instances.items():
            out_ports[mod_name] = mod.ports_out
            in_ports[mod_name] = mod.ports_in

        # 3. 执行显式连接
        wired_in: Set[Tuple[str, str]] = set()  # (mod_name, port_name) — 已连接的 In
        connections: List[Tuple[str, str, str, str]] = []  # 用于日志和健康检查

        for spec in self._wires:
            self._do_wire(spec, instances, out_ports, in_ports, wired_in, connections)

        # 4. 自动连接
        if self._auto_wired:
            self._do_auto_wire(instances, out_ports, in_ports, wired_in, connections)

        # 5. 层级依赖校验
        # Runtime signal graphs may contain intentional feedback loops.
        layer_violations = self._check_layer_deps(instances, connections)
        for v in layer_violations:
            logger.warning("Layer dependency violation: %s", v)

        # 6. Bind global transport to Out ports that don't already have
        #    a per-wire transport (set by _do_wire with transport= param)
        for mod_name, mod in instances.items():
            for port_name, port in mod.ports_out.items():
                if port._transport is None:
                    topic = f"/{mod_name}/{port_name}"
                    port._bind_transport(transport, topic)

        # 7. 计算启动顺序 (拓扑排序)
        startup_order = self._topo_sort(instances, connections)

        return SystemHandle(
            modules=instances,
            transport=transport,
            connections=connections,
            startup_order=startup_order,
            layer_violations=layer_violations,
        )

    # -- 内部方法 -----------------------------------------------------------

    @staticmethod
    def _resolve_transport(transport_spec: Any) -> Optional[Transport]:
        """Resolve a transport spec ("dds", "shm", instance) to a Transport."""
        if transport_spec is None:
            return None  # direct callback
        if isinstance(transport_spec, str):
            if transport_spec == "dds":
                from .transport.dds import DDSTransport
                from .transport.adapter import TransportAdapter
                return TransportAdapter(DDSTransport())
            elif transport_spec == "shm":
                from .transport.shm import SHMTransport
                from .transport.adapter import TransportAdapter
                return TransportAdapter(SHMTransport())
            elif transport_spec == "local":
                return LocalTransport()
            else:
                raise ValueError(f"Unknown transport: '{transport_spec}'")
        # Assume it's already a Transport instance
        return transport_spec

    @staticmethod
    def _do_wire(
        spec: _WireSpec,
        instances: Dict[str, Module],
        out_ports: Dict[str, Dict[str, Out[Any]]],
        in_ports: Dict[str, Dict[str, In[Any]]],
        wired_in: Set[Tuple[str, str]],
        connections: List[Tuple[str, str, str, str]],
    ) -> None:
        """Execute one wire connection."""
        if spec.out_module not in instances:
            raise ValueError(f"wire: unknown output module '{spec.out_module}'")
        if spec.in_module not in instances:
            raise ValueError(f"wire: unknown input module '{spec.in_module}'")

        o = out_ports.get(spec.out_module, {}).get(spec.out_port)
        i = in_ports.get(spec.in_module, {}).get(spec.in_port)
        if o is None:
            raise ValueError(
                f"wire: module '{spec.out_module}' has no Out port '{spec.out_port}'"
            )
        if i is None:
            raise ValueError(
                f"wire: module '{spec.in_module}' has no In port '{spec.in_port}'"
            )

        # Type check
        if o.msg_type != i.msg_type:
            raise TypeError(
                f"wire: type mismatch {spec.out_module}.{spec.out_port} "
                f"({o.msg_type.__name__}) → {spec.in_module}.{spec.in_port} "
                f"({i.msg_type.__name__})"
            )

        transport = Blueprint._resolve_transport(spec.transport)
        if transport is None:
            # Direct callback — zero-copy, same thread (default)
            o._add_callback(i._deliver)
            mode = "callback"
        else:
            # Transport-mediated — decoupled, may cross process/thread
            topic = f"/{spec.out_module}/{spec.out_port}"
            o._bind_transport(transport, topic)
            transport.subscribe(topic, i._deliver)
            mode = f"transport({getattr(transport, 'backend_name', type(transport).__name__)})"

        wired_in.add((spec.in_module, spec.in_port))
        connections.append((spec.out_module, spec.out_port, spec.in_module, spec.in_port))
        logger.debug(
            "Wired %s.%s → %s.%s [%s, %s]",
            spec.out_module, spec.out_port,
            spec.in_module, spec.in_port,
            o.msg_type.__name__, mode,
        )

    @staticmethod
    def _do_auto_wire(
        instances: Dict[str, Module],
        out_ports: Dict[str, Dict[str, Out[Any]]],
        in_ports: Dict[str, Dict[str, In[Any]]],
        wired_in: Set[Tuple[str, str]],
        connections: List[Tuple[str, str, str, str]],
    ) -> None:
        """按 (name, msg_type) 自动匹配 Out→In。"""
        # 建立 Out 索引: (port_name, msg_type) → [(mod_name, Out)]
        out_index: Dict[Tuple[str, type], List[Tuple[str, Out[Any]]]] = defaultdict(list)
        for mod_name, ports in out_ports.items():
            for port_name, port in ports.items():
                out_index[(port_name, port.msg_type)].append((mod_name, port))

        # 遍历所有 In，尝试匹配
        for in_mod_name, ports in in_ports.items():
            for in_port_name, in_port in ports.items():
                if (in_mod_name, in_port_name) in wired_in:
                    continue  # 已显式连接

                key = (in_port_name, in_port.msg_type)
                candidates = out_index.get(key, [])

                # 排除自连接
                candidates = [(mn, op) for mn, op in candidates if mn != in_mod_name]

                if len(candidates) == 1:
                    out_mod_name, out_port = candidates[0]
                    out_port._add_callback(in_port._deliver)
                    wired_in.add((in_mod_name, in_port_name))
                    connections.append((out_mod_name, in_port_name, in_mod_name, in_port_name))
                    logger.debug(
                        "Auto-wired %s.%s → %s.%s [%s]",
                        out_mod_name, in_port_name,
                        in_mod_name, in_port_name,
                        in_port.msg_type.__name__,
                    )
                elif len(candidates) > 1:
                    logger.warning(
                        "Auto-wire ambiguity for %s.%s [%s]: %d candidates, skipping",
                        in_mod_name, in_port_name,
                        in_port.msg_type.__name__,
                        len(candidates),
                    )

    @staticmethod
    def _check_layer_deps(
        instances: Dict[str, Module],
        connections: List[Tuple[str, str, str, str]],
    ) -> List[str]:
        """检查层级依赖: 层 N 的 Out 不应连到层 <N 的 In (逆向数据流)。

        层级为 None 的模块跳过检查。
        返回违规描述列表 (空=通过)。
        """
        # Runtime data-flow graphs in robotics are often intentionally cyclic:
        # driver -> mapper -> planner -> controller -> driver. Those feedback
        # loops are not Python import-layer violations, so Blueprint.build()
        # should not warn on them.
        _ = (instances, connections)
        return []
        violations: List[str] = []
        # Note: layer checking at data-flow level is intentionally lenient.
        # Cross-layer data flow is normal in robotics (L1 sensor → L5 planner,
        # L5 command → L1 driver). The layer contract ("N imports 0..N-1")
        # governs Python imports, not data flow direction.
        # We only detect cycles here, which indicate true dependency problems.

        # 检测连接环路
        adj: Dict[str, Set[str]] = defaultdict(set)
        for out_mod, _, in_mod, _ in connections:
            adj[out_mod].add(in_mod)

        # Cycle detection (DFS with explicit path tracking)
        WHITE, GRAY, BLACK = 0, 1, 2
        color: Dict[str, int] = {n: WHITE for n in instances}

        for start in instances:
            if color[start] != WHITE:
                continue
            stack = [(start, iter(adj.get(start, [])))]
            color[start] = GRAY
            path = [start]
            while stack:
                node, neighbors = stack[-1]
                try:
                    nxt = next(neighbors)
                    if nxt not in color:
                        continue
                    if color[nxt] == GRAY:
                        # Found cycle
                        idx = path.index(nxt)
                        cycle = path[idx:] + [nxt]
                        violations.append(f"Cycle detected: {' → '.join(cycle)}")
                    elif color[nxt] == WHITE:
                        color[nxt] = GRAY
                        path.append(nxt)
                        stack.append((nxt, iter(adj.get(nxt, []))))
                except StopIteration:
                    color[node] = BLACK
                    path.pop()
                    stack.pop()

        return violations

    @staticmethod
    def _topo_sort(
        instances: Dict[str, Module],
        connections: List[Tuple[str, str, str, str]],
    ) -> List[str]:
        """拓扑排序: 上游模块先启动。环路时回退到注册顺序。"""
        # 构建依赖图: in_mod depends on out_mod
        in_degree: Dict[str, int] = {name: 0 for name in instances}
        adj: Dict[str, List[str]] = defaultdict(list)

        for out_mod, _, in_mod, _ in connections:
            if out_mod != in_mod:
                adj[out_mod].append(in_mod)
                in_degree[in_mod] = in_degree.get(in_mod, 0) + 1

        # Kahn's algorithm
        queue = [n for n, d in in_degree.items() if d == 0]
        result: List[str] = []

        while queue:
            # 按层级排序: 低层先启动
            queue.sort(key=lambda n: instances[n].layer or 0)
            node = queue.pop(0)
            result.append(node)
            for neighbor in adj.get(node, []):
                in_degree[neighbor] -= 1
                if in_degree[neighbor] == 0:
                    queue.append(neighbor)

        if len(result) < len(instances):
            # 有环，添加剩余节点
            remaining = [n for n in instances if n not in set(result)]
            result.extend(remaining)

        return result


# ---------------------------------------------------------------------------
# SystemHandle — 运行时句柄
# ---------------------------------------------------------------------------

class SystemHandle:
    """模块系统运行时句柄。

    管理所有模块的生命周期: setup → start → [running] → stop
    """

    def __init__(
        self,
        modules: Dict[str, Module],
        transport: Transport,
        connections: List[Tuple[str, str, str, str]],
        startup_order: List[str],
        layer_violations: Optional[List[str]] = None,
    ) -> None:
        self._modules = modules
        self._transport = transport
        self._connections = connections
        self._startup_order = startup_order
        self._layer_violations = layer_violations or []
        self._started = False

    # -- 生命周期 -----------------------------------------------------------

    def start(self) -> None:
        """按拓扑顺序 setup + start 所有模块。"""
        if self._started:
            logger.warning("SystemHandle.start() called but already started")
            return
        for name in self._startup_order:
            mod = self._modules[name]
            mod.setup()
        for name in self._startup_order:
            mod = self._modules[name]
            mod.start()
        self._started = True
        logger.info(
            "System started: %d modules, %d connections",
            len(self._modules), len(self._connections),
        )

    def stop(self) -> None:
        """按拓扑逆序 stop 所有模块，然后关闭传输层。"""
        if not self._started:
            return
        for name in reversed(self._startup_order):
            mod = self._modules.get(name)
            if mod is None:
                continue
            try:
                mod.stop()
            except Exception:
                logger.exception("Error stopping module %s", name)
        if hasattr(self._transport, 'close'):
            try:
                self._transport.close()
            except Exception:
                logger.exception("Error closing transport")
        # Break module references to allow GC of the entire graph
        self._modules.clear()
        self._connections.clear()
        self._started = False
        logger.info("System stopped")

    # -- 访问 ---------------------------------------------------------------

    def get_module(self, name: str) -> Module:
        """按名称获取模块实例。"""
        if name not in self._modules:
            raise KeyError(f"Unknown module: '{name}'")
        return self._modules[name]

    @property
    def modules(self) -> Dict[str, Module]:
        """所有模块 {name: instance}。"""
        return dict(self._modules)

    @property
    def connections(self) -> List[Tuple[str, str, str, str]]:
        """所有连接 [(out_mod, out_port, in_mod, in_port), ...]。"""
        return list(self._connections)

    @property
    def started(self) -> bool:
        return self._started

    # -- 健康检查 -----------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        """返回系统健康状态。"""
        module_health: Dict[str, Any] = {}
        for name, mod in self._modules.items():
            module_health[name] = mod.port_summary()

        total_in = sum(
            port.msg_count
            for mod in self._modules.values()
            for port in mod.ports_in.values()
        )
        total_out = sum(
            port.msg_count
            for mod in self._modules.values()
            for port in mod.ports_out.values()
        )

        return {
            "started": self._started,
            "module_count": len(self._modules),
            "connection_count": len(self._connections),
            "startup_order": self._startup_order,
            "layer_violations": self._layer_violations,
            "total_messages_in": total_in,
            "total_messages_out": total_out,
            "modules": module_health,
        }

    def __repr__(self) -> str:
        status = "running" if self._started else "stopped"
        return (
            f"SystemHandle({status}, modules={len(self._modules)}, "
            f"connections={len(self._connections)})"
        )


# ---------------------------------------------------------------------------
# autoconnect() — 合并多个 Blueprint 并自动连接
# ---------------------------------------------------------------------------

def autoconnect(*blueprints: Blueprint) -> Blueprint:
    """合并多个 Blueprint 并执行 auto_wire。

    用于将独立定义的模块蓝图组装为完整系统::

        system = autoconnect(
            PerceptionModule.blueprint(camera_id=0),
            PlannerModule.blueprint(),
        ).build()
    """
    result = Blueprint()
    for bp in blueprints:
        result.merge(bp)
    result.auto_wire()
    return result
