"""core.blueprint — Declarative module orchestration.

Blueprint assembles a system from Module classes and explicit or auto-matched
wiring rules.  build() instantiates everything and returns a SystemHandle.

Typical usage::

    system = (
        Blueprint()
        .add(PerceptionModule, camera_id=0)
        .add(PlannerModule)
        .auto_wire()
        .build()
    )
    system.start()
    system.stop()

autoconnect() merges multiple blueprints and enables auto_wire in one call::

    system = autoconnect(
        driver("thunder"),
        slam("localizer"),
        perception("bpu"),
        navigation("astar"),
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
# _ModuleEntry
# ---------------------------------------------------------------------------

@dataclass
class _ModuleEntry:
    """One module slot in a Blueprint."""

    module_cls: Type[Module]
    config: Dict[str, Any]
    alias: Optional[str] = None
    instance: Optional[Module] = None  # pre-instantiated (e.g. NativeModule)

    @property
    def name(self) -> str:
        if self.alias:
            return self.alias
        if self.instance is not None:
            return type(self.instance).__name__
        return self.module_cls.__name__


# ---------------------------------------------------------------------------
# _WireSpec
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class _WireSpec:
    """One Out→In connection specification.

    transport: None  — direct callback (zero-copy, same thread, default)
               "dds" — CycloneDDS (cross-process, ~1 ms)
               "shm" — shared memory (same machine, high-bandwidth, ~50 μs)
               Transport instance — custom backend
    """

    out_module: str
    out_port: str
    in_module: str
    in_port: str
    transport: Any = None


# ---------------------------------------------------------------------------
# Blueprint
# ---------------------------------------------------------------------------

class Blueprint:
    """Declarative module orchestration builder.

    All configuration methods return *self* to support method chaining.
    A Blueprint should not be modified after build() is called.
    """

    def __init__(self) -> None:
        self._entries: List[_ModuleEntry] = []
        self._wires: List[_WireSpec] = []
        self._auto_wired: bool = False
        self._global_cfg: Dict[str, Any] = {}

    # -- registration -------------------------------------------------------

    def add(
        self,
        module_or_cls,
        alias: Optional[str] = None,
        **config: Any,
    ) -> "Blueprint":
        """Register a Module class or a pre-instantiated Module.

        Args:
            module_or_cls: Module subclass (instantiated during build()) or a
                Module instance (used as-is, e.g. NativeModule).
            alias: Optional name override for multi-instance scenarios.
            **config: Forwarded to Module.__init__ when a class is given.

        Returns:
            self
        """
        if isinstance(module_or_cls, Module):
            entry = _ModuleEntry(
                module_cls=type(module_or_cls),
                config={},
                alias=alias,
                instance=module_or_cls,
            )
        else:
            entry = _ModuleEntry(module_cls=module_or_cls, config=config, alias=alias)

        existing = {e.name for e in self._entries}
        if entry.name in existing:
            raise ValueError(
                f"Blueprint already contains '{entry.name}'. "
                "Use alias= to distinguish multiple instances of the same class."
            )
        self._entries.append(entry)
        return self

    # -- wiring -------------------------------------------------------------

    def wire(
        self,
        out_module: str,
        out_port: str,
        in_module: str,
        in_port: str,
        transport: Any = None,
    ) -> "Blueprint":
        """Connect an Out port to an In port.

        Args:
            out_module: Source module name (class name or alias).
            out_port:   Source port name.
            in_module:  Destination module name.
            in_port:    Destination port name.
            transport:  Delivery strategy (None = direct callback).

        Returns:
            self
        """
        self._wires.append(_WireSpec(out_module, out_port, in_module, in_port, transport))
        return self

    def global_config(self, n_workers: int = 0, **kwargs: Any) -> "Blueprint":
        """Set system-level configuration.

        Supports the dimos-style chained build::

            autoconnect(...).global_config(n_workers=4).build()

        Args:
            n_workers: Worker subprocess count.  0 = single-process (default).
        """
        self._global_cfg = {"n_workers": n_workers, **kwargs}
        return self

    def auto_wire(self) -> "Blueprint":
        """Enable automatic port matching by (port_name, msg_type).

        Rules:
          1. Same name + same msg_type → connected.
          2. Ports already explicitly wired are skipped.
          3. One Out may fan out to multiple In ports.
          4. If multiple Out ports match a single In, the connection is skipped
             with a warning (ambiguity).  Add an explicit wire() to resolve.

        Returns:
            self
        """
        self._auto_wired = True
        return self

    # -- merge --------------------------------------------------------------

    def merge(self, other: "Blueprint") -> "Blueprint":
        """Merge all modules and wires from another Blueprint into this one."""
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

    # -- build --------------------------------------------------------------

    def build(
        self,
        transport: Optional[Transport] = None,
        n_workers: int = 0,
    ) -> "SystemHandle":
        """Instantiate all modules, apply wiring, and return a runtime handle.

        Args:
            transport: Optional shared Transport.  Defaults to LocalTransport.
            n_workers: Worker subprocess count.  0 = single-process (default).

        Returns:
            SystemHandle (n_workers=0) or WorkerSystemHandle (n_workers>0)

        Raises:
            ValueError: Unknown module or port in an explicit wire().
            TypeError:  Type mismatch in an explicit wire().
        """
        n_workers = self._global_cfg.get("n_workers", n_workers)
        if n_workers > 0:
            return self._build_worker_mode(n_workers)

        if transport is None:
            transport = LocalTransport()

        # 1. Instantiate modules
        instances: Dict[str, Module] = {}
        for entry in self._entries:
            inst = entry.instance if entry.instance is not None else entry.module_cls(**entry.config)
            instances[entry.name] = inst

        # 2. Collect ports
        out_ports: Dict[str, Dict[str, Out]] = {n: m.ports_out for n, m in instances.items()}
        in_ports:  Dict[str, Dict[str, In]]  = {n: m.ports_in  for n, m in instances.items()}

        # 3. Apply explicit wires
        wired_in: Set[Tuple[str, str]] = set()
        connections: List[Tuple[str, str, str, str]] = []
        for spec in self._wires:
            _do_wire(spec, instances, out_ports, in_ports, wired_in, connections)

        # 4. Auto-wire remaining ports
        if self._auto_wired:
            _do_auto_wire(instances, out_ports, in_ports, wired_in, connections)

        # 5. Bind the global transport to Out ports that have no per-wire transport
        for mod_name, mod in instances.items():
            for port_name, port in mod.ports_out.items():
                if port._transport is None:
                    port._bind_transport(transport, f"/{mod_name}/{port_name}")

        # 6. Topological startup order
        startup_order = _topo_sort(instances, connections)

        # 7. Notify modules of the full module set
        for mod in instances.values():
            mod.on_system_modules(dict(instances))

        return SystemHandle(
            modules=instances,
            transport=transport,
            connections=connections,
            startup_order=startup_order,
        )

    def __repr__(self) -> str:
        modules = ", ".join(e.name for e in self._entries)
        return f"Blueprint(modules=[{modules}], wires={len(self._wires)}, auto_wire={self._auto_wired})"

    # -- worker-mode build --------------------------------------------------

    def _build_worker_mode(self, n_workers: int) -> "WorkerSystemHandle":
        """Deploy modules to Worker subprocesses.

        Modules with _run_in_main=True stay in the main process and receive
        RPCClient proxies via on_system_modules().  All others are deployed
        to worker subprocesses round-robin.

        Cross-boundary wires use SHM for the data path.
        """
        from core.coordinator import ModuleCoordinator
        from core.transport.shm import SHMTransport
        from core.transport.adapter import TransportAdapter

        worker_entries = [e for e in self._entries if not e.module_cls._run_in_main]
        local_entries  = [e for e in self._entries if     e.module_cls._run_in_main]

        coord = ModuleCoordinator(n_workers=n_workers)
        coord.start()
        proxies: Dict[str, Any] = {}
        for entry in worker_entries:
            proxies[entry.name] = coord.deploy(entry.module_cls, entry.name, kwargs=entry.config)

        local_instances: Dict[str, Any] = {}
        for entry in local_entries:
            inst = entry.instance if entry.instance is not None else entry.module_cls(**entry.config)
            local_instances[entry.name] = inst

        local_out = {n: m.ports_out for n, m in local_instances.items()}
        local_in  = {n: m.ports_in  for n, m in local_instances.items()}
        wired_in: Set[Tuple[str, str]] = set()
        connections: List[Tuple[str, str, str, str]] = []

        for spec in self._wires:
            out_mod, out_port = spec.out_module, spec.out_port
            in_mod,  in_port  = spec.in_module,  spec.in_port
            topic = f"/{out_mod}/{out_port}"
            out_worker = out_mod in proxies
            in_worker  = in_mod  in proxies

            if out_worker and in_worker:
                coord._mgr.bind_port(coord._assignments[out_mod], out_mod, out_port, "out", topic)
                coord._mgr.bind_port(coord._assignments[in_mod],  in_mod,  in_port,  "in",  topic)
            elif not out_worker and not in_worker:
                _do_wire(spec, local_instances, local_out, local_in, wired_in, connections)
                continue
            elif out_worker:
                coord._mgr.bind_port(coord._assignments[out_mod], out_mod, out_port, "out", topic)
                p = local_in.get(in_mod, {}).get(in_port)
                if p is not None and (in_mod, in_port) not in wired_in:
                    TransportAdapter(SHMTransport()).subscribe(topic, p._deliver)
                    wired_in.add((in_mod, in_port))
            else:
                p = local_out.get(out_mod, {}).get(out_port)
                if p is not None:
                    p._bind_transport(TransportAdapter(SHMTransport()), topic)
                coord._mgr.bind_port(coord._assignments[in_mod], in_mod, in_port, "in", topic)

            connections.append((out_mod, out_port, in_mod, in_port))

        if self._auto_wired:
            _do_auto_wire(local_instances, local_out, local_in, wired_in, connections)

        coord.setup_all()
        coord.start_all()
        for inst in local_instances.values():
            inst.setup()
        for inst in local_instances.values():
            inst.start()

        all_modules: Dict[str, Any] = {**proxies, **local_instances}
        for inst in local_instances.values():
            inst.on_system_modules(all_modules)

        logger.info(
            "WorkerMode: %d worker modules (%d workers), %d local, %d connections",
            len(proxies), n_workers, len(local_instances), len(connections),
        )
        return WorkerSystemHandle(coord, proxies, local_instances, connections)


# ---------------------------------------------------------------------------
# Module-level helpers (extracted from Blueprint to remove @staticmethod noise)
# ---------------------------------------------------------------------------

def _resolve_transport(spec: Any) -> Optional[Transport]:
    """Resolve a transport spec string or instance to a Transport."""
    if spec is None:
        return None
    if isinstance(spec, str):
        if spec == "dds":
            from .transport.dds import DDSTransport
            from .transport.adapter import TransportAdapter
            return TransportAdapter(DDSTransport())
        if spec == "shm":
            from .transport.shm import SHMTransport
            from .transport.adapter import TransportAdapter
            return TransportAdapter(SHMTransport())
        if spec == "local":
            return LocalTransport()
        raise ValueError(f"Unknown transport spec: '{spec}'")
    return spec  # already a Transport instance


def _do_wire(
    spec: _WireSpec,
    instances: Dict[str, Any],
    out_ports: Dict[str, Dict[str, Out]],
    in_ports:  Dict[str, Dict[str, In]],
    wired_in:  Set[Tuple[str, str]],
    connections: List[Tuple[str, str, str, str]],
) -> None:
    """Apply one explicit wire specification."""
    if spec.out_module not in instances:
        raise ValueError(f"wire(): unknown output module '{spec.out_module}'")
    if spec.in_module not in instances:
        raise ValueError(f"wire(): unknown input module '{spec.in_module}'")

    out = out_ports.get(spec.out_module, {}).get(spec.out_port)
    inp = in_ports.get(spec.in_module, {}).get(spec.in_port)

    if out is None:
        raise ValueError(f"wire(): '{spec.out_module}' has no Out port '{spec.out_port}'")
    if inp is None:
        raise ValueError(f"wire(): '{spec.in_module}' has no In port '{spec.in_port}'")

    if out.msg_type != inp.msg_type:
        raise TypeError(
            f"wire(): type mismatch "
            f"{spec.out_module}.{spec.out_port} ({out.msg_type.__name__}) → "
            f"{spec.in_module}.{spec.in_port} ({inp.msg_type.__name__})"
        )

    transport = _resolve_transport(spec.transport)
    if transport is None:
        out._add_callback(inp._deliver)
        mode = "callback"
    else:
        topic = f"/{spec.out_module}/{spec.out_port}"
        out._bind_transport(transport, topic)
        transport.subscribe(topic, inp._deliver)
        mode = f"transport({getattr(transport, 'backend_name', type(transport).__name__)})"

    wired_in.add((spec.in_module, spec.in_port))
    connections.append((spec.out_module, spec.out_port, spec.in_module, spec.in_port))
    logger.debug(
        "Wired %s.%s → %s.%s [%s, %s]",
        spec.out_module, spec.out_port,
        spec.in_module, spec.in_port,
        out.msg_type.__name__, mode,
    )


def _do_auto_wire(
    instances: Dict[str, Any],
    out_ports: Dict[str, Dict[str, Out]],
    in_ports:  Dict[str, Dict[str, In]],
    wired_in:  Set[Tuple[str, str]],
    connections: List[Tuple[str, str, str, str]],
) -> None:
    """Auto-match Out→In by (port_name, msg_type)."""
    # Index: (port_name, msg_type) → [(module_name, Out)]
    out_index: Dict[Tuple[str, type], List[Tuple[str, Out]]] = defaultdict(list)
    for mod_name, ports in out_ports.items():
        for port_name, port in ports.items():
            out_index[(port_name, port.msg_type)].append((mod_name, port))

    for in_mod, ports in in_ports.items():
        for in_port_name, in_port in ports.items():
            if (in_mod, in_port_name) in wired_in:
                continue

            candidates = [
                (mn, op)
                for mn, op in out_index.get((in_port_name, in_port.msg_type), [])
                if mn != in_mod
            ]

            if len(candidates) == 1:
                out_mod, out_port = candidates[0]
                out_port._add_callback(in_port._deliver)
                wired_in.add((in_mod, in_port_name))
                connections.append((out_mod, in_port_name, in_mod, in_port_name))
                logger.debug(
                    "Auto-wired %s.%s → %s.%s [%s]",
                    out_mod, in_port_name, in_mod, in_port_name,
                    in_port.msg_type.__name__,
                )
            elif len(candidates) > 1:
                logger.warning(
                    "Auto-wire ambiguity for %s.%s [%s]: %d candidates — "
                    "add an explicit wire() to resolve",
                    in_mod, in_port_name, in_port.msg_type.__name__, len(candidates),
                )


def _topo_sort(
    instances: Dict[str, Any],
    connections: List[Tuple[str, str, str, str]],
) -> List[str]:
    """Kahn's topological sort; falls back to registration order on cycles.

    Upstream modules start first.  Within each topological level, lower-layer
    modules are sorted before higher-layer ones.
    """
    in_degree: Dict[str, int] = {n: 0 for n in instances}
    adj: Dict[str, List[str]] = defaultdict(list)
    for out_mod, _, in_mod, _ in connections:
        if out_mod != in_mod:
            adj[out_mod].append(in_mod)
            in_degree[in_mod] = in_degree.get(in_mod, 0) + 1

    result: List[str] = []
    ready = sorted(
        (n for n, d in in_degree.items() if d == 0),
        key=lambda n: instances[n].layer or 0,
    )

    while ready:
        node = ready.pop(0)
        result.append(node)
        newly_ready = []
        for neighbor in adj.get(node, []):
            in_degree[neighbor] -= 1
            if in_degree[neighbor] == 0:
                newly_ready.append(neighbor)
        # Insert new candidates in layer order
        newly_ready.sort(key=lambda n: instances[n].layer or 0)
        ready = sorted(ready + newly_ready, key=lambda n: instances[n].layer or 0)

    if len(result) < len(instances):
        # Cycle detected — append remaining nodes in registration order
        remaining = [n for n in instances if n not in set(result)]
        result.extend(remaining)

    return result


# ---------------------------------------------------------------------------
# SystemHandle
# ---------------------------------------------------------------------------

class SystemHandle:
    """Runtime handle for a single-process module system.

    Manages the full lifecycle: setup → start → [running] → stop.
    """

    def __init__(
        self,
        modules: Dict[str, Module],
        transport: Transport,
        connections: List[Tuple[str, str, str, str]],
        startup_order: List[str],
    ) -> None:
        self._modules = modules
        self._transport = transport
        self._connections = connections
        self._startup_order = startup_order
        self._started = False

    # -- lifecycle ----------------------------------------------------------

    def start(self) -> None:
        """setup() then start() all modules in topological order."""
        if self._started:
            logger.warning("SystemHandle.start() called but system is already running")
            return
        for name in self._startup_order:
            self._modules[name].setup()
        for name in self._startup_order:
            self._modules[name].start()
        self._started = True
        logger.info(
            "System started: %d modules, %d connections",
            len(self._modules), len(self._connections),
        )

    def stop(self) -> None:
        """stop() all modules in reverse topological order, then close transport."""
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
        if hasattr(self._transport, "close"):
            try:
                self._transport.close()
            except Exception:
                logger.exception("Error closing transport")
        self._modules.clear()
        self._connections.clear()
        self._started = False
        logger.info("System stopped")

    # -- access -------------------------------------------------------------

    def get_module(self, name: str) -> Module:
        if name not in self._modules:
            raise KeyError(f"Unknown module: '{name}'")
        return self._modules[name]

    @property
    def modules(self) -> Dict[str, Module]:
        return dict(self._modules)

    @property
    def connections(self) -> List[Tuple[str, str, str, str]]:
        return list(self._connections)

    @property
    def started(self) -> bool:
        return self._started

    # -- health -------------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        """Return a health summary dict."""
        total_in  = sum(p.msg_count for m in self._modules.values() for p in m.ports_in.values())
        total_out = sum(p.msg_count for m in self._modules.values() for p in m.ports_out.values())
        return {
            "started":           self._started,
            "module_count":      len(self._modules),
            "connection_count":  len(self._connections),
            "startup_order":     self._startup_order,
            "layer_violations":  [],
            "total_messages_in": total_in,
            "total_messages_out": total_out,
            "modules": {n: m.port_summary() for n, m in self._modules.items()},
        }

    def comm_health(self) -> Dict[str, Any]:
        """Aggregate communication health across all modules.

        Returns per-connection stats: rate, drops, errors, latency.
        Flags unhealthy links (stale, high error rate, high latency).
        """
        links = []
        warnings = []
        total_drops = 0
        total_errors = 0

        for src_mod, src_port, dst_mod, dst_port in self._connections:
            src_m = self._modules.get(src_mod)
            dst_m = self._modules.get(dst_mod)
            if not src_m or not dst_m:
                continue
            out_p = src_m.ports_out.get(src_port)
            in_p = dst_m.ports_in.get(dst_port)
            if not out_p or not in_p:
                continue

            drops = in_p.drop_count
            errors = in_p.callback_errors + out_p.publish_errors
            total_drops += drops
            total_errors += errors

            link = {
                "src": f"{src_mod}.{src_port}",
                "dst": f"{dst_mod}.{dst_port}",
                "out_rate_hz": round(out_p.rate_hz, 1),
                "in_rate_hz": round(in_p.rate_hz, 1),
                "out_count": out_p.msg_count,
                "in_count": in_p.msg_count,
                "drops": drops,
                "errors": errors,
                "avg_cb_ms": round(in_p.avg_callback_ms, 2),
                "max_cb_ms": round(in_p.max_callback_ms, 2),
                "stale_ms": round(in_p.stale_ms, 1),
            }
            links.append(link)

            # Flag unhealthy links
            if in_p.stale_ms > 5000 and in_p.msg_count > 0:
                warnings.append(f"{link['src']}→{link['dst']}: stale {in_p.stale_ms:.0f}ms")
            if errors > 0:
                warnings.append(f"{link['src']}→{link['dst']}: {errors} errors")
            if in_p.max_callback_ms > 500:
                warnings.append(f"{link['src']}→{link['dst']}: slow callback {in_p.max_callback_ms:.0f}ms")

        return {
            "link_count": len(links),
            "total_drops": total_drops,
            "total_errors": total_errors,
            "warnings": warnings,
            "links": links,
        }

    def __repr__(self) -> str:
        status = "running" if self._started else "stopped"
        return (
            f"SystemHandle({status}, modules={len(self._modules)}, "
            f"connections={len(self._connections)})"
        )


# ---------------------------------------------------------------------------
# WorkerSystemHandle
# ---------------------------------------------------------------------------

class WorkerSystemHandle:
    """Runtime handle for a multi-process worker-mode system.

    Worker modules are accessible as RPCClient proxies; main-process modules
    as bare instances.  Mirrors the SystemHandle interface.
    """

    def __init__(
        self,
        coord: Any,
        proxies: Dict[str, Any],
        local_instances: Dict[str, Any],
        connections: List[Tuple[str, str, str, str]],
    ) -> None:
        self._coord = coord
        self._proxies = proxies
        self._local = local_instances
        self._connections = connections
        self._started = True  # lifecycle already completed during build()

    def start(self) -> None:
        """No-op — lifecycle is handled inside build()."""

    def stop(self) -> None:
        """Stop main-process modules, then shut down all worker subprocesses."""
        for name in reversed(list(self._local)):
            try:
                self._local[name].stop()
            except Exception:
                logger.exception("Error stopping local module %s", name)
        self._coord.shutdown()
        self._local.clear()
        self._proxies.clear()
        self._connections.clear()
        self._started = False

    def get_module(self, name: str) -> Any:
        if name in self._proxies:
            return self._proxies[name]
        if name in self._local:
            return self._local[name]
        raise KeyError(f"Unknown module: '{name}'")

    @property
    def modules(self) -> Dict[str, Any]:
        return {**self._proxies, **self._local}

    @property
    def connections(self) -> List[Tuple[str, str, str, str]]:
        return list(self._connections)

    @property
    def started(self) -> bool:
        return self._started

    def health(self) -> Dict[str, Any]:
        modules: Dict[str, Any] = {}
        for name, proxy in self._proxies.items():
            try:
                modules[name] = proxy.health()
            except Exception as exc:
                modules[name] = {"error": str(exc)}
        for name, inst in self._local.items():
            modules[name] = inst.port_summary()
        return {
            "started":         self._started,
            "module_count":    len(self._proxies) + len(self._local),
            "worker_modules":  list(self._proxies.keys()),
            "local_modules":   list(self._local.keys()),
            "connection_count": len(self._connections),
            "modules":         modules,
        }

    def __repr__(self) -> str:
        status = "running" if self._started else "stopped"
        return (
            f"WorkerSystemHandle({status}, workers={list(self._proxies.keys())}, "
            f"local={list(self._local.keys())})"
        )


# ---------------------------------------------------------------------------
# autoconnect()
# ---------------------------------------------------------------------------

def autoconnect(*blueprints: Blueprint) -> Blueprint:
    """Merge multiple Blueprints and enable auto_wire.

    Convenience factory for the composable-stack pattern::

        system = autoconnect(
            driver("thunder", host="192.168.66.190"),
            slam("localizer"),
            perception("bpu"),
            navigation("astar"),
        ).build()
    """
    result = Blueprint()
    for bp in blueprints:
        result.merge(bp)
    result.auto_wire()
    return result
