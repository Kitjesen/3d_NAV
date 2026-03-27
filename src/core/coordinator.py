"""ModuleCoordinator — deploys Modules across Worker processes.

High-level orchestrator that manages module-to-worker assignment,
RPC proxy creation, and lifecycle coordination.

Usage:
    coord = ModuleCoordinator(n_workers=4)
    coord.start()
    proxy = coord.deploy(MyModule, "my_module", kwargs={"port": 5050})
    coord.setup_all()
    coord.start_all()
    result = proxy.some_rpc_method(x=5)
    coord.shutdown()
"""
import logging
from typing import Any, Dict, Optional, Set, Tuple, Type

from core.rpc_client import RPCClient
from core.module import Module

logger = logging.getLogger(__name__)


class ModuleCoordinator:
    """Orchestrates Module deployment across Worker processes.

    Wraps a WorkerManager (imported lazily to avoid a hard dependency while
    worker_manager.py is being developed in parallel) and provides a clean
    high-level API for deploying modules and issuing lifecycle commands.
    """

    def __init__(self, n_workers: int = 4) -> None:
        # Lazy import so this file is importable even before worker_manager exists.
        from core.worker_manager import WorkerManager  # type: ignore[import]

        self._mgr = WorkerManager(n_workers=n_workers)
        self._proxies: Dict[str, RPCClient] = {}
        self._assignments: Dict[str, int] = {}  # module_id → worker_id
        self._next_worker: int = 0

    def start(self) -> None:
        """Start all worker processes."""
        self._mgr.start()

    def deploy(
        self,
        module_cls: Type[Module],
        module_id: str,
        args: Tuple = (),
        kwargs: Optional[Dict[str, Any]] = None,
        worker_id: Optional[int] = None,
    ) -> RPCClient:
        """Deploy a Module to a Worker and return an RPC proxy.

        Args:
            module_cls: Module subclass to instantiate in the worker.
            module_id: Unique string identifier for this module instance.
            args: Positional constructor arguments forwarded to the worker.
            kwargs: Keyword constructor arguments forwarded to the worker.
            worker_id: Pin to a specific worker (None = round-robin).

        Returns:
            RPCClient proxy; calling proxy.some_rpc() forwards the call.
        """
        if worker_id is None:
            worker_id = self._next_worker % self._mgr.n_workers
            self._next_worker += 1

        self._mgr.deploy(worker_id, module_cls, module_id, args, kwargs or {})

        # Discover @rpc methods from the class without instantiating it.
        rpc_methods: Set[str] = set()
        for name in dir(module_cls):
            if name.startswith("_"):
                continue
            try:
                attr = getattr(module_cls, name)
                if callable(attr) and getattr(attr, "__rpc__", False):
                    rpc_methods.add(name)
            except Exception:
                continue

        proxy = RPCClient(self._mgr, worker_id, module_id, rpc_methods)
        self._proxies[module_id] = proxy
        self._assignments[module_id] = worker_id
        return proxy

    def setup_all(self) -> None:
        """Call setup() on all deployed modules (in registration order)."""
        for mod_id, wid in self._assignments.items():
            self._mgr.setup(wid, mod_id)

    def start_all(self) -> None:
        """Call start() on all deployed modules (in registration order)."""
        for mod_id, wid in self._assignments.items():
            self._mgr.start_module(wid, mod_id)

    def stop_all(self) -> None:
        """Call stop() on all deployed modules. Errors are suppressed."""
        for mod_id, wid in self._assignments.items():
            try:
                self._mgr.stop_module(wid, mod_id)
            except Exception:
                logger.exception("Error stopping module '%s'", mod_id)

    def shutdown(self) -> None:
        """Stop all modules and shut down all worker processes."""
        self.stop_all()
        self._mgr.shutdown()
        self._proxies.clear()
        self._assignments.clear()

    def get_proxy(self, module_id: str) -> Optional[RPCClient]:
        """Return the RPCClient for a deployed module, or None."""
        return self._proxies.get(module_id)

    @property
    def proxies(self) -> Dict[str, RPCClient]:
        """Shallow copy of the proxy registry."""
        return dict(self._proxies)

    @property
    def n_workers(self) -> int:
        return self._mgr.n_workers
