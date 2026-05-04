from __future__ import annotations

import sys
import types

import pytest


class _FakeGateway:
    def __init__(self, run_result: bool):
        self._defer_server = False
        self._run_result = run_result
        self.run_server_called = False

    def _run_server(self) -> bool:
        self.run_server_called = True
        return self._run_result


class _FakeSystem:
    def __init__(self, gateway: _FakeGateway):
        self.gateway = gateway
        self.modules = {"GatewayModule": gateway}
        self.started = False
        self.stopped = False

    def get_module(self, name: str):
        if name != "GatewayModule":
            raise KeyError(name)
        return self.gateway

    def start(self) -> None:
        assert self.gateway._defer_server is True
        self.started = True

    def stop(self) -> None:
        self.stopped = True


class _FakeBuilder:
    def __init__(self, system: _FakeSystem):
        self._system = system

    def build(self) -> _FakeSystem:
        return self._system


def _install_cli_harness(monkeypatch, tmp_path, system: _FakeSystem) -> None:
    import cli.main as main_mod

    fake_full_stack = types.ModuleType("core.blueprints.full_stack")
    fake_full_stack.full_stack_blueprint = lambda **kwargs: _FakeBuilder(system)
    monkeypatch.setitem(sys.modules, "core.blueprints.full_stack", fake_full_stack)

    fake_ros2 = types.ModuleType("core.ros2_context")
    fake_ros2.shutdown_shared_executor = lambda: None
    monkeypatch.setitem(sys.modules, "core.ros2_context", fake_ros2)

    fake_service_manager = types.ModuleType("core.service_manager")
    fake_service_manager.get_service_manager = lambda: types.SimpleNamespace(
        _started=[]
    )
    monkeypatch.setitem(sys.modules, "core.service_manager", fake_service_manager)

    monkeypatch.setattr(main_mod, "setup_logging", lambda *args, **kwargs: tmp_path)
    monkeypatch.setattr(main_mod, "preflight", lambda *args, **kwargs: None)
    monkeypatch.setattr(main_mod, "health_check", lambda _system: True)
    monkeypatch.setattr(main_mod, "kill_residual_ports", lambda _cfg: None)
    monkeypatch.setattr(main_mod, "print_banner", lambda *args, **kwargs: None)
    monkeypatch.setattr(main_mod, "save_run_state", lambda *args, **kwargs: None)
    monkeypatch.setattr(main_mod, "update_run_state", lambda *args, **kwargs: None)
    monkeypatch.setattr(main_mod, "clear_run_state", lambda: None)
    monkeypatch.setattr(main_mod.signal, "signal", lambda *args, **kwargs: None)


def test_no_repl_exits_nonzero_when_gateway_server_returns_false(
    monkeypatch, tmp_path
):
    import cli.main as main_mod

    gateway = _FakeGateway(run_result=False)
    system = _FakeSystem(gateway)
    _install_cli_harness(monkeypatch, tmp_path, system)
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "stub", "--no-repl"])

    with pytest.raises(SystemExit) as exc:
        main_mod.main()

    assert exc.value.code == 1
    assert gateway._defer_server is True
    assert gateway.run_server_called is True
    assert system.started is True
    assert system.stopped is True


def test_no_repl_clean_gateway_shutdown_exits_zero(monkeypatch, tmp_path):
    import cli.main as main_mod

    gateway = _FakeGateway(run_result=True)
    system = _FakeSystem(gateway)
    _install_cli_harness(monkeypatch, tmp_path, system)
    monkeypatch.setattr(sys, "argv", ["lingtu.py", "stub", "--no-repl"])

    main_mod.main()

    assert gateway._defer_server is True
    assert gateway.run_server_called is True
    assert system.started is True
    assert system.stopped is True
