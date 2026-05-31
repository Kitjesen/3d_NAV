"""Contract tests: verify global_planning subpackages import without error."""


def test_import_pct_adapters():
    """global_planning.pct_adapters package is importable."""
    import global_planning.pct_adapters  # noqa: F401


def test_import_pct_adapters_module():
    """pct_adapters.global_planner_module is importable."""
    from global_planning.pct_adapters import global_planner_module

    assert global_planner_module is not None


def test_import_pct_planner_config():
    """pct_planner.planner.config.Config is importable."""
    from global_planning.pct_planner.planner.config import Config

    assert Config is not None


def test_import_pct_planner_runnable():
    """global_planning.pct_planner_runnable is importable."""
    import global_planning.pct_planner_runnable  # noqa: F401


def test_import_all_subpackages():
    """All global_planning subpackages import cleanly together."""
    from global_planning import pct_adapters, pct_planner_runnable

    assert pct_adapters is not None
    assert pct_planner_runnable is not None
