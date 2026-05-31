"""Contract tests: verify all semantic subpackages import without error."""


def test_import_semantic_perception():
    """semantic.perception package is importable."""
    import semantic.perception  # noqa: F401


def test_import_semantic_planner():
    """semantic.planner package is importable."""
    import semantic.planner  # noqa: F401


def test_import_semantic_reconstruction():
    """semantic.reconstruction package is importable."""
    import semantic.reconstruction  # noqa: F401


def test_import_all_subpackages():
    """All three semantic subpackages can be imported in one pass."""
    from semantic import perception, planner, reconstruction

    assert perception is not None
    assert planner is not None
    assert reconstruction is not None
