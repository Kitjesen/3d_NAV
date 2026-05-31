"""Contract tests: verify the lingtu import-ready API is accessible."""


def test_import_lingtu_package():
    """The lingtu package is importable."""
    import lingtu  # noqa: F401


def test_import_lingtu_classes():
    """All six public API classes import correctly."""
    from lingtu import SLAM, Camera, Detector, LiDAR, Navigator, Robot

    assert SLAM is not None
    assert Camera is not None
    assert Detector is not None
    assert LiDAR is not None
    assert Navigator is not None
    assert Robot is not None


def test_import_lingtu_all():
    """lingtu.__all__ matches the public API."""
    import lingtu

    expected = {"SLAM", "Camera", "Detector", "LiDAR", "Navigator", "Robot"}
    assert set(lingtu.__all__) == expected, (
        f"__all__ mismatch: got {set(lingtu.__all__)}, expected {expected}"
    )
