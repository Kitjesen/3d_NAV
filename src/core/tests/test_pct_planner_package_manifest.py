from pathlib import Path


def test_pct_planner_cmake_installs_existing_legacy_scripts() -> None:
    repo = Path(__file__).resolve().parents[3]
    package_dir = repo / "src" / "global_planning" / "pct_planner"
    cmake = (package_dir / "CMakeLists.txt").read_text(encoding="utf-8")

    expected_scripts = [
        "../../../src/legacy/pct_planner/global_planner.py",
        "../../../src/legacy/pct_planner/pct_planner_astar.py",
        "../../../src/legacy/pct_planner/fake_localization.py",
    ]
    for script in expected_scripts:
        assert script in cmake
        assert (package_dir / script).is_file()

    assert "planner/scripts/global_planner.py" not in cmake
    assert "planner/scripts/pct_planner_astar.py" not in cmake
