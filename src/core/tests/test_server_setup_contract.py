from __future__ import annotations

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]


def test_server_setup_runs_multifloor_closure_without_robot_motion():
    script = (REPO_ROOT / "scripts/deploy/setup_server_ros_pct.sh").read_text(
        encoding="utf-8"
    )

    assert 'RUN_MULTIFLOOR="${LINGTU_RUN_MULTIFLOOR:-1}"' in script
    assert 'RUN_NAV_CORE="${LINGTU_RUN_NAV_CORE:-1}"' in script
    assert "build_nav_core_runtime" in script
    assert 'bash "${ROOT}/scripts/build_nav_core.sh" --clean' in script
    assert "building nav_core nanobind runtime for production local planning" in script
    assert "multi-floor exploration/local-planning closure gate" in script
    assert "--route matrix" in script
    assert "--frontier-loop" in script
    assert "--local-planner-backend nanobind" in script
    assert "--require-production-local-planner" in script
    assert "artifacts/server_sim_closure/multifloor_exploration/report.json" in script
    assert "cmd_vel" in script
    assert "physical hardware" in script
