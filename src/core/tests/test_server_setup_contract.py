from __future__ import annotations

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]


def test_server_setup_runs_multifloor_closure_without_robot_motion():
    script = (REPO_ROOT / "scripts/deploy/setup_server_ros_pct.sh").read_text(
        encoding="utf-8"
    )

    assert 'RUN_MULTIFLOOR="${LINGTU_RUN_MULTIFLOOR:-1}"' in script
    assert 'RUN_NAV_CORE="${LINGTU_RUN_NAV_CORE:-1}"' in script
    assert 'RUN_ROUTECHECK_PREFLIGHT="${LINGTU_RUN_ROUTECHECK_PREFLIGHT:-1}"' in script
    assert 'SETUP_CLOSURE_MAX_REPORT_AGE_S="${LINGTU_SETUP_CLOSURE_MAX_REPORT_AGE_S:-21600}"' in script
    assert "build_nav_core_runtime" in script
    assert 'bash "${ROOT}/scripts/build_nav_core.sh" --clean' in script
    assert "building nav_core nanobind runtime for production local planning" in script
    assert "multi-floor exploration/local-planning closure gate" in script
    assert "Gateway routecheck preflight closure gate, non-motion" in script
    assert "sim/scripts/routecheck_preflight_gate.py" in script
    assert "server simulation closure summary for setup-generated gates" in script
    assert "sim/scripts/server_sim_closure.py" in script
    assert "artifacts/server_sim_closure_summary_setup.json" in script
    assert "--max-report-age-s" in script
    assert '"${SETUP_CLOSURE_MAX_REPORT_AGE_S}"' in script
    assert "--route matrix" in script
    assert "--frontier-loop" in script
    assert "--local-planner-backend nanobind" in script
    assert "--require-production-local-planner" in script
    assert "artifacts/server_sim_closure/multifloor_exploration/report.json" in script
    assert "cmd_vel" in script
    assert "physical hardware" in script


def test_p0_scripts_use_current_gateway_contracts():
    goto = (REPO_ROOT / "docs/07-testing/p0_goto.sh").read_text(
        encoding="utf-8"
    )
    estop = (REPO_ROOT / "docs/07-testing/p0_estop.sh").read_text(
        encoding="utf-8"
    )
    mapping = (REPO_ROOT / "docs/07-testing/p0_mapping.sh").read_text(
        encoding="utf-8"
    )

    assert "/api/v1/navigation/status" in goto
    assert "/api/v1/nav/status" not in goto
    assert '\\"frame_id\\":\\"map\\"' in goto
    assert '\\"client_id\\":\\"p0_goto\\"' in goto

    assert "POST /api/v1/stop" in estop
    assert "GET /api/v1/state" in estop
    assert "/api/v1/safety/state" not in estop
    assert "curl -sf http://localhost:5050/api/v1/cmd_vel" not in estop

    assert "/api/v1/maps" in mapping
    assert '\\"action\\":\\"save\\"' in mapping
    assert '\\"action\\":\\"set_active\\"' in mapping
    assert "/api/v1/map/save" not in mapping
    assert "/api/v1/map/activate" not in mapping
    assert "SAVE_PATH=" in mapping
    assert 'd.get("map_dir") or d.get("path")' in mapping
    assert "NAV_MAP_DIR" in mapping
    assert "~/data/nova/maps" in mapping
    assert "data/inovxio/data/maps" not in mapping


def test_l2_hook_runs_stub_build_and_offline_start_smoke():
    hook = (REPO_ROOT / "docs/07-testing/install_hooks.sh").read_text(
        encoding="utf-8"
    )

    assert "running stub blueprint smoke" in hook
    assert "stub profile build OK" in hook
    assert "stub profile start OK" in hook
    assert "runtime.start()" in hook
    assert "runtime.stop()" in hook
    assert "failed_modules" in hook
    assert "enable_native=False" in hook
    assert "enable_gateway=False" in hook
    assert "enable_map_modules=False" in hook
    assert "run_startup_checks=False" in hook
    assert "profile='stub'" not in hook
    assert 'profile="stub"' not in hook


def test_p0_field_runbook_matches_script_contracts():
    readme = (REPO_ROOT / "docs/07-testing/README.md").read_text(
        encoding="utf-8"
    )
    p0_scripts = [
        REPO_ROOT / "docs/07-testing/p0_all.sh",
        REPO_ROOT / "docs/07-testing/p0_cold_boot.sh",
        REPO_ROOT / "docs/07-testing/p0_estop.sh",
        REPO_ROOT / "docs/07-testing/p0_goto.sh",
        REPO_ROOT / "docs/07-testing/p0_mapping.sh",
    ]

    assert "/api/v1/navigation/status" in readme
    assert "POST /api/v1/stop" in readme
    assert "/api/v1/state" in readme
    assert "CmdVelMux` outputs `Twist.zero()`" not in readme
    assert "watchdog log entry" not in readme

    for script in p0_scripts:
        script.read_text(encoding="utf-8").encode("ascii")
