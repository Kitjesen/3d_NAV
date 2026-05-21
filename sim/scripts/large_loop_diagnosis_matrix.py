#!/usr/bin/env python3
"""Summarize large-loop control-run reports into a diagnosis matrix.

This script does not run the simulator. It compares already generated runtime
reports so a failed large-loop gate can be classified without reading scattered
logs by hand.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _load_report(path: Path | str | None) -> dict[str, Any] | None:
    if path is None:
        return None
    report_path = Path(path)
    if not report_path.exists():
        return {
            "_missing": True,
            "_path": str(report_path),
        }
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    if isinstance(payload, dict):
        payload["_path"] = str(report_path)
        return payload
    return {
        "_path": str(report_path),
        "_invalid": True,
        "_type": type(payload).__name__,
    }


def _as_bool(value: Any) -> bool:
    return bool(value is True)


def _as_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result


def _as_int(value: Any) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _last_nav_diagnostic(report: dict[str, Any]) -> dict[str, Any]:
    diagnostics = report.get("navigation_diagnostics")
    if not isinstance(diagnostics, dict):
        return {}
    last = diagnostics.get("last_sample")
    if isinstance(last, dict) and last:
        return last
    tail = diagnostics.get("samples_tail")
    if isinstance(tail, list) and tail and isinstance(tail[-1], dict):
        return tail[-1]
    samples = diagnostics.get("samples")
    if isinstance(samples, list) and samples and isinstance(samples[-1], dict):
        return samples[-1]
    return {}


def _report_evidence(report: dict[str, Any] | None) -> dict[str, Any]:
    if report is None:
        return {
            "present": False,
        }
    if report.get("_missing"):
        return {
            "present": False,
            "path": report.get("_path"),
            "missing": True,
        }
    z_check = report.get("fastlio2_z_consistency")
    if not isinstance(z_check, dict):
        z_check = {}
    inspection = report.get("lingtu_inspection")
    if not isinstance(inspection, dict):
        inspection = {}
    outputs = report.get("outputs")
    if not isinstance(outputs, dict):
        outputs = {}
    commanded = report.get("commanded_sim_velocity")
    if not isinstance(commanded, dict):
        commanded = {}
    last = _last_nav_diagnostic(report)
    nav_cmd = last.get("nav_cmd") if isinstance(last.get("nav_cmd"), dict) else {}
    nav = last.get("navigation") if isinstance(last.get("navigation"), dict) else {}
    paths = last.get("paths") if isinstance(last.get("paths"), dict) else {}
    diagnostic_report = report.get("fastlio_large_loop_diagnostic_report")
    if not isinstance(diagnostic_report, dict):
        diagnostic_report = {}
    global_path_count = max(
        _as_int(paths.get("global_path_count")),
        _as_int(inspection.get("global_path_count")),
    )
    local_path_count = max(
        _as_int(paths.get("local_path_count")),
        _as_int(inspection.get("local_path_count")),
    )

    return {
        "present": True,
        "path": report.get("_path"),
        "ok": _as_bool(report.get("ok")),
        "nav_data_source": str(report.get("nav_data_source") or ""),
        "drive_source": str(report.get("drive_source") or commanded.get("source") or ""),
        "z_ok": _as_bool(z_check.get("ok")),
        "z_delta_error_m": _as_float(z_check.get("z_delta_error_m")),
        "z_limit_m": _as_float(z_check.get("max_allowed_z_drift_m")),
        "inspection_enabled": _as_bool(inspection.get("enabled")),
        "inspection_verified": _as_bool(inspection.get("verified")),
        "inspection_state": str(inspection.get("patrol_state") or ""),
        "successful_goal_count": _as_int(
            inspection.get("successful_navigation_goal_count")
        ),
        "goal_count": _as_int(inspection.get("goal_count")),
        "global_planner": str(inspection.get("global_planner") or ""),
        "sim_path_length_m": _as_float(report.get("sim_path_length_m")),
        "fastlio2_path_length_m": _as_float(report.get("fastlio2_path_length_m")),
        "nav_cmd_vel_nonzero": _as_int(outputs.get("nav_cmd_vel_nonzero")),
        "nav_cmd_fresh": _as_bool(nav_cmd.get("fresh")),
        "nav_cmd_linear_norm": _as_float(nav_cmd.get("linear_norm")),
        "navigation_state": str(nav.get("state") or ""),
        "primary_planner": str(nav.get("primary_planner") or ""),
        "selected_planner": str(nav.get("selected_planner") or ""),
        "global_path_count": global_path_count,
        "local_path_count": local_path_count,
        "latest_runtime_fault": str(
            last.get("latest_runtime_fault") or "; ".join(report.get("runtime_faults") or [])
        ),
        "diagnostic_report": diagnostic_report,
    }


def _has_z_failure(evidence: dict[str, Any]) -> bool:
    if not evidence.get("present"):
        return False
    if evidence.get("z_ok") is False:
        return True
    z_error = evidence.get("z_delta_error_m")
    z_limit = evidence.get("z_limit_m")
    return bool(z_error is not None and z_limit is not None and z_error > z_limit)


def _truth_nav_planning_failed(evidence: dict[str, Any]) -> bool:
    if not evidence.get("present") or evidence.get("inspection_verified"):
        return False
    has_path_and_command = bool(
        evidence.get("global_path_count", 0) > 0
        and evidence.get("local_path_count", 0) > 0
        and evidence.get("nav_cmd_vel_nonzero", 0) > 0
    )
    state = str(evidence.get("navigation_state") or evidence.get("inspection_state") or "").upper()
    latest_fault = str(evidence.get("latest_runtime_fault") or "").lower()
    if has_path_and_command and state in {"PATROLLING", "RUNNING", "TRACKING"}:
        return False
    if has_path_and_command and "fast-lio" in latest_fault:
        return False
    return True


def _blocking_failures(
    *,
    baseline: dict[str, Any],
    truth_nav: dict[str, Any],
    fixed_fastlio: dict[str, Any],
) -> list[str]:
    failures: list[str] = []
    if _truth_nav_planning_failed(truth_nav):
        failures.append("planning_tracking_or_frame")
    if _has_z_failure(baseline) and (
        truth_nav.get("inspection_verified") or _has_z_failure(fixed_fastlio)
    ):
        failures.append("slam_localization")
    elif _has_z_failure(baseline):
        failures.append("slam_localization_unisolated")
    if baseline.get("local_path_count", 0) > 0 and not baseline.get("nav_cmd_fresh"):
        failures.append("command_freshness_or_mux")
    return failures


def _classify(
    *,
    baseline: dict[str, Any],
    truth_nav: dict[str, Any],
    fixed_fastlio: dict[str, Any],
) -> str:
    if baseline.get("inspection_verified") and not _has_z_failure(baseline):
        return "none"
    failures = _blocking_failures(
        baseline=baseline,
        truth_nav=truth_nav,
        fixed_fastlio=fixed_fastlio,
    )
    if failures:
        return failures[0]
    return "unknown"


def _recommend(primary_failure: str) -> str:
    if primary_failure == "none":
        return "Large-loop controls do not show a blocking failure."
    if primary_failure == "slam_localization":
        return (
            "Focus the next run on Fast-LIO Z drift: fixed-motion Fast-LIO, "
            "scan timing profile, and IMU specific-force diagnostics."
        )
    if primary_failure == "planning_tracking_or_frame":
        return (
            "Focus the next run on truth-nav planning/tracking/frame alignment: "
            "inspect PCT path terminal-to-goal, goal frame metadata, and local path output."
        )
    if primary_failure == "command_freshness_or_mux":
        return (
            "Focus the next run on cmd freshness and mux/path follower handoff: "
            "record stale-zeroed intervals and active cmd source."
        )
    if primary_failure == "slam_localization_unisolated":
        return (
            "Run truth-nav and fixed-motion controls before changing thresholds; "
            "baseline shows Fast-LIO drift but isolation is incomplete."
        )
    return "Run the full baseline/truth-nav/fixed Fast-LIO control matrix."


def evaluate_matrix(
    *,
    baseline_fastlio: Path | str,
    truth_nav: Path | str | None = None,
    fixed_fastlio: Path | str | None = None,
) -> dict[str, Any]:
    baseline_evidence = _report_evidence(_load_report(baseline_fastlio))
    truth_evidence = _report_evidence(_load_report(truth_nav))
    fixed_evidence = _report_evidence(_load_report(fixed_fastlio))
    blocking_failures = _blocking_failures(
        baseline=baseline_evidence,
        truth_nav=truth_evidence,
        fixed_fastlio=fixed_evidence,
    )
    primary_failure = _classify(
        baseline=baseline_evidence,
        truth_nav=truth_evidence,
        fixed_fastlio=fixed_evidence,
    )
    return {
        "schema_version": "lingtu.large_loop_diagnosis_matrix.v1",
        "ok": primary_failure == "none",
        "primary_failure": primary_failure,
        "blocking_failures": blocking_failures,
        "recommended_next_step": _recommend(primary_failure),
        "evidence": {
            "baseline_fastlio": baseline_evidence,
            "truth_nav": truth_evidence,
            "fixed_fastlio": fixed_evidence,
        },
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Summarize large-loop baseline/truth/fixed control reports."
    )
    parser.add_argument("--baseline-fastlio", required=True)
    parser.add_argument("--truth-nav", default="")
    parser.add_argument("--fixed-fastlio", default="")
    parser.add_argument("--json-out", default="")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    summary = evaluate_matrix(
        baseline_fastlio=args.baseline_fastlio,
        truth_nav=args.truth_nav or None,
        fixed_fastlio=args.fixed_fastlio or None,
    )
    text = json.dumps(summary, indent=2, sort_keys=True)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
