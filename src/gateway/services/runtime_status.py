"""App/Web runtime status builders for localization and navigation."""

from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Any


LOCALIZATION_STATUS_SCHEMA_VERSION = 1
NAVIGATION_STATUS_SCHEMA_VERSION = 1

PATH_ENDPOINT = "/api/v1/path"

MISSION_ACTIVE_STATES = {"PLANNING", "EXECUTING", "PATROLLING"}
MISSION_TERMINAL_STATES = {"SUCCESS", "FAILED", "CANCELLED"}

CONTROL_SOURCE_META: dict[str, dict[str, Any]] = {
    "teleop": {
        "label": "Teleop joystick",
        "category": "manual",
        "owner": "teleop",
        "preempts_autonomy": True,
    },
    "visual_servo": {
        "label": "Visual servo",
        "category": "autonomy_assist",
        "owner": "visual_servo",
        "preempts_autonomy": True,
    },
    "recovery": {
        "label": "Navigation recovery",
        "category": "autonomy_recovery",
        "owner": "navigation",
        "preempts_autonomy": False,
    },
    "path_follower": {
        "label": "Path follower",
        "category": "autonomy",
        "owner": "navigation",
        "preempts_autonomy": False,
    },
}


def _mapping(value: Any) -> dict[str, Any]:
    if isinstance(value, Mapping):
        return dict(value)
    return {}


def _as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_float(value: Any, default: float | None = None) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _as_optional_int(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _as_optional_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes", "y"}:
            return True
        if lowered in {"false", "0", "no", "n"}:
            return False
    return None


def _reason_code_from_text(prefix: str, text: str) -> str:
    words: list[str] = []
    current: list[str] = []
    for ch in text.lower():
        if ch.isalnum():
            current.append(ch)
        elif current:
            words.append("".join(current))
            current = []
    if current:
        words.append("".join(current))
    if not words:
        return prefix
    return f"{prefix}_{'_'.join(words[:6])}"


def safe_session(gw: Any) -> dict[str, Any]:
    try:
        snapshot = gw._session_snapshot()
        if isinstance(snapshot, Mapping):
            return dict(snapshot)
    except Exception:
        pass
    return {
        "mode": getattr(gw, "_session_mode", "unknown"),
        "active_map": getattr(gw, "_session_map", None),
        "pending": bool(getattr(gw, "_session_pending", False)),
        "error": "session_snapshot_unavailable",
        "can_start_mapping": False,
        "can_start_navigating": False,
        "can_start_exploring": False,
        "can_end": False,
    }


def safe_lease(gw: Any) -> dict[str, Any]:
    lease = getattr(gw, "_lease", None)
    if hasattr(lease, "to_dict"):
        try:
            data = lease.to_dict()
            if isinstance(data, Mapping):
                return dict(data)
        except Exception:
            pass
    return {}


def _reported_state(raw: Any) -> str:
    state = str(raw or "").strip()
    return state.upper() if state else ""


def _localization_state(
    odometry: Any,
    session: Mapping[str, Any],
    icp_quality: float,
    diagnostics: Mapping[str, Any],
) -> tuple[str, list[str]]:
    reasons: list[str] = []
    mode = str(session.get("mode", "unknown"))
    ready = bool(session.get("localizer_ready", False))
    pending = bool(session.get("pending", False))
    reported = _reported_state(diagnostics.get("state"))
    degeneracy = _reported_state(diagnostics.get("degeneracy"))
    confidence = diagnostics.get("confidence")

    if odometry is None:
        reasons.append("odometry_missing")
        return "no_odometry", reasons

    if "RELOCAL" in reported or (pending and mode == "navigating"):
        reasons.append("relocalization_pending")
        return "relocalizing", reasons

    if reported in {"LOST", "UNINIT", "UNINITIALIZED"}:
        reasons.append(f"reported_state:{reported.lower()}")
        return "lost", reasons

    if (
        reported in {"DEGRADED", "FALLBACK_GNSS_ONLY"}
        or degeneracy in {"MILD", "MODERATE", "SEVERE"}
        or isinstance(confidence, (int, float)) and confidence < 0.5
    ):
        if reported:
            reasons.append(f"reported_state:{reported.lower()}")
        if degeneracy and degeneracy != "NONE":
            reasons.append(f"degeneracy:{degeneracy.lower()}")
        if isinstance(confidence, (int, float)) and confidence < 0.5:
            reasons.append("low_confidence")
        return "degraded", reasons

    if ready:
        return "ready", reasons

    if mode == "navigating":
        reasons.append("localizer_not_ready")
        return "initializing" if icp_quality <= 0.0 else "degraded", reasons

    if reported in {"TRACKING", "OK", "READY"}:
        return "tracking", reasons

    return "tracking", reasons


def build_localization_status_from_parts(
    odometry: Any,
    session: Mapping[str, Any],
    icp_quality: float,
    status: Any,
) -> dict[str, Any]:
    diagnostics = _mapping(status)
    state, reasons = _localization_state(
        odometry,
        session,
        float(icp_quality),
        diagnostics,
    )
    ready = state == "ready"
    return {
        "schema_version": LOCALIZATION_STATUS_SCHEMA_VERSION,
        "state": state,
        "ready": ready,
        "has_odometry": odometry is not None,
        "session_mode": session.get("mode"),
        "active_map": session.get("active_map"),
        "icp_quality": float(icp_quality),
        "reported_state": diagnostics.get("state"),
        "confidence": diagnostics.get("confidence"),
        "degeneracy": diagnostics.get("degeneracy"),
        "icp_fitness": _as_float(diagnostics.get("icp_fitness")),
        "effective_ratio": _as_float(diagnostics.get("effective_ratio")),
        "condition_number": _as_float(diagnostics.get("condition_number")),
        "degenerate_dof_count": _as_optional_int(
            diagnostics.get("degenerate_dof_count")
        ),
        "pos_cov_trace": _as_float(diagnostics.get("pos_cov_trace")),
        "ieskf_iter_num": _as_optional_int(diagnostics.get("ieskf_iter_num")),
        "ieskf_converged": _as_optional_bool(diagnostics.get("ieskf_converged")),
        "localizer_health": diagnostics.get("localizer_health"),
        "localizer_health_fitness": _as_float(
            diagnostics.get("localizer_health_fitness")
        ),
        "localizer_health_iter": _as_optional_int(
            diagnostics.get("localizer_health_iter")
        ),
        "localizer_health_cov_trace": _as_float(
            diagnostics.get("localizer_health_cov_trace")
        ),
        "ts": diagnostics.get("ts"),
        "can_relocalize": state in {"degraded", "lost"} and odometry is not None,
        "reasons": reasons,
        "raw": diagnostics,
    }


def build_localization_status(gw: Any) -> dict[str, Any]:
    with gw._state_lock:
        odometry = gw._odom
        localization_status = getattr(gw, "_localization_status", None)

    session = safe_session(gw)
    return build_localization_status_from_parts(
        odometry,
        session,
        float(getattr(gw, "_icp_quality", 0.0)),
        localization_status,
    )


def _cmd_vel_health(gw: Any) -> dict[str, Any]:
    modules = getattr(gw, "_all_modules", None) or {}
    mux = modules.get("CmdVelMux")
    if mux is None:
        return {"active_source": "unknown", "sources": {}, "available": False}
    try:
        health = mux.health() if hasattr(mux, "health") else {}
        if isinstance(health, Mapping):
            data = dict(health)
            data["available"] = True
            data.setdefault("active_source", "unknown")
            data.setdefault("sources", {})
            return data
    except Exception as exc:
        return {
            "active_source": "unknown",
            "sources": {},
            "available": False,
            "error": str(exc),
        }
    return {"active_source": "unknown", "sources": {}, "available": False}


def _control_summary(
    *,
    mode: str,
    lease: Mapping[str, Any],
    mission_state: str,
    cmd_vel: Mapping[str, Any],
) -> dict[str, Any]:
    active_source = str(cmd_vel.get("active_source") or "none")
    sources = _mapping(cmd_vel.get("sources"))
    active_source_health = _mapping(sources.get(active_source))
    meta = CONTROL_SOURCE_META.get(active_source, {})
    if active_source == "none":
        meta = {
            "label": "No active command source",
            "category": "none",
            "owner": "none",
            "preempts_autonomy": False,
        }
    elif not meta:
        meta = {
            "label": active_source,
            "category": "unknown",
            "owner": "unknown",
            "preempts_autonomy": False,
        }

    autonomy_requested = mission_state in MISSION_ACTIVE_STATES
    manual_override = active_source == "teleop"
    preempting_autonomy = bool(
        autonomy_requested and meta.get("preempts_autonomy", False)
    )

    return {
        "mode": mode,
        "lease": dict(lease),
        "active_cmd_source": active_source,
        "command_owner": meta["owner"],
        "source_category": meta["category"],
        "manual_override": manual_override,
        "autonomy_requested": autonomy_requested,
        "preempting_autonomy": preempting_autonomy,
        "mux_available": bool(cmd_vel.get("available", False)),
        "active_source": {
            "name": active_source,
            "label": meta["label"],
            "category": meta["category"],
            "owner": meta["owner"],
            "priority": active_source_health.get("priority"),
            "active": active_source_health.get("active", active_source != "none"),
            "age_ms": active_source_health.get("age_ms"),
        },
        "sources": sources,
        "cmd_vel_mux": dict(cmd_vel),
    }


def _progress_summary(
    *,
    state: str,
    wp_index: int,
    wp_total: int,
    path_points: int,
    replan_count: int,
) -> dict[str, Any]:
    if state == "SUCCESS":
        fraction = 1.0
    elif wp_total > 0:
        fraction = max(0.0, min(1.0, wp_index / wp_total))
    else:
        fraction = 0.0

    return {
        "wp_index": wp_index,
        "wp_total": wp_total,
        "fraction": round(fraction, 4),
        "path_points": path_points,
        "replan_count": replan_count,
        "active": state in MISSION_ACTIVE_STATES,
        "terminal": state in MISSION_TERMINAL_STATES,
    }


def _navigation_reason_codes(
    *,
    state: str,
    failure_reason: str,
    has_odometry: bool,
    mode: str,
    session: Mapping[str, Any],
    localization: Mapping[str, Any],
    control: Mapping[str, Any],
) -> list[str]:
    codes: list[str] = []
    if not has_odometry:
        codes.append("odometry_missing")
    if mode == "estop":
        codes.append("estop_active")
    if bool(session.get("pending", False)):
        codes.append("session_transition_pending")

    localization_state = str(localization.get("state") or "unknown")
    if localization_state in {"degraded", "lost", "relocalizing", "initializing"}:
        codes.append(f"localization_{localization_state}")

    if state == "STUCK":
        codes.append("mission_stuck")
    elif state == "FAILED":
        codes.append("mission_failed")
    elif state == "CANCELLED":
        codes.append("mission_cancelled")

    if failure_reason:
        codes.append(_reason_code_from_text("failure", failure_reason))

    if control.get("preempting_autonomy"):
        codes.append(f"control_preempted_by_{control.get('active_cmd_source')}")
    if not control.get("mux_available", False):
        codes.append("cmd_vel_mux_unavailable")

    return list(dict.fromkeys(codes))


def _readiness_summary(
    *,
    can_accept_goal: bool,
    reason_codes: list[str],
    localization: Mapping[str, Any],
    control: Mapping[str, Any],
) -> dict[str, Any]:
    blockers = [
        code
        for code in reason_codes
        if code
        in {
            "odometry_missing",
            "estop_active",
            "session_transition_pending",
            "localization_lost",
            "localization_relocalizing",
            "localization_initializing",
        }
    ]
    advisories = [code for code in reason_codes if code not in blockers]
    return {
        "can_accept_goal": can_accept_goal,
        "can_execute_autonomy": not blockers,
        "blockers": blockers,
        "advisories": advisories,
        "localization_ready": bool(localization.get("ready", False)),
        "control_owner": control.get("command_owner", "unknown"),
    }


def build_navigation_status(gw: Any) -> dict[str, Any]:
    with gw._state_lock:
        mission = _mapping(gw._mission)
        odometry = gw._odom
        path_len = len(gw._last_path)
        mode = gw._mode

    session = safe_session(gw)
    lease = safe_lease(gw)
    localization = build_localization_status(gw)
    cmd_vel = _cmd_vel_health(gw)
    state = str(mission.get("state", "IDLE"))
    wp_index = _as_int(mission.get("wp_index"), 0)
    wp_total = _as_int(mission.get("wp_total"), 0)
    replan_count = _as_int(mission.get("replan_count"), 0)
    speed_scale = _as_float(mission.get("speed_scale"), None)
    failure_reason = str(mission.get("failure_reason", "") or "")
    degraded = localization.get("state") in {
        "no_odometry",
        "initializing",
        "degraded",
        "lost",
        "relocalizing",
    }
    can_accept_goal = (
        mode != "estop"
        and odometry is not None
        and not bool(session.get("pending", False))
    )
    control = _control_summary(
        mode=mode,
        lease=lease,
        mission_state=state,
        cmd_vel=cmd_vel,
    )
    progress = _progress_summary(
        state=state,
        wp_index=wp_index,
        wp_total=wp_total,
        path_points=path_len,
        replan_count=replan_count,
    )
    reason_codes = _navigation_reason_codes(
        state=state,
        failure_reason=failure_reason,
        has_odometry=odometry is not None,
        mode=mode,
        session=session,
        localization=localization,
        control=control,
    )
    readiness = _readiness_summary(
        can_accept_goal=can_accept_goal,
        reason_codes=reason_codes,
        localization=localization,
        control=control,
    )

    return {
        "schema_version": NAVIGATION_STATUS_SCHEMA_VERSION,
        "state": state,
        "has_odometry": odometry is not None,
        "can_accept_goal": can_accept_goal,
        "wp_index": wp_index,
        "wp_total": wp_total,
        "replan_count": replan_count,
        "speed_scale": speed_scale,
        "failure_reason": failure_reason,
        "reason_codes": reason_codes,
        "readiness": readiness,
        "progress": progress,
        "path": {
            "points": path_len,
            "endpoint": PATH_ENDPOINT,
        },
        "control": control,
        "localization": {
            "state": localization.get("state"),
            "ready": localization.get("ready"),
            "degraded": degraded,
            "speed_scale": speed_scale,
            "reasons": localization.get("reasons", []),
        },
        "diagnostics": {
            "reason_codes": reason_codes,
            "failure_reason": failure_reason,
            "localization_reasons": localization.get("reasons", []),
            "cmd_vel_mux_available": control.get("mux_available", False),
        },
        "mission": {
            "state": state,
            "raw": mission,
        },
        "ts": float(mission.get("ts", time.time()) or time.time()),
    }
