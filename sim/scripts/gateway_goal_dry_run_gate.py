#!/usr/bin/env python3
"""Validate Gateway goal command flow without any robot driver.

The gate calls Gateway route handlers in-process. It does not start a server,
does not create a driver module, and does not publish hardware cmd_vel.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


class _FakePlanPreviewNav:
    def __init__(self) -> None:
        self.calls: list[tuple[float, float, float]] = []

    def preview_plan(self, x: float, y: float, z: float) -> dict[str, Any]:
        self.calls.append((float(x), float(y), float(z)))
        ts = time.time()
        return {
            "schema_version": 1,
            "ok": True,
            "feasible": True,
            "frame_id": "map",
            "start": {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map"},
            "goal": {"x": float(x), "y": float(y), "z": float(z), "frame_id": "map"},
            "path": [
                {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map"},
                {"x": float(x), "y": float(y), "z": float(z), "frame_id": "map"},
            ],
            "reasons": [],
            "error": None,
            "ts": ts,
        }


def _endpoint(gateway: Any, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _mark_navigation_ready(gateway: Any) -> None:
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    odom = {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "ts": time.time(),
        "frame_id": "map",
    }
    mission = {"state": "IDLE"}
    localization_status = {
        "state": "TRACKING",
        "confidence": 0.9,
        "degeneracy": "NONE",
        "odom_age_ms": 100.0,
        "localizer_health": "RECOVERED",
    }
    lock = getattr(gateway, "_state_lock", None)
    if lock is not None:
        with lock:
            gateway._odom = odom
            gateway._mission = mission
            gateway._localization_status = localization_status
    else:
        gateway._latest_odom = odom
        gateway._latest_mission = mission
        gateway._latest_safety = {"level": "ok"}


def run_gate(*, x: float, y: float, z: float, client_id: str) -> dict[str, Any]:
    try:
        from gateway.gateway_module import GatewayModule
    except Exception as exc:
        return {
            "schema_version": "lingtu.gateway_goal_dry_run_gate.v1",
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "error": f"gateway imports unavailable: {exc}",
        }
    try:
        from gateway.schemas import GoalRequest, PlanPreviewRequest
    except Exception:
        GoalRequest = None
        PlanPreviewRequest = None

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)

    sent_goals = []
    sent_cmd_vel = []
    sent_stops = []
    gateway.goal_pose._add_callback(sent_goals.append)
    gateway.cmd_vel._add_callback(sent_cmd_vel.append)
    gateway.stop_cmd._add_callback(sent_stops.append)

    goal_endpoint = _endpoint(gateway, "/api/v1/goal")

    preview = {
        "schema_version": 1,
        "ok": True,
        "feasible": True,
        "frame_id": "map",
        "legacy_gateway_preview": True,
        "path": [
            {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map"},
            {"x": float(x), "y": float(y), "z": float(z), "frame_id": "map"},
        ],
    }
    route_paths = {route.path for route in gateway._app.routes}
    if "/api/v1/navigation/plan" in route_paths and PlanPreviewRequest is not None:
        preview_endpoint = _endpoint(gateway, "/api/v1/navigation/plan")
        preview = asyncio.run(
            preview_endpoint(
                PlanPreviewRequest(
                    x=float(x),
                    y=float(y),
                    z=float(z),
                    frame_id="map",
                    client_id=client_id,
                )
            )
        )

    goal_body: Any
    if GoalRequest is not None:
        goal_body = (
            GoalRequest(
                x=float(x),
                y=float(y),
                z=float(z),
                frame_id="map",
                client_id=client_id,
                request_id="server-sim-dry-run-goal",
            )
        )
    else:
        goal_body = {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "frame_id": "map",
            "client_id": client_id,
            "request_id": "server-sim-dry-run-goal",
        }
    goal = asyncio.run(goal_endpoint(goal_body))

    goal_frame = str(getattr(sent_goals[-1], "frame_id", "")) if sent_goals else ""
    goal_ok = bool(goal.get("ok")) or str(goal.get("status", "")).lower() == "ok"
    ok = (
        bool(preview.get("feasible"))
        and goal_ok
        and len(sent_goals) == 1
        and len(sent_cmd_vel) == 0
        and len(sent_stops) == 0
        and goal_frame == "map"
    )
    return {
        "schema_version": "lingtu.gateway_goal_dry_run_gate.v1",
        "ok": bool(ok),
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "driver_used": False,
        "gateway_used": True,
        "frames": {
            "goal": "map",
            "preview_path": str(preview.get("frame_id") or ""),
            "published_goal": goal_frame,
            "cmd_vel": "not_published",
        },
        "target": [float(x), float(y), float(z)],
        "preview": preview,
        "goal_response": goal,
        "nav_preview_calls": nav.calls,
        "published": {
            "goal_pose": len(sent_goals),
            "cmd_vel": len(sent_cmd_vel),
            "stop_cmd": len(sent_stops),
        },
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=float, default=1.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--client-id", default="server-sim")
    parser.add_argument("--json-out", default="")
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = run_gate(x=args.x, y=args.y, z=args.z, client_id=args.client_id)
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    return 0 if report.get("ok") or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
