#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT_DIR="${ROOT_DIR}/artifacts/benchmark_planner"
REPORT="${OUT_DIR}/report.json"
STRUCTURED_REPORT="${OUT_DIR}/report_structured.json"

cd "${ROOT_DIR}"
mkdir -p "${OUT_DIR}"

echo "=========================================="
echo "Planner performance benchmark"
echo "=========================================="

PYTHON_BIN="${PYTHON:-python3}"
if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
  echo "Python executable not found: ${PYTHON_BIN}" >&2
  exit 127
fi

# ------------------------------------------------------------------ #
# Step 1: Structured benchmark (lightweight, no ROS2 needed)         #
# ------------------------------------------------------------------ #
echo ""
echo "--- Step 1: Structured planner benchmark ---"
echo "Output: ${STRUCTURED_REPORT}"

PYTHONPATH="${ROOT_DIR}/src:${ROOT_DIR}:${PYTHONPATH:-}" \
  "${PYTHON_BIN}" tests/benchmark/benchmark_planner_structured.py \
    --output-dir "${OUT_DIR}" \
    --json-out "${STRUCTURED_REPORT}"

# Validate structured report
"${PYTHON_BIN}" - "${STRUCTURED_REPORT}" <<'PYCHECK'
import json, sys
from pathlib import Path

report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
results = report.get("results") or []
errors = [r for r in results if r.get("error")]
warnings = [r for r in errors if "pct planner unavailable" in r.get("error", "")]
hard_errors = [r for r in errors if "pct planner unavailable" not in r.get("error", "")]

if hard_errors:
    print("structured benchmark has unexpected failures:", file=sys.stderr)
    for r in hard_errors:
        print(f"  planner={r['planner']} route={r.get('route','?')} error={r['error']}", file=sys.stderr)

print(f"results={len(results)} routes={report.get('routes')} planners={report.get('planners')}")
for r in results:
    if r.get("error"):
        tag = "OK" if "pct planner unavailable" in r["error"] else "FAIL"
        print(f"  {tag} planner={r['planner']} route={r.get('route','?')} {r['error']}")
    else:
        print(f"  OK  planner={r['planner']} route={r.get('route','?')} "
              f"latency_ms={r['latency_ms']} path_m={r['path_length_m']} "
              f"waypoints={r['waypoint_count']} safety={r['path_safety_score']}")

if hard_errors:
    raise SystemExit(1)
PYCHECK

# ------------------------------------------------------------------ #
# Step 2: Large-terrain validation (requires simulation assets)      #
# ------------------------------------------------------------------ #
echo ""
echo "--- Step 2: Large-terrain navigation validation ---"
echo "Output: ${REPORT}"

PYTHONPATH="${ROOT_DIR}/src:${ROOT_DIR}:${PYTHONPATH:-}" \
  "${PYTHON_BIN}" sim/scripts/large_terrain_nav_validation.py \
    --output-dir "${OUT_DIR}" \
    --planners pct,astar \
    --json-out "${REPORT}"

"${PYTHON_BIN}" - "${REPORT}" <<'PY'
import json
import sys
from pathlib import Path

report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
if report.get("ok") is not True:
    print("planner benchmark failed: report.ok is not true", file=sys.stderr)
    print(json.dumps(report.get("blockers") or [], ensure_ascii=False), file=sys.stderr)
    raise SystemExit(1)

cases = report.get("cases") or []
print(f"cases={len(cases)} passed={report.get('passed_case_count')}")
for case in cases[:5]:
    print(
        "case={name} planner={planner} ok={ok} plan_ms={plan_ms}".format(
            name=case.get("name", "?"),
            planner=case.get("planner") or case.get("global_planner") or "?",
            ok=case.get("ok"),
            plan_ms=case.get("plan_ms", case.get("planning_time_ms", "?")),
        )
    )
PY

echo ""
echo "Benchmark complete. See ${REPORT} and ${STRUCTURED_REPORT}"
