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
skips = [r for r in results if r.get("status") == "skip"]
hard_errors = [r for r in results if r.get("status") == "fail"]

if hard_errors:
    print("structured benchmark has unexpected failures:", file=sys.stderr)
    for r in hard_errors:
        print(f"  planner={r['planner']} route={r.get('route','?')} error={r['error']}", file=sys.stderr)

print(
    f"results={len(results)} pass={len([r for r in results if r.get('status') == 'pass'])} "
    f"skip={len(skips)} fail={len(hard_errors)} routes={report.get('routes')} "
    f"planners={report.get('planners')}"
)
for r in results:
    if r.get("status") == "skip":
        print(f"  SKIP planner={r['planner']} route={r.get('route','?')} {r.get('error')}")
    elif r.get("status") == "fail":
        print(f"  FAIL planner={r['planner']} route={r.get('route','?')} {r['error']}")
    else:
        print(f"  PASS planner={r['planner']} route={r.get('route','?')} "
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

STEP2_LOG="${OUT_DIR}/large_terrain_validation.log"
set +e
PYTHONPATH="${ROOT_DIR}/src:${ROOT_DIR}:${PYTHONPATH:-}" \
  "${PYTHON_BIN}" sim/scripts/large_terrain_nav_validation.py \
    --output-dir "${OUT_DIR}" \
    --planners pct,astar \
    --json-out "${REPORT}" >"${STEP2_LOG}" 2>&1
STEP2_STATUS=$?
set -e
cat "${STEP2_LOG}"
if [ "${STEP2_STATUS}" -ne 0 ]; then
  if grep -E "ModuleNotFoundError|No module named|planner unavailable|missing dependency" "${STEP2_LOG}" >/dev/null 2>&1; then
    "${PYTHON_BIN}" - "${REPORT}" "${STEP2_LOG}" <<'PYSKIP'
import json
import sys
import time
from pathlib import Path

report_path = Path(sys.argv[1])
log_path = Path(sys.argv[2])
payload = {
    "schema_version": "lingtu.large_terrain_nav_validation.skip.v1",
    "ok": False,
    "status": "skip",
    "synthetic": True,
    "claim_scope": "planner_regression_only",
    "freshness": {"generated_at": time.time(), "max_age_s": 86400.0},
    "blockers": [log_path.read_text(encoding="utf-8", errors="replace").strip().splitlines()[-1]],
}
report_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print("large-terrain validation SKIP: dependency unavailable")
PYSKIP
  else
    exit "${STEP2_STATUS}"
  fi
fi

"${PYTHON_BIN}" - "${REPORT}" <<'PY'
import json
import sys
from pathlib import Path

report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
if report.get("status") == "skip":
    print("large-terrain validation SKIP:", json.dumps(report.get("blockers") or [], ensure_ascii=False))
    raise SystemExit(0)
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
