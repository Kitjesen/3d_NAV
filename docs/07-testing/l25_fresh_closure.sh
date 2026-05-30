#!/usr/bin/env bash
# L2.5: strict server simulation closure with report freshness enforcement.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

MAX_REPORT_AGE_S="${LINGTU_L25_MAX_REPORT_AGE_S:-21600}"
JSON_OUT="${LINGTU_L25_JSON_OUT:-artifacts/server_sim_closure_summary_all.json}"
PYTHON_BIN="${PYTHON:-}"
if [[ -z "$PYTHON_BIN" ]]; then
  if command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="python3"
  elif command -v python >/dev/null 2>&1; then
    PYTHON_BIN="python"
  else
    echo "FAIL: no Python interpreter found (set PYTHON=/path/to/python)" >&2
    exit 127
  fi
fi

PYTHONPATH="${ROOT}/src:${ROOT}:${PYTHONPATH:-}" \
  "$PYTHON_BIN" sim/scripts/server_sim_closure.py \
    --max-report-age-s "$MAX_REPORT_AGE_S" \
    --json-out "$JSON_OUT" \
    --strict \
    "$@"
