#!/usr/bin/env bash
# L2.5: strict server simulation closure with report freshness enforcement.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

MAX_REPORT_AGE_S="${LINGTU_L25_MAX_REPORT_AGE_S:-21600}"
JSON_OUT="${LINGTU_L25_JSON_OUT:-artifacts/server_sim_closure_summary_all.json}"

PYTHONPATH="${ROOT}/src:${ROOT}:${PYTHONPATH:-}" \
  python sim/scripts/server_sim_closure.py \
    --max-report-age-s "$MAX_REPORT_AGE_S" \
    --json-out "$JSON_OUT" \
    --strict \
    "$@"
