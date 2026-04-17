#!/usr/bin/env bash
# p0_all.sh — run all four P0 scripts in sequence. Each one logs to its own
# file; this script collates a summary.

set -e
cd "$(dirname "$0")"

STAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="${HOME}/data/nav_logs"
mkdir -p "$LOG_DIR"
SUMMARY="$LOG_DIR/${STAMP}_p0_all_summary.log"
exec > >(tee -a "$SUMMARY") 2>&1

echo "=== P0 ALL — $(date) ==="

ran=0
passed=0

run_one() {
  local name="$1"
  local script="$2"
  shift 2
  ran=$((ran + 1))
  echo ""
  echo "──────────── $name ────────────"
  if bash "$script" "$@"; then
    echo "[SUMMARY] $name  PASS"
    passed=$((passed + 1))
  else
    echo "[SUMMARY] $name  FAIL (exit $?)"
  fi
}

run_one "P0-01 cold boot"   p0_cold_boot.sh
run_one "P0-02 mapping"     p0_mapping.sh
run_one "P0-03 goto"        p0_goto.sh 2.0 0.0 60
run_one "P0-04 estop"       p0_estop.sh

echo ""
echo "=== P0 ALL COMPLETE: ${passed}/${ran} passed ==="
echo "Summary log: $SUMMARY"
[[ "$passed" -eq "$ran" ]] || exit 1
