#!/usr/bin/env bash
# Thin wrapper — implementation: scripts/build/planning.sh
set -e
exec "$(dirname "$0")/scripts/build/planning.sh" "$@"
