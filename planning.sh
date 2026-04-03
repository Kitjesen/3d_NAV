#!/usr/bin/env bash
# Thin wrapper — implementation: scripts/ws/planning.sh
set -e
exec "$(dirname "$0")/scripts/ws/planning.sh" "$@"
