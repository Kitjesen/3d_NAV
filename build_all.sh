#!/usr/bin/env bash
# Thin wrapper — implementation: scripts/ws/build_all.sh
set -e
exec "$(dirname "$0")/scripts/ws/build_all.sh" "$@"
