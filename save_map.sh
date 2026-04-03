#!/usr/bin/env bash
# Thin wrapper — implementation: scripts/ws/save_map.sh
set -e
exec "$(dirname "$0")/scripts/ws/save_map.sh" "$@"
