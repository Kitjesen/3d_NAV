#!/usr/bin/env bash
# Thin wrapper — implementation: scripts/build/save_map.sh
set -e
exec "$(dirname "$0")/scripts/build/save_map.sh" "$@"
