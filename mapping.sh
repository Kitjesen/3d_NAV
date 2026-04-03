#!/usr/bin/env bash
# Thin wrapper — implementation: scripts/ws/mapping.sh
set -e
exec "$(dirname "$0")/scripts/ws/mapping.sh" "$@"
