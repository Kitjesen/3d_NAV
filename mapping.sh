#!/usr/bin/env bash
# Thin wrapper — implementation: scripts/build/mapping.sh
set -e
exec "$(dirname "$0")/scripts/build/mapping.sh" "$@"
