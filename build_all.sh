#!/usr/bin/env bash
# Thin wrapper — implementation: scripts/build/build_all.sh
set -e
exec "$(dirname "$0")/scripts/build/build_all.sh" "$@"
