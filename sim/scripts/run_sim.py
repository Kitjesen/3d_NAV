#!/usr/bin/env python3
"""MapPilot MuJoCo Simulation — compatibility wrapper.

Delegates to the unified CLI entry point at sim/engine/cli.py.
All new features should be added there, not here.

Usage (unchanged):
    python sim/scripts/run_sim.py --world open_field
    python sim/scripts/run_sim.py --headless
    python sim/scripts/run_sim.py --world spiral_terrain
"""
import sys
import os

# Ensure project root is on path
_PROJ = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _PROJ)

from sim.engine.cli import main

# Map legacy arg names to engine/cli args:
#   --world spiral_terrain  → --world spiral  (registry name)
#   --no-ros                → (drop, cli always tries ROS2 with graceful fallback)
#   --lidar-hz / --sim-speed → (drop, cli uses defaults)
_WORLD_ALIASES = {
    "spiral_terrain": "spiral",
    "open_field": "open_field",
}


def _translate_argv():
    """Translate legacy argv to engine/cli format."""
    translated = []
    skip_next = False
    argv = sys.argv[1:]
    for i, arg in enumerate(argv):
        if skip_next:
            skip_next = False
            continue
        if arg == "--no-ros":
            continue  # cli handles ROS2 absence gracefully
        if arg in ("--lidar-hz", "--sim-speed"):
            skip_next = True  # drop the value too
            continue
        if arg == "--world" and i + 1 < len(argv):
            translated.append(arg)
            world = argv[i + 1]
            translated.append(_WORLD_ALIASES.get(world, world))
            skip_next = True
            continue
        translated.append(arg)
    return translated


if __name__ == "__main__":
    main(_translate_argv())
