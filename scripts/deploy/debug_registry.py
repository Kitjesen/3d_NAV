#!/usr/bin/env python3
import os
import sys

repo = os.environ.get("LINGTU_ROOT", os.path.join(os.path.dirname(__file__), "..", ".."))
repo = os.path.abspath(repo)
sys.path[:0] = [os.path.join(repo, "src"), repo]

for mod in (
    "drivers.thunder.han_dog_module",
    "core.blueprints.stub",
    "drivers.sim.mujoco_driver_module",
    "drivers.sim.ros2_sim_driver",
):
    try:
        __import__(mod)
        print(mod, "OK")
    except Exception as e:
        print(mod, "FAIL", type(e).__name__, e)

from core.registry import list_plugins, _registry  # noqa: E402

print("drivers:", list_plugins("driver"))
print("categories:", list(_registry.keys()))
