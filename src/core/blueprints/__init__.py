"""lingtu.core.blueprints -- pre-built system blueprints.

Four ways to build a navigation stack:

- make_blueprint(robot="nova_dog")  -- pluggable: pick driver by name
- navigation_blueprint()            -- real robot (hardcoded HanDogModule)
- simulation_blueprint()            -- MuJoCo sim (SimDogModule)
- stub_blueprint()                  -- CI testing (StubDogModule)

Prefer make_blueprint() for new code -- it uses the driver registry
so adding a new robot type requires zero changes here.
"""

from .navigation import navigation_blueprint
from .simulation import simulation_blueprint
from .stub import stub_blueprint
from .factory import make_blueprint

__all__ = [
    "make_blueprint",
    "navigation_blueprint",
    "simulation_blueprint",
    "stub_blueprint",
]
