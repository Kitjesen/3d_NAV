"""lingtu.core.blueprints — pre-built system blueprints.

Primary entry point:
    full_stack_blueprint(robot="thunder", slam_profile="fastlio2", ...)

Legacy:
    stub_blueprint() — CI testing (StubDogModule, no hardware)
"""

from .full_stack import full_stack_blueprint
from .stub import stub_blueprint

__all__ = [
    "full_stack_blueprint",
    "stub_blueprint",
]
