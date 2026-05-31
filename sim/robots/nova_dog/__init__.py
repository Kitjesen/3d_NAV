"""Nova Dog (Thunder v3) robot model.

This package auto-registers the NOVA_DOG_MODEL on import so that
``auto_discover()`` can find it without explicit configuration.
"""

from sim.engine.core.robot_model import NOVA_DOG_MODEL, register_robot

register_robot(NOVA_DOG_MODEL)
