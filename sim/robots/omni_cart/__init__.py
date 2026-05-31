"""Omni-Cart wheeled robot model.

This package auto-registers the OMNI_CART_MODEL on import so that
``auto_discover()`` can find it without explicit configuration.
"""

from sim.engine.core.robot_model import OMNI_CART_MODEL, register_robot

register_robot(OMNI_CART_MODEL)
