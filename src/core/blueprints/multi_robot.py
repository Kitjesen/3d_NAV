"""Multi-robot Blueprint composition.

Each robot gets an independent full navigation stack under a unique
``robot_N/`` namespace, merged into a single Blueprint so they can be
built and started together as one system.

Typical usage::

    from core.blueprint import autoconnect
    from core.blueprints.multi_robot import multi_robot_blueprint

    system = autoconnect(
        multi_robot_blueprint(
            ["nova_dog", "omni_cart"],
            enable_semantic=True,
            llm="mock",
            enable_gateway=False,
        ),
    ).build()
    system.start()

    nav_0 = system.get_module("robot_0/NavigationModule")
    nav_1 = system.get_module("robot_1/NavigationModule")
"""

from __future__ import annotations

from core.blueprint import Blueprint

from .full_stack import full_stack_blueprint


def multi_robot_blueprint(
    robots: list[str],
    **shared_config,
) -> Blueprint:
    """Build a multi-robot system from per-robot namespaced stacks.

    Each robot in *robots* receives a full LingTu navigation stack under
    the namespace ``robot_<i>`` (where *i* is its index in the list).
    Namespaces prevent module-name collisions when the same module class
    (e.g. ``NavigationModule``) appears in every robot's stack.

    Args:
        robots:
            List of robot profile names (e.g. ``["nova_dog", "omni_cart"]``).
            Each must be a registered robot preset understood by
            :func:`full_stack_blueprint`.
        **shared_config:
            Configuration forwarded to :func:`full_stack_blueprint` for every
            robot.  Use this to set shared options such as ``llm="mock"``,
            ``enable_gateway=False``, or ``gateway_port=...``.

    Returns:
        A single :class:`~core.blueprint.Blueprint` with every robot's
        modules under distinct namespaces.

    Cross-robot wiring example::

        bp = multi_robot_blueprint(["nova_dog", "omni_cart"])
        bp.wire(
            "robot_0/NavigationModule", "mission_status",
            "robot_1/SafetyRingModule", "external_mission_status",
        )
        system = bp.auto_wire().build()
    """
    combined = Blueprint()

    for i, robot_name in enumerate(robots):
        robot_bp = full_stack_blueprint(
            robot=robot_name,
            namespace=f"robot_{i}",
            **shared_config,
        )
        combined.merge(robot_bp)

    return combined
