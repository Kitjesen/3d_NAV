"""Multi-robot Blueprint composition.

Each robot gets an independent full navigation stack under a unique
``robot_N/`` namespace, merged into a single Blueprint so they can be
built and started together as one system.

Typical usage::

    from core.blueprint import autoconnect
    from core.blueprints.multi_robot import multi_robot_blueprint

    system = autoconnect(
        multi_robot_blueprint(
            ["stub", "stub"],
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
    port_offset: int = 0,
    **shared_config,
) -> Blueprint:
    """Build a multi-robot system from per-robot namespaced stacks.

    Each robot in *robots* receives a full LingTu navigation stack under
    the namespace ``robot_<i>`` (where *i* is its index in the list).
    Namespaces prevent module-name collisions when the same module class
    (e.g. ``NavigationModule``) appears in every robot's stack.

    Args:
        robots:
            List of robot preset names (e.g. ``["stub", "nova_dog"]``).
            Each must be a registered driver backend understood by
            :func:`full_stack_blueprint`.  Valid values include ``"stub"``
            (testing, no hardware), ``"thunder"`` (real S100P),
            ``"nova_dog"`` (gRPC brainstem), ``"sim_mujoco"`` (MuJoCo sim),
            and ``"sim_ros2"`` (ROS2 bridge).
        port_offset:
            Offset added to the base gateway port (5050).  Each robot *i*
            receives gateway port ``5050 + i + port_offset``, ensuring no
            two robots bind the same port.  Default ``0``.
        **shared_config:
            Configuration forwarded to :func:`full_stack_blueprint` for every
            robot.  Use this to set shared options such as ``llm="mock"``,
            ``enable_gateway=False``, or ``gateway_port=...``.
            Do **not** pass ``robot`` or ``namespace`` here — these are
            set per-robot automatically.

    Returns:
        A single :class:`~core.blueprint.Blueprint` with every robot's
        modules under distinct namespaces.

    Cross-robot wiring example::

        bp = multi_robot_blueprint(["stub", "stub"])
        bp.wire("robot_0/NavigationModule", "mission_status",
                "robot_1/SafetyRingModule", "external_mission_status")
    """
    # Pop namespace if accidentally included in shared_config, so it
    # does not collide with the per-robot namespace passed explicitly.
    shared_config.pop("namespace", None)

    combined = Blueprint()

    for i, robot_name in enumerate(robots):
        kwargs = dict(shared_config)
        # Fix 1: unique gateway port per robot to avoid port collisions
        kwargs["gateway_port"] = 5050 + i + port_offset
        # Fix 2: only robot_0 manages systemd services (SLAM, detectors)
        if i > 0:
            kwargs["manage_external_services"] = False

        robot_bp = full_stack_blueprint(
            robot=robot_name,
            namespace=f"robot_{i}",
            **kwargs,
        )
        combined.merge(robot_bp)

    return combined
