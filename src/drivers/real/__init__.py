"""Real hardware driver backends.

Symmetric to ``drivers/sim`` (simulation backends). Holds the Python drivers
that talk to physical hardware:

- ``real/thunder/`` — Thunder quadruped (gRPC -> brainstem) + camera bridge
- ``real/lidar/``   — Livox MID-360 Python driver

ROS2 colcon packages (``drivers/livox_ros_driver2``, ``drivers/gnss``) stay at
the ``drivers/`` top level because colcon must discover them on the workspace
path; they are build artifacts, not interchangeable Python backends. See
``src/drivers/README.md`` and ``src/drivers/spec.py``.
"""
