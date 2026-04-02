"""MuJoCo person model — mocap body with capsule geometry.

The person is a mocap body: position-controlled each step via
data.mocap_pos, not affected by physics. Visible to camera and LiDAR.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from sim.following.interfaces import PersonState, PersonTrajectory


PERSON_XML = """\
<body name="person" pos="{x} {y} {z}" mocap="true">
  <!-- Torso: red capsule -->
  <geom name="person_torso" type="capsule" size="0.2 0.4"
        pos="0 0 1.0" rgba="0.8 0.2 0.2 1.0"
        contype="0" conaffinity="0" group="1"/>
  <!-- Head: skin-colored sphere -->
  <geom name="person_head" type="sphere" size="0.12"
        pos="0 0 1.55" rgba="0.9 0.7 0.6 1.0"
        contype="0" conaffinity="0" group="1"/>
  <!-- Left leg -->
  <geom name="person_leg_l" type="capsule" size="0.08 0.4"
        pos="0 0.1 0.4" rgba="0.2 0.2 0.6 1.0"
        contype="0" conaffinity="0" group="1"/>
  <!-- Right leg -->
  <geom name="person_leg_r" type="capsule" size="0.08 0.4"
        pos="0 -0.1 0.4" rgba="0.2 0.2 0.6 1.0"
        contype="0" conaffinity="0" group="1"/>
</body>
"""


def get_person_xml(start_pos: list[float] | None = None) -> str:
    """Return XML fragment for the person mocap body."""
    pos = start_pos or [5.0, 3.0, 0.0]
    return PERSON_XML.format(x=pos[0], y=pos[1], z=pos[2])


class PersonController:
    """Drives the person mocap body position each simulation step.

    Usage::

        ctrl = PersonController(model, trajectory)
        ctrl.reset(data)
        for step in sim_loop:
            person_state = ctrl.step(data, dt)
    """

    def __init__(self, model, trajectory: "PersonTrajectory"):
        import mujoco
        person_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "person"
        )
        if person_body_id < 0:
            raise ValueError("No 'person' body found in MuJoCo model")
        self._mocap_id = model.body_mocapid[person_body_id]
        if self._mocap_id < 0:
            raise ValueError("'person' body is not a mocap body")
        self._trajectory = trajectory

    def reset(self, data) -> "PersonState":
        state = self._trajectory.reset()
        self._apply(data, state)
        return state

    def step(self, data, dt: float) -> "PersonState":
        state = self._trajectory.step(dt)
        self._apply(data, state)
        return state

    def _apply(self, data, state: "PersonState") -> None:
        data.mocap_pos[self._mocap_id] = state.position
        # Yaw-only quaternion: [cos(h/2), 0, 0, sin(h/2)]
        cy = math.cos(state.heading / 2)
        sy = math.sin(state.heading / 2)
        data.mocap_quat[self._mocap_id] = [cy, 0.0, 0.0, sy]
