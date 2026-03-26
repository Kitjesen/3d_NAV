"""Go1SimDriverModule — Go1 MuJoCo walking via mujoco_playground RL policy.

Wraps the Go1 playground scene + ONNX policy directly in-process.
No TCP bridge, no separate process. Data flows through In/Out ports.

Provides: odometry, lidar_cloud, alive
Consumes: cmd_vel, stop_signal

Usage::

    from drivers.sim.go1_sim_driver import Go1SimDriverModule
    bp.add(Go1SimDriverModule,
           obstacles=[{"name": "wall", "type": "box", ...}],
           start_pos=(0, 0, 0.30),
           sim_rate=50.0)
"""

from __future__ import annotations

import logging
import math
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from core.module import Module
from core.stream import In, Out
from core.msgs.geometry import Twist, Vector3, Pose, Quaternion
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud
from core.registry import register

logger = logging.getLogger(__name__)

# Resolve sim/ directory relative to this file (src/drivers/sim/ -> repo root / sim/)
_SIM_ROOT = Path(__file__).resolve().parents[3] / "sim"
_GO1_DIR = _SIM_ROOT / "robots" / "go1_playground"
_SCENE_XML = _GO1_DIR / "xmls" / "scene_mjx_feetonly_flat_terrain.xml"
_POLICY_ONNX = _GO1_DIR / "go1_policy.onnx"

# Policy constants (from mujoco_playground Go1 defaults)
_CTRL_DT = 0.02       # policy decision frequency (20ms = 50Hz)
_ACTION_SCALE = 0.5   # joint action scale
_N_RAYS = 180         # LiDAR ray count
_LIDAR_MIN = 0.2      # metres
_LIDAR_MAX = 10.0     # metres


@register("driver", "go1_sim", description="Go1 MuJoCo RL walking sim driver")
class Go1SimDriverModule(Module, layer=1):
    """Go1 quadruped simulation using mujoco_playground RL policy.

    Steps MuJoCo physics at sim_rate Hz driven by set_mjcb_control callback.
    cmd_vel maps directly to policy nav_cmd (no axis rotation needed).
    LiDAR via mj_multiRay with trunk body exclusion.
    """

    # -- Inputs --
    cmd_vel: In[Twist]
    stop_signal: In[int]

    # -- Outputs --
    odometry: Out[Odometry]
    lidar_cloud: Out[PointCloud]
    alive: Out[bool]

    def __init__(
        self,
        obstacles: list = None,
        start_pos: tuple = (0, 0, 0.30),
        sim_rate: float = 50.0,
        goal_marker: tuple = None,
        **kw,
    ):
        super().__init__(**kw)
        self._obstacles = obstacles or []
        self._start_pos = tuple(start_pos)
        self._sim_rate = sim_rate
        self._goal_marker = goal_marker

        # MuJoCo model/data (created in setup)
        self._model = None
        self._data = None

        # Policy state
        self._policy_sess = None
        self._policy_out_name: str = ""
        self._default_angles: Optional[np.ndarray] = None
        self._last_action: np.ndarray = np.zeros(12, dtype=np.float32)
        self._nav_cmd: np.ndarray = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self._n_substeps: int = 1
        self._step_counter: int = 0

        # Thread control
        self._sim_thread: Optional[threading.Thread] = None
        self._running = False
        self._stopped = False
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def setup(self):
        self.cmd_vel.subscribe(self._on_cmd_vel)
        self.stop_signal.subscribe(self._on_stop)

        try:
            import mujoco
            import onnxruntime as ort

            if not _SCENE_XML.exists():
                logger.error("Go1SimDriverModule: scene XML not found: %s", _SCENE_XML)
                return
            if not _POLICY_ONNX.exists():
                logger.error("Go1SimDriverModule: policy ONNX not found: %s", _POLICY_ONNX)
                return

            # Inject obstacles (and optional goal marker) into scene XML
            scene_text = _SCENE_XML.read_text(encoding="utf-8")
            extra_geoms = self._build_obstacle_xml()
            if extra_geoms:
                scene_text = scene_text.replace("</worldbody>", extra_geoms + "  </worldbody>")

            # Write patched XML to a temp file in the same directory so MuJoCo
            # can resolve relative mesh/texture paths correctly.
            tmp = tempfile.NamedTemporaryFile(
                suffix=".xml",
                delete=False,
                dir=str(_SCENE_XML.parent),
                mode="w",
                encoding="utf-8",
            )
            tmp.write(scene_text)
            tmp.close()
            tmp_path = Path(tmp.name)

            try:
                self._model = mujoco.MjModel.from_xml_path(str(tmp_path))
                self._data = mujoco.MjData(self._model)
                logger.info(
                    "Go1SimDriverModule: loaded scene (nq=%d nu=%d ngeom=%d)",
                    self._model.nq, self._model.nu, self._model.ngeom,
                )
            finally:
                tmp_path.unlink(missing_ok=True)

            # Load ONNX policy
            self._policy_sess = ort.InferenceSession(
                str(_POLICY_ONNX),
                providers=ort.get_available_providers(),
            )
            self._policy_out_name = self._policy_sess.get_outputs()[0].name

            # Default joint angles from keyframe "home" (index 0)
            mujoco.mj_resetDataKeyframe(self._model, self._data, 0)
            self._default_angles = self._data.qpos[7:].copy()
            self._last_action = np.zeros(12, dtype=np.float32)

            # Compute substep ratio: policy runs at ctrl_dt, physics at model.opt.timestep
            self._n_substeps = max(1, round(_CTRL_DT / self._model.opt.timestep))

            # Register MuJoCo control callback
            mujoco.set_mjcb_control(self._ctrl_callback)

            # Stabilize: 1000 steps with cmd=[0,0,0]
            logger.info("Go1SimDriverModule: stabilizing (1000 steps)...")
            self._nav_cmd[:] = [0.0, 0.0, 0.0]
            for _ in range(1000):
                mujoco.mj_step(self._model, self._data)
            pos = self._data.qpos[:3]
            logger.info(
                "Go1SimDriverModule: standing at (%.2f, %.2f, %.2f)", *pos
            )

        except ImportError as e:
            logger.error("Go1SimDriverModule: missing dependency: %s", e)
        except Exception as e:
            logger.error("Go1SimDriverModule: setup failed: %s", e)

    def start(self):
        super().start()
        if self._model is None:
            self.alive.publish(False)
            return

        self._running = True
        self._sim_thread = threading.Thread(
            target=self._sim_loop, name="go1_sim", daemon=True
        )
        self._sim_thread.start()
        self.alive.publish(True)
        logger.info("Go1SimDriverModule: sim loop started at %.0f Hz", self._sim_rate)

    def stop(self):
        self._running = False
        if self._sim_thread:
            self._sim_thread.join(timeout=3.0)
            self._sim_thread = None
        try:
            import mujoco
            mujoco.set_mjcb_control(None)
        except Exception:
            pass
        self._model = None
        self._data = None
        self.alive.publish(False)
        super().stop()

    # ------------------------------------------------------------------
    # Port callbacks
    # ------------------------------------------------------------------

    def _on_cmd_vel(self, twist: Twist):
        if self._stopped:
            return
        # Go1 playground model: +X is forward, no axis rotation needed.
        vx = float(twist.linear.x) if hasattr(twist.linear, "x") else 0.0
        vy = float(twist.linear.y) if hasattr(twist.linear, "y") else 0.0
        wz = float(twist.angular.z) if hasattr(twist.angular, "z") else 0.0
        with self._lock:
            self._nav_cmd[:] = [vx, vy, wz]

    def _on_stop(self, level: int):
        if level >= 1:
            with self._lock:
                self._nav_cmd[:] = [0.0, 0.0, 0.0]
            self._stopped = True

    # ------------------------------------------------------------------
    # MuJoCo control callback (called every physics substep)
    # ------------------------------------------------------------------

    def _ctrl_callback(self, m, d):
        """Policy callback registered with mujoco.set_mjcb_control.

        Only runs the neural network every n_substeps steps so that the
        policy decision rate stays at ctrl_dt (~50 Hz) regardless of the
        physics timestep.
        """
        self._step_counter += 1
        if self._step_counter % self._n_substeps != 0:
            return

        try:
            # Sensor readings
            linvel = d.sensor("local_linvel").data.copy().astype(np.float32)   # (3,)
            gyro = d.sensor("gyro").data.copy().astype(np.float32)             # (3,)

            # Gravity vector in IMU body frame
            imu_id = m.site("imu").id
            imu_xmat = d.site_xmat[imu_id].reshape(3, 3)
            gravity = (imu_xmat.T @ np.array([0.0, 0.0, -1.0])).astype(np.float32)

            # Joint state (relative to default pose)
            jpos = (d.qpos[7:] - self._default_angles).astype(np.float32)     # (12,)
            jvel = d.qvel[6:].astype(np.float32)                               # (12,)

            # Thread-safe nav command snapshot
            with self._lock:
                nav_cmd = self._nav_cmd.copy()

            # Build 48-dim observation:
            # linvel(3) + gyro(3) + gravity(3) + jpos(12) + jvel(12) + last_action(12) + nav_cmd(3)
            obs = np.concatenate([
                linvel,             # 3
                gyro,               # 3
                gravity,            # 3
                jpos,               # 12
                jvel,               # 12
                self._last_action,  # 12
                nav_cmd,            # 3
            ]).reshape(1, -1)       # total = 48

            # Run policy
            action = self._policy_sess.run(
                [self._policy_out_name], {"obs": obs}
            )[0][0]

            self._last_action = action.copy()
            d.ctrl[:] = action * _ACTION_SCALE + self._default_angles

        except Exception as e:
            logger.debug("Go1SimDriverModule: ctrl_callback error: %s", e)

    # ------------------------------------------------------------------
    # Simulation loop (independent thread)
    # ------------------------------------------------------------------

    def _sim_loop(self):
        """Step MuJoCo at sim_rate Hz and publish odometry + LiDAR."""
        import mujoco

        dt = 1.0 / self._sim_rate
        step_count = 0

        # Trunk body id for LiDAR bodyexclude
        try:
            trunk_id = self._model.body("trunk").id
        except Exception:
            trunk_id = -1

        geomgroup = np.ones(6, dtype=np.uint8)

        while self._running and self._model is not None:
            t0 = time.monotonic()
            ts = time.time()

            try:
                mujoco.mj_step(self._model, self._data)

                # -- Odometry --
                pos = self._data.qpos[:3]
                # MuJoCo quaternion convention: qpos[3:7] = (w, x, y, z)
                qw = float(self._data.qpos[3])
                qx = float(self._data.qpos[4])
                qy = float(self._data.qpos[5])
                qz = float(self._data.qpos[6])
                self.odometry.publish(Odometry(
                    pose=Pose(
                        position=Vector3(
                            float(pos[0]),
                            float(pos[1]),
                            float(pos[2]),
                        ),
                        orientation=Quaternion(qx, qy, qz, qw),
                    ),
                    twist=Twist(
                        linear=Vector3(
                            float(self._data.qvel[0]),
                            float(self._data.qvel[1]),
                            float(self._data.qvel[2]),
                        ),
                        angular=Vector3(
                            float(self._data.qvel[3]),
                            float(self._data.qvel[4]),
                            float(self._data.qvel[5]),
                        ),
                    ),
                    ts=ts,
                ))

                # -- LiDAR (every 5 steps ~ 10 Hz at 50 Hz sim rate) --
                if step_count % 5 == 0:
                    try:
                        pts = self._scan_lidar(mujoco, geomgroup, trunk_id)
                        if pts is not None and len(pts) > 0:
                            self.lidar_cloud.publish(PointCloud(
                                points=pts,
                                frame_id="lidar",
                                ts=ts,
                            ))
                    except Exception as e:
                        logger.debug("Go1SimDriverModule: lidar error: %s", e)

                step_count += 1

            except Exception as e:
                logger.error("Go1SimDriverModule: sim step error: %s", e)
                break

            elapsed = time.monotonic() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info("Go1SimDriverModule: sim loop ended after %d steps", step_count)

    # ------------------------------------------------------------------
    # LiDAR raycasting
    # ------------------------------------------------------------------

    def _scan_lidar(
        self,
        mujoco,
        geomgroup: np.ndarray,
        trunk_id: int,
    ) -> Optional[np.ndarray]:
        """180-ray horizontal LiDAR via mj_multiRay, excluding trunk body."""
        if self._model is None or self._data is None:
            return None

        pos = self._data.qpos[:3].copy().astype(np.float64)
        pos[2] = max(pos[2], 0.15)  # ensure origin is above ground

        # Robot yaw from MuJoCo quaternion qpos[3:7] = (w, x, y, z)
        q = self._data.qpos[3:7]
        yaw = math.atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2),
        )
        cy, sy = math.cos(yaw), math.sin(yaw)

        # Build ray directions (world frame, horizontal plane)
        angles = np.linspace(0.0, 2.0 * math.pi, _N_RAYS, endpoint=False)
        dirs = np.zeros((_N_RAYS, 3), dtype=np.float64)
        for i, a in enumerate(angles):
            lx, ly = math.cos(a), math.sin(a)
            dirs[i, 0] = lx * cy - ly * sy
            dirs[i, 1] = lx * sy + ly * cy
            dirs[i, 2] = 0.0

        dist_out = np.full(_N_RAYS, -1.0, dtype=np.float64)
        geomid_out = np.full(_N_RAYS, -1, dtype=np.int32)

        mujoco.mj_multiRay(
            self._model, self._data,
            pos, dirs.flatten(),
            geomgroup, 1, trunk_id,
            geomid_out, dist_out, None,
            _N_RAYS, _LIDAR_MAX,
        )

        mask = (dist_out > _LIDAR_MIN) & (dist_out < _LIDAR_MAX)
        if not mask.any():
            return np.zeros((0, 3), dtype=np.float32)

        return (pos + dirs[mask] * dist_out[mask, None]).astype(np.float32)

    # ------------------------------------------------------------------
    # Obstacle XML helpers
    # ------------------------------------------------------------------

    def _build_obstacle_xml(self) -> str:
        """Convert self._obstacles list into MuJoCo geom XML strings."""
        lines: List[str] = []

        for o in self._obstacles:
            if isinstance(o, dict):
                name = o.get("name", "obs")
                shape = o.get("type", o.get("shape", "box"))
                size = o.get("size", [0.3, 0.3, 0.3])
                pos = o.get("pos", o.get("position", [0.0, 0.0, 0.0]))
                rgba = o.get("rgba", [0.5, 0.5, 0.6, 1.0])
            else:
                # Duck-typed ObstacleConfig or similar
                name = getattr(o, "name", "obs")
                shape = getattr(o, "shape", "box")
                size = getattr(o, "size", [0.3, 0.3, 0.3])
                pos = getattr(o, "position", [0.0, 0.0, 0.0])
                rgba = getattr(o, "rgba", [0.5, 0.5, 0.6, 1.0])

            size_str = " ".join(str(s) for s in size)
            pos_str = " ".join(str(p) for p in pos)
            rgba_str = " ".join(str(c) for c in rgba)
            lines.append(
                f'    <geom name="{name}" type="{shape}" '
                f'size="{size_str}" pos="{pos_str}" rgba="{rgba_str}"/>'
            )

        if self._goal_marker:
            gx = float(self._goal_marker[0])
            gy = float(self._goal_marker[1])
            gz = float(self._goal_marker[2]) if len(self._goal_marker) > 2 else 0.15
            lines.append(
                f'    <geom name="goal_vis" type="sphere" size="0.15" '
                f'pos="{gx} {gy} {gz}" rgba="1 0.2 0.2 0.7" '
                f'contype="0" conaffinity="0"/>'
            )

        return "\n".join(lines) + "\n" if lines else ""

    # ------------------------------------------------------------------
    # Health
    # ------------------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        pos = None
        if self._data is not None:
            try:
                p = self._data.qpos[:3]
                pos = [round(float(p[0]), 2), round(float(p[1]), 2), round(float(p[2]), 2)]
            except Exception:
                pass
        info["go1_sim"] = {
            "running": self._running,
            "has_model": self._model is not None,
            "sim_rate": self._sim_rate,
            "n_substeps": self._n_substeps,
            "step_counter": self._step_counter,
            "nav_cmd": self._nav_cmd.tolist(),
            "pos": pos,
        }
        return info
