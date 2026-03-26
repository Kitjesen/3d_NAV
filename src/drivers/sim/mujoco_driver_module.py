"""MujocoDriverModule — MuJoCo simulation as a Module (dimos-style).

Wraps sim/engine/mujoco/engine.py directly in-process.
No TCP bridge, no separate process. Data flows through In/Out ports.

Provides: odometry, lidar_cloud, camera_image, depth_image, imu
Consumes: cmd_vel, stop_signal

Usage::

    from drivers.sim.mujoco_driver_module import MujocoDriverModule
    bp.add(MujocoDriverModule,
           world="building_scene",
           render=True,
           sim_rate=50.0)
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from core.module import Module
from core.stream import In, Out
from core.msgs.geometry import Twist, Vector3, Pose, Quaternion, PoseStamped
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud, Image
from core.registry import register

logger = logging.getLogger(__name__)

# Resolve sim/ directory relative to this file
_SIM_ROOT = Path(__file__).resolve().parents[3] / "sim"
_WORLDS_DIR = _SIM_ROOT / "worlds"
_ROBOT_XML = _SIM_ROOT / "robot" / "robot.xml"

# Known worlds
WORLDS = {
    "building": "building_scene.xml",
    "building_scene": "building_scene.xml",
    "factory": "factory_scene.xml",
    "factory_scene": "factory_scene.xml",
    "open_field": "open_field.xml",
    "spiral": "spiral_terrain.xml",
    "spiral_terrain": "spiral_terrain.xml",
}


@register("driver", "sim_mujoco", description="MuJoCo sim driver (in-process, dimos-style)")
class MujocoDriverModule(Module, layer=1):
    """MuJoCo simulation running inside the Module framework.

    Steps physics at sim_rate Hz, publishes sensor data through ports.
    cmd_vel controls the robot via RL policy → joint PD control.
    """

    # -- Inputs --
    cmd_vel: In[Twist]
    stop_signal: In[int]

    # -- Outputs --
    odometry: Out[Odometry]
    lidar_cloud: Out[PointCloud]
    camera_image: Out[Image]
    depth_image: Out[Image]
    alive: Out[bool]

    def __init__(
        self,
        world: str = "building_scene",
        render: bool = False,
        sim_rate: float = 50.0,
        policy_path: str = "",
        robot_xml: str = "",
        start_pos: tuple = (0.0, 0.0, 0.35),
        **kw,
    ):
        super().__init__(**kw)
        self._world_name = world
        self._render = render
        self._sim_rate = sim_rate
        self._policy_path = policy_path or str(_SIM_ROOT / "robot" / "policy.onnx")
        self._robot_xml = robot_xml or str(_ROBOT_XML)
        self._start_pos = start_pos

        self._engine = None
        self._sim_thread: Optional[threading.Thread] = None
        self._running = False
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._stopped = False

    def setup(self):
        self.cmd_vel.subscribe(self._on_cmd_vel)
        self.stop_signal.subscribe(self._on_stop)

        try:
            # Add sim/ to path so imports work
            sim_root = str(_SIM_ROOT)
            if sim_root not in sys.path:
                sys.path.insert(0, str(_SIM_ROOT.parent))

            from sim.engine.mujoco.engine import MuJoCoEngine
            from sim.engine.core.robot import RobotConfig
            from sim.engine.core.world import WorldConfig
            from sim.engine.core.sensor import LidarConfig, CameraConfig

            # Resolve world XML
            world_file = WORLDS.get(self._world_name, self._world_name)
            world_path = _WORLDS_DIR / world_file
            if not world_path.exists():
                logger.error("World not found: %s", world_path)
                return

            robot_cfg = RobotConfig.default_nova_dog()
            robot_cfg.resolve_paths(base_dir=str(_SIM_ROOT))
            if self._policy_path:
                robot_cfg.policy_onnx = self._policy_path

            # Don't set scene_xml in WorldConfig — let load(xml_path) merge robot+scene
            world_cfg = WorldConfig()

            camera_cfg = CameraConfig(
                name="front_camera", width=640, height=480,
                fovy=60.0, render_depth=True)

            self._engine = MuJoCoEngine(
                robot_config=robot_cfg,
                world_config=world_cfg,
                lidar_config=LidarConfig(),
                camera_configs=[camera_cfg],
                headless=not self._render,
            )
            # building_scene.xml etc. are standalone scenes without robot include.
            # Pass xml_path only if the scene XML contains actuators (merged robot).
            # Otherwise let engine generate a flat scene that includes robot.xml.
            import mujoco as _mj
            _scene_content = world_path.read_text(encoding="utf-8", errors="ignore")
            _has_robot = '<actuator>' in _scene_content and 'joint' in _scene_content
            if _has_robot:
                self._engine.load(str(world_path))
            else:
                # Generate flat ground with real robot, add obstacles later
                logger.info("MujocoDriverModule: scene '%s' has no robot, using dynamic merge",
                            self._world_name)
                self._engine.load()  # uses SimWorld → <include robot.xml>
            self._engine.reset()  # stabilize + warm up policy history
            logger.info("MujocoDriverModule: loaded world '%s', robot at %s",
                        self._world_name, self._start_pos)

        except ImportError as e:
            logger.error("MujocoDriverModule: MuJoCo not available: %s", e)
        except Exception as e:
            logger.error("MujocoDriverModule: setup failed: %s", e)

    def start(self):
        super().start()
        if self._engine is None:
            self.alive.publish(False)
            return

        self._running = True
        self._sim_thread = threading.Thread(
            target=self._sim_loop, name="mujoco_sim", daemon=True)
        self._sim_thread.start()
        self.alive.publish(True)
        logger.info("MujocoDriverModule: sim loop started at %.0f Hz", self._sim_rate)

    def stop(self):
        self._running = False
        if self._sim_thread:
            self._sim_thread.join(timeout=3.0)
            self._sim_thread = None
        if self._engine:
            self._engine.close()
            self._engine = None
        self.alive.publish(False)
        super().stop()

    def _on_cmd_vel(self, twist: Twist):
        if self._stopped:
            return
        self._cmd_vx = twist.linear.x if hasattr(twist.linear, 'x') else 0.0
        self._cmd_vy = twist.linear.y if hasattr(twist.linear, 'y') else 0.0
        self._cmd_wz = twist.angular.z if hasattr(twist.angular, 'z') else 0.0

    def _on_stop(self, level: int):
        if level >= 1:
            self._cmd_vx = 0.0
            self._cmd_vy = 0.0
            self._cmd_wz = 0.0
            self._stopped = True

    def _sim_loop(self):
        """Main simulation loop — steps physics and publishes sensor data."""
        from sim.engine.core.engine import VelocityCommand

        dt = 1.0 / self._sim_rate
        step_count = 0

        while self._running and self._engine:
            t0 = time.monotonic()

            try:
                # Step physics with current command
                cmd = VelocityCommand(
                    linear_x=self._cmd_vx, linear_y=self._cmd_vy, angular_z=self._cmd_wz)
                state = self._engine.step(cmd)

                # Publish odometry
                ts = time.time()
                quat = state.orientation  # [x, y, z, w]
                self.odometry.publish(Odometry(
                    pose=Pose(
                        position=Vector3(
                            float(state.position[0]),
                            float(state.position[1]),
                            float(state.position[2])),
                        orientation=Quaternion(
                            float(quat[0]), float(quat[1]),
                            float(quat[2]), float(quat[3]))),
                    twist=Twist(
                        linear=Vector3(
                            float(state.linear_velocity[0]),
                            float(state.linear_velocity[1]),
                            float(state.linear_velocity[2])),
                        angular=Vector3(
                            float(state.angular_velocity[0]),
                            float(state.angular_velocity[1]),
                            float(state.angular_velocity[2]))),
                    ts=ts,
                ))

                # Publish LiDAR (every 5th step = ~10 Hz)
                if step_count % 5 == 0:
                    try:
                        pts = self._engine.get_lidar_points()
                        if pts is not None and len(pts) > 0:
                            self.lidar_cloud.publish(PointCloud(
                                points=pts.astype(np.float32),
                                frame_id="lidar",
                                ts=ts,
                            ))
                    except Exception:
                        pass

                # Publish camera (every 10th step = ~5 Hz)
                if step_count % 10 == 0:
                    try:
                        cam = self._engine.get_camera_data("front_camera")
                        if cam is not None:
                            self.camera_image.publish(Image(
                                data=cam.rgb, ts=ts))
                            if cam.depth is not None:
                                self.depth_image.publish(Image(
                                    data=cam.depth, ts=ts))
                    except Exception:
                        pass

                step_count += 1

            except Exception as e:
                logger.error("MujocoDriverModule: sim step error: %s", e)
                break

            # Rate control
            elapsed = time.monotonic() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info("MujocoDriverModule: sim loop ended after %d steps", step_count)

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["mujoco"] = {
            "world": self._world_name,
            "running": self._running,
            "has_engine": self._engine is not None,
            "sim_rate": self._sim_rate,
            "render": self._render,
        }
        return info
