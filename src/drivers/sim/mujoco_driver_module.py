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
from core.msgs.sensor import PointCloud2, Image, ImageFormat, CameraIntrinsics
from core.registry import register

logger = logging.getLogger(__name__)

# Resolve sim/ directory relative to this file
_SIM_ROOT = Path(__file__).resolve().parents[3] / "sim"
_WORLDS_DIR = _SIM_ROOT / "worlds"
_ROBOTS_DIR = _SIM_ROOT / "robots" / "nova_dog"
_ROBOT_XML = _ROBOTS_DIR / "robot_with_camera.xml"
_POLICY_ONNX = _ROBOTS_DIR / "policy.onnx"
_DEFAULT_START_POS = (0.0, 0.0, 0.35)


def _is_default_start_pos(start_pos: tuple) -> bool:
    return tuple(float(v) for v in start_pos[:3]) == _DEFAULT_START_POS


def _scene_placeholder_start(scene_xml: Path) -> Optional[list[float]]:
    try:
        import xml.etree.ElementTree as ET

        root = ET.fromstring(scene_xml.read_text(encoding="utf-8", errors="ignore"))
        worldbody = root.find("worldbody")
        if worldbody is None:
            return None
        for body in worldbody.findall("body"):
            if body.attrib.get("name") != "robot_placeholder":
                continue
            pos_str = body.attrib.get("pos", "")
            parts = [float(v) for v in pos_str.split()]
            if len(parts) >= 3:
                return parts[:3]
    except Exception:
        logger.debug("Failed to parse robot_placeholder pose from %s", scene_xml, exc_info=True)
    return None


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
    lidar_cloud: Out[PointCloud2]
    camera_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraIntrinsics]
    alive: Out[bool]

    def __init__(
        self,
        world: str = "building_scene",
        render: bool = False,
        enable_camera: bool = False,
        sim_rate: float = 50.0,
        policy_path: str = "",
        robot_xml: str = "",
        start_pos: tuple = (0.0, 0.0, 0.35),
        obstacles: list = None,
        **kw,
    ):
        super().__init__(**kw)
        self._world_name = world
        self._render = render
        self._enable_camera = bool(enable_camera or render)
        self._sim_rate = sim_rate
        default_policy = str(_POLICY_ONNX) if _POLICY_ONNX.exists() else ""
        self._policy_path = policy_path or default_policy
        self._robot_xml = robot_xml or str(_ROBOT_XML)
        self._start_pos = start_pos
        self._obstacles = obstacles or []

        self._engine = None
        self._sim_thread: Optional[threading.Thread] = None
        self._running = False
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._stopped = False
        self._camera_warned = False

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
            if not Path(self._robot_xml).exists():
                logger.error("Robot XML not found: %s", self._robot_xml)
                return

            robot_cfg = RobotConfig.default_nova_dog()
            robot_cfg.resolve_paths(base_dir=str(_SIM_ROOT))
            robot_cfg.robot_xml = self._robot_xml
            scene_start = _scene_placeholder_start(world_path)
            start_pos = scene_start if scene_start is not None and _is_default_start_pos(self._start_pos) else list(self._start_pos)
            robot_cfg.init_position = [float(v) for v in start_pos[:3]]
            if self._policy_path:
                robot_cfg.policy_onnx = self._policy_path

            from sim.engine.core.world import ObstacleConfig
            obs_cfgs = []
            for o in self._obstacles:
                if isinstance(o, dict):
                    obs_cfgs.append(ObstacleConfig(**o))
                else:
                    obs_cfgs.append(o)
            world_cfg = WorldConfig(scene_xml=str(world_path), obstacles=obs_cfgs)

            # Camera capture is independent from the on-screen viewer so the
            # semantic stack can run in headless simulation.
            camera_cfgs = []
            if self._enable_camera:
                camera_cfg = CameraConfig(
                    name="front_camera", width=640, height=480,
                    fovy=60.0, render_depth=True)
                camera_cfgs = [camera_cfg]

            self._engine = MuJoCoEngine(
                robot_config=robot_cfg,
                world_config=world_cfg,
                lidar_config=LidarConfig(
                    body_name=robot_cfg.lidar_body_name,
                    geom_group=0,
                ),
                camera_configs=camera_cfgs,
                headless=not self._render,
            )
            # Let MuJoCoEngine decide whether to use the scene directly or merge
            # the selected world geometry into the robot model.
            self._engine.load(str(world_path))
            self._engine.reset()  # stabilize + warm up policy history
            logger.info("MujocoDriverModule: loaded world '%s', robot at %s",
                        self._world_name, tuple(robot_cfg.init_position))

        except ImportError as e:
            self._engine = None
            logger.error("MujocoDriverModule: MuJoCo not available: %s", e)
        except Exception as e:
            self._engine = None
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
        # MuJoCo engine: linear_x = forward, linear_y = lateral (verified by Step 2 test)
        # Nav stack: vx = forward, vy = lateral — same convention, direct passthrough
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
                            self.lidar_cloud.publish(PointCloud2(
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
                            h, w = (cam.rgb.shape[:2] if cam.rgb is not None
                                    else cam.depth.shape[:2])
                            self.camera_image.publish(Image(
                                data=cam.rgb,
                                format=ImageFormat.RGB,
                                ts=ts,
                                frame_id="camera_link",
                            ))
                            if cam.depth is not None:
                                self.depth_image.publish(Image(
                                    data=cam.depth,
                                    format=ImageFormat.DEPTH_F32,
                                    ts=ts,
                                    frame_id="camera_link",
                                ))
                            fx, fy, cx, cy = cam.intrinsics
                            self.camera_info.publish(CameraIntrinsics(
                                fx=float(fx),
                                fy=float(fy),
                                cx=float(cx),
                                cy=float(cy),
                                width=int(w),
                                height=int(h),
                                depth_scale=1.0,
                            ))
                    except Exception as e:
                        if not self._camera_warned:
                            logger.warning("MujocoDriverModule: camera publish failed: %s", e)
                            self._camera_warned = True

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
