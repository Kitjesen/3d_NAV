"""MuJoCo simulation engine implementation
# Extracted from sim/bridge/nova_nav_bridge.py

Core logic sourced from nova_nav_bridge.py:
  - MuJoCo model loading and scene XML generation
  - Physics stepping loop (mujoco.mj_step)
  - cmd_vel -> ONNX policy -> joint control
  - IMU / joint state reading (get_imu, get_joint_state)
  - LiDAR scan wrapper
"""
import os
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from sim.engine.core.engine import CameraData, RobotState, SimEngine, VelocityCommand
from sim.engine.core.robot import RobotConfig
from sim.engine.core.sensor import CameraConfig, IMUConfig, LidarConfig
from sim.engine.core.world import WorldConfig, SimWorld
from sim.engine.mujoco.camera import MuJoCoCamera
from sim.engine.mujoco.lidar import MuJoCoLidar
from sim.engine.mujoco.robot_controller import (
    DART_TO_MJ,
    MJ_TO_DART,
    STANDING_POSE,
    PolicyRunner,
)


# Actuator offset (first 8 actuators are arm; legs start at index 8)
# Extracted from sim/bridge/nova_nav_bridge.py
LEG_JOINT_NAMES = [
    'fr_hip_joint', 'fr_thigh_joint', 'fr_calf_joint', 'fr_foot_joint',
    'fl_hip_joint', 'fl_thigh_joint', 'fl_calf_joint', 'fl_foot_joint',
    'rr_hip_joint', 'rr_thigh_joint', 'rr_calf_joint', 'rr_foot_joint',
    'rl_hip_joint', 'rl_thigh_joint', 'rl_calf_joint', 'rl_foot_joint',
]


class MuJoCoEngine(SimEngine):
    """MuJoCo simulation engine.

    Implements the SimEngine abstract interface, wrapping:
      - Scene loading (WorldConfig + RobotConfig)
      - Physics stepping (500Hz physics, 50Hz policy)
      - ONNX gait policy (PolicyRunner)
      - LiDAR scanning (MuJoCoLidar)
      - Camera rendering (MuJoCoCamera)
    """

    def __init__(self,
                 robot_config: Optional[RobotConfig] = None,
                 world_config: Optional[WorldConfig] = None,
                 lidar_config: Optional[LidarConfig] = None,
                 camera_configs: Optional[List[CameraConfig]] = None,
                 imu_config: Optional[IMUConfig] = None,
                 headless: bool = True) -> None:
        """Initialize MuJoCo engine (model not loaded; call load() before use).

        Args:
            robot_config: robot configuration, defaults to NOVA Dog
            world_config: scene configuration, defaults to flat ground
            lidar_config: LiDAR configuration, defaults to MID-360
            camera_configs: list of camera configurations
            imu_config: IMU configuration
            headless: True=headless mode, False=launch MuJoCo viewer
        """
        super().__init__()

        self._robot_cfg = robot_config or RobotConfig.default_nova_dog()
        self._world_cfg = world_config or WorldConfig()
        self._lidar_cfg = lidar_config or LidarConfig()
        self._camera_cfgs = camera_configs or []
        self._imu_cfg = imu_config or IMUConfig()
        self._headless = headless

        # MuJoCo core objects (initialized after load())
        self._model = None
        self._data = None

        # Sensors
        self._lidar: Optional[MuJoCoLidar] = None
        self._cameras: Dict[str, MuJoCoCamera] = {}

        # Policy controller
        self._policy: Optional[PolicyRunner] = None

        # Joint ID lists (populated after load())
        self._leg_joint_ids: List[int] = []
        self._base_body_id: int = 0
        self._lidar_body_id: int = 0
        self._leg_actuator_offset: int = 0

        # cmd_vel (thread-safe)
        self._cmd_vel = np.zeros(3, dtype=np.float64)  # [vx, vy, wz]
        self._cmd_vel_time = 0.0
        self._lock = threading.Lock()

        # Physics parameters
        self._physics_dt: float = 0.002   # read from model.opt.timestep
        self._control_dt: float = 0.02    # 50Hz policy

        # Background physics thread
        self._stop_event = threading.Event()
        self._sim_thread: Optional[threading.Thread] = None

    # ──────────────────────────────────────────────────────────────
    # Lifecycle
    # ──────────────────────────────────────────────────────────────

    def load(self, xml_path: str = "", **kwargs: Any) -> None:
        """Load scene and initialize all subsystems.

        Args:
            xml_path: scene XML path. If empty, generates from WorldConfig.
            **kwargs: extra parameters (ignored, for interface compatibility)
        """
        import mujoco

        # Resolve robot XML path
        self._robot_cfg.resolve_paths()
        robot_xml = self._robot_cfg.robot_xml

        if xml_path:
            # Check whether XML contains a ground plane (geom in worldbody)
            # If it is a bare robot.xml (no ground), auto-wrap with a scene
            _xml_content = Path(xml_path).read_text(encoding="utf-8", errors="ignore")
            # Only consider it grounded if it explicitly contains a floor/plane
            _has_floor = ('floor' in _xml_content.lower()
                          or 'name="ground"' in _xml_content
                          or 'type="plane"' in _xml_content)
            _has_actuators = '<actuator>' in _xml_content
            if _has_floor and _has_actuators:
                # Scene already has robot (actuators present) — use as-is
                self._model = mujoco.MjModel.from_xml_path(xml_path)
            elif _has_floor and not _has_actuators:
                # Scene has floor but no robot — merge robot into scene using
                # structured XML editing so nested placeholder bodies are
                # removed safely.
                import copy
                import xml.etree.ElementTree as ET

                scene_root = ET.fromstring(_xml_content)
                robot_root = ET.fromstring(
                    Path(robot_xml).read_text(encoding="utf-8", errors="ignore")
                )

                scene_worldbody = scene_root.find("worldbody")
                robot_worldbody = robot_root.find("worldbody")
                if scene_worldbody is None or robot_worldbody is None:
                    self._model = mujoco.MjModel.from_xml_path(xml_path)
                else:
                    scene_asset = scene_root.find("asset")
                    robot_asset = robot_root.find("asset")
                    if scene_asset is not None and len(scene_asset) > 0:
                        if robot_asset is None:
                            robot_asset = ET.Element("asset")
                            robot_root.append(robot_asset)
                        existing_asset_keys = {
                            (child.tag, child.attrib.get("name"))
                            for child in list(robot_asset)
                        }
                        for child in list(scene_asset):
                            asset_key = (child.tag, child.attrib.get("name"))
                            if asset_key in existing_asset_keys:
                                continue
                            robot_asset.append(copy.deepcopy(child))
                            existing_asset_keys.add(asset_key)

                    for child in list(scene_worldbody):
                        if (
                            child.tag == "body"
                            and child.attrib.get("name") in {"robot_placeholder", "base_link"}
                        ):
                            continue
                        robot_worldbody.append(copy.deepcopy(child))

                    _merged = ET.tostring(robot_root, encoding="unicode")
                    _robot_dir = str(Path(robot_xml).parent)
                    _tmp = tempfile.NamedTemporaryFile(
                        suffix=".xml", delete=False, dir=_robot_dir,
                        mode="w", encoding="utf-8")
                    _tmp.write(_merged)
                    _tmp.close()
                    try:
                        self._model = mujoco.MjModel.from_xml_path(_tmp.name)
                    finally:
                        Path(_tmp.name).unlink(missing_ok=True)
            else:
                # Auto-inject ground: insert floor geom after <worldbody>
                _robot_dir = str(Path(xml_path).parent)
                _floor_geom = (
                    '    <geom name="floor" type="plane" size="50 50 0.1"'
                    ' conaffinity="1" condim="3" friction="1 0.5 0.5"/>\n'
                )
                _patched = _xml_content.replace(
                    "<worldbody>",
                    "<worldbody>\n" + _floor_geom,
                    1  # only replace the first occurrence
                )
                _tmp = tempfile.NamedTemporaryFile(
                    suffix=".xml", delete=False, dir=_robot_dir,
                    mode="w", encoding="utf-8"
                )
                _tmp.write(_patched)
                _tmp.close()
                try:
                    self._model = mujoco.MjModel.from_xml_path(_tmp.name)
                finally:
                    Path(_tmp.name).unlink(missing_ok=True)
        elif self._world_cfg.scene_xml and Path(self._world_cfg.scene_xml).exists():
            self._model = mujoco.MjModel.from_xml_path(self._world_cfg.scene_xml)
        else:
            # Generate scene XML dynamically from WorldConfig
            world = SimWorld(self._world_cfg)
            # MuJoCo is sensitive to Windows-style include paths. Use a normalized
            # absolute POSIX path inside the generated XML so headless startup
            # resolves robot.xml consistently.
            scene_xml_str = world.get_scene_xml(Path(robot_xml).resolve().as_posix())
            # Write to temp file (MuJoCo needs file path to handle <include>)
            robot_dir = str(Path(robot_xml).parent)
            tmp = tempfile.NamedTemporaryFile(
                suffix=".xml", delete=False,
                dir=robot_dir, mode="w", encoding="utf-8"
            )
            tmp.write(scene_xml_str)
            tmp.close()
            try:
                self._model = mujoco.MjModel.from_xml_path(tmp.name)
            finally:
                Path(tmp.name).unlink(missing_ok=True)

        self._data = mujoco.MjData(self._model)
        self._physics_dt = float(self._model.opt.timestep)

        # Resolve body/joint IDs
        self._resolve_ids()
        valid_leg_joints = len([jid for jid in self._leg_joint_ids if jid >= 0])
        if valid_leg_joints > 0 and self._model is not None:
            self._leg_actuator_offset = max(0, int(self._model.nu) - valid_leg_joints)
        else:
            self._leg_actuator_offset = self._robot_cfg.leg_act_offset

        # Initialize LiDAR
        self._lidar = MuJoCoLidar(self._model, self._data, self._lidar_cfg)

        # Initialize cameras
        for cam_cfg in self._camera_cfgs:
            try:
                self._cameras[cam_cfg.name] = MuJoCoCamera(self._model, cam_cfg)
            except Exception as e:
                print(f'[MuJoCoEngine] Warning: {e}')

        # Initialize policy
        policy_path = self._robot_cfg.policy_onnx
        if policy_path:
            if Path(policy_path).exists():
                self._policy = PolicyRunner(policy_path)
            else:
                print(f'[MuJoCoEngine] Policy not found at {policy_path}, '
                      f'running PD-only mode')
                self._policy = None
        else:
            print('[MuJoCoEngine] No policy configured, running PD-only mode')
            self._policy = None

        self._running = True
        print(f'[MuJoCoEngine] Loaded. dt={self._physics_dt:.4f}s, '
              f'joints={len(self._leg_joint_ids)}, '
              f'ctrl_offset={self._leg_actuator_offset}, '
              f'cameras={list(self._cameras.keys())}')

    def _resolve_ids(self) -> None:
        """Resolve body/joint IDs in MuJoCo.
        # Extracted from sim/bridge/nova_nav_bridge.py NavBridge.__init__
        """
        import mujoco

        self._base_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY,
            self._robot_cfg.base_body_name
        )
        self._lidar_body_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_BODY,
            self._lidar_cfg.body_name
        )
        self._leg_joint_ids = [
            mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, n)
            for n in LEG_JOINT_NAMES
        ]
        missing = [n for n, jid in zip(LEG_JOINT_NAMES, self._leg_joint_ids) if jid < 0]
        if missing:
            print(f'[MuJoCoEngine] Warning: joints not found: {missing}')

    def reset(self) -> RobotState:
        """Reset simulation to initial state."""
        import mujoco

        mujoco.mj_resetData(self._model, self._data)

        # Set initial position
        pos = self._robot_cfg.init_position
        q_wxyz = self._robot_cfg.init_orientation_wxyz
        self._data.qpos[0:3] = pos
        self._data.qpos[3] = q_wxyz[0]   # w
        self._data.qpos[4] = q_wxyz[1]   # x
        self._data.qpos[5] = q_wxyz[2]   # y
        self._data.qpos[6] = q_wxyz[3]   # z

        # Set initial standing joint angles
        self._apply_standing_pose()

        # Zero arm joints ctrl (required by original bridge)
        offset = self._leg_actuator_offset
        for i in range(offset):
            if i < len(self._data.ctrl):
                self._data.ctrl[i] = 0.0

        mujoco.mj_forward(self._model, self._data)

        # Stabilization phase: run 1000 steps (2s) with PD control, no policy
        # (policy outputs bad actions from initial state and would fling the robot)
        _policy_backup = self._policy
        self._policy = None  # temporarily disable policy
        for _ in range(1000):
            mujoco.mj_step(self._model, self._data)
        self._policy = _policy_backup  # restore policy

        self._sim_time = 0.0

        # Reset policy history (use stabilized real sensor data)
        if self._policy is not None:
            self._policy.reset()
            gyro, pg = self._get_imu()
            jp, jv = self._get_joint_state()
            self._policy.warm_up(gyro, pg, jp, jv)

        # Update LiDAR data reference
        if self._lidar is not None:
            self._lidar.update_data(self._data)

        with self._lock:
            self._cmd_vel[:] = 0.0
            self._cmd_vel_time = 0.0

        return self.get_robot_state()

    def _apply_standing_pose(self) -> None:
        """Write standing pose to joint ctrl (used during initialization)."""
        if not self._leg_joint_ids:
            return
        import mujoco

        standing_dart = self._robot_cfg.standing_pose_array
        standing_mj = standing_dart[self._robot_cfg.dart_to_mj_array]
        offset = self._leg_actuator_offset

        for i, jid in enumerate(self._leg_joint_ids):
            if jid < 0:
                continue
            qadr = self._model.jnt_qposadr[jid]
            self._data.qpos[qadr] = standing_mj[i]
            if offset + i < len(self._data.ctrl):
                self._data.ctrl[offset + i] = standing_mj[i]

    def close(self) -> None:
        """Release all resources."""
        self._running = False
        self._stop_event.set()
        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=2.0)
        for cam in self._cameras.values():
            cam.close()
        self._cameras.clear()
        print('[MuJoCoEngine] Closed.')

    # ──────────────────────────────────────────────────────────────
    # Simulation stepping
    # ──────────────────────────────────────────────────────────────

    def step(self, cmd: Optional[VelocityCommand] = None) -> RobotState:
        """Advance one control cycle (policy_dt = 0.02s = 10 physics steps).

        Args:
            cmd: velocity command; if given, updates internal cmd_vel.

        Returns:
            robot state after stepping
        """
        import mujoco

        if cmd is not None:
            self.set_cmd_vel(cmd)

        # Watchdog: zero out cmd_vel if no command within 200ms
        self._watchdog_cmd_vel()

        # Run policy inference, write to ctrl
        self._step_policy()

        # Physics stepping (policy_dt / physics_dt steps)
        n_sub = max(1, round(self._control_dt / self._physics_dt))
        for _ in range(n_sub):
            mujoco.mj_step(self._model, self._data)
        self._sim_time += self._control_dt

        return self.get_robot_state()

    def step_physics(self, n_steps: int = 1) -> None:
        """Pure physics step, no policy inference."""
        import mujoco

        for _ in range(n_steps):
            mujoco.mj_step(self._model, self._data)
        self._sim_time += self._physics_dt * n_steps

    def _watchdog_cmd_vel(self) -> None:
        """Zero out cmd_vel if watchdog timeout exceeded.
        # Extracted from sim/bridge/nova_nav_bridge.py _watchdog_cmd_vel
        """
        with self._lock:
            if (self._cmd_vel_time > 0 and
                    time.time() - self._cmd_vel_time >
                    self._robot_cfg.cmd_vel_watchdog_sec):
                self._cmd_vel[:] = 0.0

    def _step_policy(self) -> None:
        """Run one policy inference step, write action to MuJoCo ctrl.
        # Extracted from sim/bridge/nova_nav_bridge.py NavBridge.step_policy
        """
        with self._lock:
            direction = self._cmd_vel.copy()

        gyro, pg = self._get_imu()
        jp, jv = self._get_joint_state()

        if self._policy is not None:
            obs = self._policy.build_obs(gyro, pg, direction, jp, jv)
            action_dart = self._policy.infer(obs)     # (16,) Dart order
            # Dart -> MuJoCo order
            action_mj = action_dart[DART_TO_MJ]
            offset = self._leg_actuator_offset
            ctrl_span = min(len(action_mj), max(0, len(self._data.ctrl) - offset))
            self._data.ctrl[offset:offset + ctrl_span] = action_mj[:ctrl_span]
        else:
            # No policy: hold standing pose
            standing_dart = self._robot_cfg.standing_pose_array
            action_mj = standing_dart[DART_TO_MJ]
            offset = self._leg_actuator_offset
            ctrl_span = min(len(action_mj), max(0, len(self._data.ctrl) - offset))
            self._data.ctrl[offset:offset + ctrl_span] = action_mj[:ctrl_span]

    # ──────────────────────────────────────────────────────────────
    # State reading
    # ──────────────────────────────────────────────────────────────

    def get_robot_state(self) -> RobotState:
        """Read current robot state snapshot."""
        pos = self._data.qpos[:3].copy()
        quat_wxyz = self._data.qpos[3:7].copy()
        # MuJoCo quaternion w,x,y,z -> ROS x,y,z,w
        orientation = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
        linear_vel = self._data.qvel[:3].copy()
        angular_vel = self._data.qvel[3:6].copy()

        jp, jv = self._get_joint_state()
        gyro, pg = self._get_imu()

        return RobotState(
            position=pos,
            orientation=orientation,
            linear_velocity=linear_vel,
            angular_velocity=angular_vel,
            joint_positions=jp,
            joint_velocities=jv,
            imu_gyro=gyro,
            imu_projected_gravity=pg,
        )

    def _get_imu(self):
        """Extract IMU data (gyroscope + projected gravity).
        # Extracted from sim/bridge/nova_nav_bridge.py get_imu()
        """
        body_id = self._base_body_id
        R = self._data.xmat[body_id].reshape(3, 3)  # body-to-world
        omega_world = self._data.qvel[3:6]
        gyroscope = R.T @ omega_world                 # world -> body frame

        gravity_world = np.array([0.0, 0.0, -1.0])
        projected_gravity = R.T @ gravity_world

        return gyroscope.astype(np.float64), projected_gravity.astype(np.float64)

    def _get_joint_state(self):
        """Read position and velocity of 16 leg joints.
        # Extracted from sim/bridge/nova_nav_bridge.py get_joint_state()
        """
        valid_ids = [j for j in self._leg_joint_ids if j >= 0]
        if not valid_ids:
            return np.zeros(16), np.zeros(16)

        pos = np.array([self._data.qpos[self._model.jnt_qposadr[j]]
                        for j in self._leg_joint_ids
                        if j >= 0], dtype=np.float64)
        vel = np.array([self._data.qvel[self._model.jnt_dofadr[j]]
                        for j in self._leg_joint_ids
                        if j >= 0], dtype=np.float64)

        # Pad to 16 if some joints are missing
        if len(pos) < 16:
            pos = np.pad(pos, (0, 16 - len(pos)))
            vel = np.pad(vel, (0, 16 - len(vel)))
        return pos, vel

    def get_camera_data(self, camera_name: str = "front_camera") -> Optional[CameraData]:
        """Read camera frame (RGB + depth)."""
        if camera_name not in self._cameras:
            return None
        return self._cameras[camera_name].render(self._data)

    def get_lidar_points(self) -> np.ndarray:
        """Read current LiDAR point cloud.

        Returns:
            (N, 4) float32 XYZI world frame, or (N, 3) from livox_mid360 module
        """
        if self._lidar is None:
            return np.zeros((0, 4), dtype=np.float32)
        return self._lidar.scan()

    # ──────────────────────────────────────────────────────────────
    # Control interface
    # ──────────────────────────────────────────────────────────────

    def set_cmd_vel(self, cmd: VelocityCommand) -> None:
        """Set velocity command (thread-safe)."""
        with self._lock:
            self._cmd_vel[0] = np.clip(cmd.linear_x,
                                       -self._robot_cfg.max_linear_vel,
                                        self._robot_cfg.max_linear_vel)
            self._cmd_vel[1] = np.clip(cmd.linear_y,
                                       -self._robot_cfg.max_linear_vel,
                                        self._robot_cfg.max_linear_vel)
            self._cmd_vel[2] = np.clip(cmd.angular_z,
                                       -self._robot_cfg.max_angular_vel,
                                        self._robot_cfg.max_angular_vel)
            self._cmd_vel_time = time.time()

    def set_joint_positions(self, positions: np.ndarray) -> None:
        """Directly set joint target positions (bypasses ONNX policy)."""
        if len(positions) < 16:
            raise ValueError(f"Expected 16 joint positions, got {len(positions)}")
        offset = self._leg_actuator_offset
        ctrl_span = min(16, max(0, len(self._data.ctrl) - offset))
        self._data.ctrl[offset:offset + ctrl_span] = positions[:ctrl_span]

    # ──────────────────────────────────────────────────────────────
    # Scene manipulation
    # ──────────────────────────────────────────────────────────────

    def set_robot_pose(self, position: np.ndarray,
                       orientation: np.ndarray) -> None:
        """Teleport robot to a given pose (ROS quaternion x,y,z,w)."""
        import mujoco

        self._data.qpos[0:3] = position[:3]
        # ROS x,y,z,w -> MuJoCo w,x,y,z
        self._data.qpos[3] = orientation[3]  # w
        self._data.qpos[4] = orientation[0]  # x
        self._data.qpos[5] = orientation[1]  # y
        self._data.qpos[6] = orientation[2]  # z
        # Zero velocities
        self._data.qvel[:] = 0.0
        mujoco.mj_forward(self._model, self._data)

    def add_obstacle(self, name: str, shape: str, size: List[float],
                     position: List[float],
                     rgba: Optional[List[float]] = None) -> None:
        """Dynamically add obstacle (MuJoCo does not support runtime geom addition).

        Note: MuJoCo does not support modifying model structure during physics stepping.
        Adding an obstacle requires a reload via load(). This method queues the request
        in world_config; it takes effect on next reset()+load().
        """
        from sim.engine.core.world import ObstacleConfig
        obs = ObstacleConfig(
            name=name,
            shape=shape,
            size=size,
            position=position,
            rgba=rgba or [0.7, 0.7, 0.7, 1.0],
        )
        self._world_cfg.obstacles.append(obs)
        print(f'[MuJoCoEngine] Obstacle "{name}" queued — call load() to apply.')

    # ──────────────────────────────────────────────────────────────
    # Properties
    # ──────────────────────────────────────────────────────────────

    @property
    def dt(self) -> float:
        return self._physics_dt

    @property
    def control_dt(self) -> float:
        return self._control_dt

    @property
    def model(self):
        """MuJoCo MjModel (for direct access)."""
        return self._model

    @property
    def data(self):
        """MuJoCo MjData (for direct access)."""
        return self._data

    @property
    def has_policy(self) -> bool:
        return self._policy is not None

    # ──────────────────────────────────────────────────────────────
    # Background physics thread (optional, for async stepping)
    # ──────────────────────────────────────────────────────────────

    def start_background_sim(self) -> None:
        """Run physics stepping in a background thread (separate from rendering)."""
        if self._sim_thread and self._sim_thread.is_alive():
            return
        self._stop_event.clear()
        self._sim_thread = threading.Thread(
            target=self._background_sim_loop,
            name="MuJoCoSimThread",
            daemon=True,
        )
        self._sim_thread.start()

    def stop_background_sim(self) -> None:
        """Stop the background physics thread."""
        self._stop_event.set()
        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=3.0)
        self._sim_thread = None

    def _background_sim_loop(self) -> None:
        """Background physics loop: 50Hz policy + 500Hz physics.
        # Extracted from sim/bridge/nova_nav_bridge.py NavBridge.spin()
        """
        import mujoco

        last_policy = 0.0
        print(f'[MuJoCoEngine] Background sim started: '
              f'physics={1/self._physics_dt:.0f}Hz, '
              f'policy={1/self._control_dt:.0f}Hz')

        while not self._stop_event.is_set():
            t0 = time.time()

            with self._lock:
                pass  # acquire lock pattern

            mujoco.mj_step(self._model, self._data)
            self._sim_time += self._physics_dt

            if self._sim_time - last_policy >= self._control_dt:
                self._watchdog_cmd_vel()
                self._step_policy()
                last_policy = self._sim_time

            elapsed = time.time() - t0
            sleep_t = self._physics_dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

        print('[MuJoCoEngine] Background sim stopped.')
