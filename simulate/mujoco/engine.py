"""MuJoCo 仿真引擎实现
# Extracted from sim/bridge/nova_nav_bridge.py

核心逻辑来自 nova_nav_bridge.py：
  - MuJoCo 模型加载和场景 XML 生成
  - 物理步进循环（mujoco.mj_step）
  - cmd_vel → ONNX policy → 关节控制
  - IMU / 关节状态读取（get_imu, get_joint_state）
  - LiDAR 扫描封装
"""
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from simulate.core.engine import CameraData, RobotState, SimEngine, VelocityCommand
from simulate.core.robot import RobotConfig
from simulate.core.sensor import CameraConfig, IMUConfig, LidarConfig
from simulate.core.world import WorldConfig, SimWorld
from simulate.mujoco.camera import MuJoCoCamera
from simulate.mujoco.lidar import MuJoCoLidar
from simulate.mujoco.robot_controller import (
    DART_TO_MJ,
    MJ_TO_DART,
    STANDING_POSE,
    PolicyRunner,
)


# Actuator 偏移（前 8 个 actuator 是臂，腿从 index 8 开始）
# Extracted from sim/bridge/nova_nav_bridge.py
LEG_JOINT_NAMES = [
    'fr_hip_joint', 'fr_thigh_joint', 'fr_calf_joint', 'fr_foot_joint',
    'fl_hip_joint', 'fl_thigh_joint', 'fl_calf_joint', 'fl_foot_joint',
    'rr_hip_joint', 'rr_thigh_joint', 'rr_calf_joint', 'rr_foot_joint',
    'rl_hip_joint', 'rl_thigh_joint', 'rl_calf_joint', 'rl_foot_joint',
]


class MuJoCoEngine(SimEngine):
    """MuJoCo 仿真引擎.

    实现 SimEngine 抽象接口，封装：
      - 场景加载（WorldConfig + RobotConfig）
      - 物理步进（500Hz physics, 50Hz policy）
      - ONNX 步态策略（PolicyRunner）
      - LiDAR 扫描（MuJoCoLidar）
      - 相机渲染（MuJoCoCamera）
    """

    def __init__(self,
                 robot_config: Optional[RobotConfig] = None,
                 world_config: Optional[WorldConfig] = None,
                 lidar_config: Optional[LidarConfig] = None,
                 camera_configs: Optional[List[CameraConfig]] = None,
                 imu_config: Optional[IMUConfig] = None,
                 headless: bool = True) -> None:
        """初始化 MuJoCo 引擎（不加载模型，调用 load() 后才可用）.

        Args:
            robot_config: 机器人配置，默认 NOVA Dog
            world_config: 场景配置，默认平地
            lidar_config: LiDAR 配置，默认 MID-360
            camera_configs: 相机配置列表
            imu_config: IMU 配置
            headless: True=无头模式，False=启动 MuJoCo viewer
        """
        super().__init__()

        self._robot_cfg = robot_config or RobotConfig.default_nova_dog()
        self._world_cfg = world_config or WorldConfig()
        self._lidar_cfg = lidar_config or LidarConfig()
        self._camera_cfgs = camera_configs or []
        self._imu_cfg = imu_config or IMUConfig()
        self._headless = headless

        # MuJoCo 核心对象（load() 后初始化）
        self._model = None
        self._data = None

        # 传感器
        self._lidar: Optional[MuJoCoLidar] = None
        self._cameras: Dict[str, MuJoCoCamera] = {}

        # 策略控制器
        self._policy: Optional[PolicyRunner] = None

        # 关节 ID 列表（load() 后填充）
        self._leg_joint_ids: List[int] = []
        self._base_body_id: int = 0
        self._lidar_body_id: int = 0

        # cmd_vel 线程安全
        self._cmd_vel = np.zeros(3, dtype=np.float64)  # [vx, vy, wz]
        self._cmd_vel_time = 0.0
        self._lock = threading.Lock()

        # 物理参数
        self._physics_dt: float = 0.002   # 从 model.opt.timestep 读取
        self._control_dt: float = 0.02    # 50Hz policy

        # 物理线程
        self._stop_event = threading.Event()
        self._sim_thread: Optional[threading.Thread] = None

    # ──────────────────────────────────────────────────────────────
    # 生命周期
    # ──────────────────────────────────────────────────────────────

    def load(self, xml_path: str = "", **kwargs: Any) -> None:
        """加载场景并初始化所有子系统.

        Args:
            xml_path: 场景 XML 路径。若为空则从 WorldConfig 生成。
            **kwargs: 额外参数（ignored，兼容接口）
        """
        import mujoco

        # 解析机器人 XML 路径
        self._robot_cfg.resolve_paths()
        robot_xml = self._robot_cfg.robot_xml

        if xml_path:
            # 直接使用给定路径
            self._model = mujoco.MjModel.from_xml_path(xml_path)
        elif self._world_cfg.scene_xml and Path(self._world_cfg.scene_xml).exists():
            self._model = mujoco.MjModel.from_xml_path(self._world_cfg.scene_xml)
        else:
            # 从 WorldConfig 动态生成场景 XML
            world = SimWorld(self._world_cfg)
            scene_xml_str = world.get_scene_xml(robot_xml)
            # 写入临时文件（MuJoCo 需要文件路径处理 <include>）
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

        # 解析 body/joint IDs
        self._resolve_ids()

        # 初始化 LiDAR
        self._lidar = MuJoCoLidar(self._model, self._data, self._lidar_cfg)

        # 初始化相机
        for cam_cfg in self._camera_cfgs:
            try:
                self._cameras[cam_cfg.name] = MuJoCoCamera(self._model, cam_cfg)
            except ValueError as e:
                print(f'[MuJoCoEngine] Warning: {e}')

        # 初始化策略
        policy_path = self._robot_cfg.policy_onnx
        if policy_path and Path(policy_path).exists():
            self._policy = PolicyRunner(policy_path)
        else:
            print(f'[MuJoCoEngine] Policy not found at {policy_path}, '
                  f'running PD-only mode')
            self._policy = None

        self._running = True
        print(f'[MuJoCoEngine] Loaded. dt={self._physics_dt:.4f}s, '
              f'joints={len(self._leg_joint_ids)}, '
              f'cameras={list(self._cameras.keys())}')

    def _resolve_ids(self) -> None:
        """解析 MuJoCo 中的 body/joint ID。
        # Extracted from sim/bridge/nova_nav_bridge.py — NavBridge.__init__
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
        """重置仿真到初始状态."""
        import mujoco

        mujoco.mj_resetData(self._model, self._data)

        # 设置初始位置
        pos = self._robot_cfg.init_position
        q_wxyz = self._robot_cfg.init_orientation_wxyz
        self._data.qpos[0:3] = pos
        self._data.qpos[3] = q_wxyz[0]   # w
        self._data.qpos[4] = q_wxyz[1]   # x
        self._data.qpos[5] = q_wxyz[2]   # y
        self._data.qpos[6] = q_wxyz[3]   # z

        # 设置站立初始关节角
        self._apply_standing_pose()

        mujoco.mj_forward(self._model, self._data)
        self._sim_time = 0.0

        # 重置策略 history
        if self._policy is not None:
            self._policy.reset()
            gyro, pg = self._get_imu()
            jp, jv = self._get_joint_state()
            self._policy.warm_up(gyro, pg, jp, jv)

        # 更新 LiDAR data 引用
        if self._lidar is not None:
            self._lidar.update_data(self._data)

        with self._lock:
            self._cmd_vel[:] = 0.0
            self._cmd_vel_time = 0.0

        return self.get_robot_state()

    def _apply_standing_pose(self) -> None:
        """将站立姿态写入关节 ctrl（初始化用）."""
        if not self._leg_joint_ids:
            return
        import mujoco

        standing_dart = self._robot_cfg.standing_pose_array
        standing_mj = standing_dart[self._robot_cfg.dart_to_mj_array]
        offset = self._robot_cfg.leg_act_offset

        for i, jid in enumerate(self._leg_joint_ids):
            if jid < 0:
                continue
            qadr = self._model.jnt_qposadr[jid]
            self._data.qpos[qadr] = standing_mj[i]
            if offset + i < len(self._data.ctrl):
                self._data.ctrl[offset + i] = standing_mj[i]

    def close(self) -> None:
        """释放所有资源."""
        self._running = False
        self._stop_event.set()
        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=2.0)
        for cam in self._cameras.values():
            cam.close()
        self._cameras.clear()
        print('[MuJoCoEngine] Closed.')

    # ──────────────────────────────────────────────────────────────
    # 仿真步进
    # ──────────────────────────────────────────────────────────────

    def step(self, cmd: Optional[VelocityCommand] = None) -> RobotState:
        """推进一个控制周期（policy_dt = 0.02s = 10 physics steps）.

        Args:
            cmd: 速度指令。若给定则更新内部 cmd_vel。

        Returns:
            步进后的机器人状态
        """
        import mujoco

        if cmd is not None:
            self.set_cmd_vel(cmd)

        # watchdog：200ms 无指令自动归零
        self._watchdog_cmd_vel()

        # 执行策略推理，写入 ctrl
        self._step_policy()

        # 物理步进（policy_dt / physics_dt 步）
        n_sub = max(1, round(self._control_dt / self._physics_dt))
        for _ in range(n_sub):
            mujoco.mj_step(self._model, self._data)
        self._sim_time += self._control_dt

        return self.get_robot_state()

    def step_physics(self, n_steps: int = 1) -> None:
        """纯物理步进，不执行策略推理."""
        import mujoco

        for _ in range(n_steps):
            mujoco.mj_step(self._model, self._data)
        self._sim_time += self._physics_dt * n_steps

    def _watchdog_cmd_vel(self) -> None:
        """超过 watchdog 时限则归零 cmd_vel.
        # Extracted from sim/bridge/nova_nav_bridge.py — _watchdog_cmd_vel
        """
        with self._lock:
            if (self._cmd_vel_time > 0 and
                    time.time() - self._cmd_vel_time >
                    self._robot_cfg.cmd_vel_watchdog_sec):
                self._cmd_vel[:] = 0.0

    def _step_policy(self) -> None:
        """执行一步策略推理，将 action 写入 MuJoCo ctrl.
        # Extracted from sim/bridge/nova_nav_bridge.py — NavBridge.step_policy
        """
        with self._lock:
            direction = self._cmd_vel.copy()

        gyro, pg = self._get_imu()
        jp, jv = self._get_joint_state()

        if self._policy is not None:
            obs = self._policy.build_obs(gyro, pg, direction, jp, jv)
            action_dart = self._policy.infer(obs)     # (16,) Dart 顺序
            # Dart → MuJoCo 顺序
            action_mj = action_dart[DART_TO_MJ]
            offset = self._robot_cfg.leg_act_offset
            self._data.ctrl[offset:offset + 16] = action_mj
        else:
            # 无策略：保持站立姿态
            standing_dart = self._robot_cfg.standing_pose_array
            action_mj = standing_dart[DART_TO_MJ]
            offset = self._robot_cfg.leg_act_offset
            self._data.ctrl[offset:offset + 16] = action_mj

    # ──────────────────────────────────────────────────────────────
    # 状态读取
    # ──────────────────────────────────────────────────────────────

    def get_robot_state(self) -> RobotState:
        """读取当前机器人状态快照."""
        pos = self._data.qpos[:3].copy()
        quat_wxyz = self._data.qpos[3:7].copy()
        # MuJoCo 四元数 w,x,y,z → ROS x,y,z,w
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
        """提取 IMU 数据（陀螺仪 + 重力投影）.
        # Extracted from sim/bridge/nova_nav_bridge.py — get_imu()
        """
        body_id = self._base_body_id
        R = self._data.xmat[body_id].reshape(3, 3)  # body-to-world
        omega_world = self._data.qvel[3:6]
        gyroscope = R.T @ omega_world                 # world → body frame

        gravity_world = np.array([0.0, 0.0, -1.0])
        projected_gravity = R.T @ gravity_world

        return gyroscope.astype(np.float64), projected_gravity.astype(np.float64)

    def _get_joint_state(self):
        """读取 16 个腿关节的位置和速度.
        # Extracted from sim/bridge/nova_nav_bridge.py — get_joint_state()
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

        # 若部分关节缺失，填充到 16 维
        if len(pos) < 16:
            pos = np.pad(pos, (0, 16 - len(pos)))
            vel = np.pad(vel, (0, 16 - len(vel)))
        return pos, vel

    def get_camera_data(self, camera_name: str = "front_camera") -> Optional[CameraData]:
        """读取相机帧（RGB + 深度）."""
        if camera_name not in self._cameras:
            return None
        return self._cameras[camera_name].render(self._data)

    def get_lidar_points(self) -> np.ndarray:
        """读取当前 LiDAR 点云.

        Returns:
            (N, 4) float32 XYZI 世界坐标系，或 (N, 3) 若使用 livox_mid360 模块
        """
        if self._lidar is None:
            return np.zeros((0, 4), dtype=np.float32)
        return self._lidar.scan()

    # ──────────────────────────────────────────────────────────────
    # 控制接口
    # ──────────────────────────────────────────────────────────────

    def set_cmd_vel(self, cmd: VelocityCommand) -> None:
        """设置速度指令（线程安全）."""
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
        """直接设置关节目标位置（绕过 ONNX 策略）."""
        if len(positions) < 16:
            raise ValueError(f"Expected 16 joint positions, got {len(positions)}")
        offset = self._robot_cfg.leg_act_offset
        self._data.ctrl[offset:offset + 16] = positions[:16]

    # ──────────────────────────────────────────────────────────────
    # 场景操作
    # ──────────────────────────────────────────────────────────────

    def set_robot_pose(self, position: np.ndarray,
                       orientation: np.ndarray) -> None:
        """传送机器人到指定位姿（ROS 四元数 x,y,z,w）."""
        import mujoco

        self._data.qpos[0:3] = position[:3]
        # ROS x,y,z,w → MuJoCo w,x,y,z
        self._data.qpos[3] = orientation[3]  # w
        self._data.qpos[4] = orientation[0]  # x
        self._data.qpos[5] = orientation[1]  # y
        self._data.qpos[6] = orientation[2]  # z
        # 清零速度
        self._data.qvel[:] = 0.0
        mujoco.mj_forward(self._model, self._data)

    def add_obstacle(self, name: str, shape: str, size: List[float],
                     position: List[float],
                     rgba: Optional[List[float]] = None) -> None:
        """动态添加障碍物（MuJoCo 不支持运行时添加 geom，打印提示）.

        注意：MuJoCo 不支持在物理步进中动态修改模型结构。
        添加障碍物需要重新 load()。此方法将请求记录到 world_config，
        下次 reset()+load() 时生效。
        """
        from simulate.core.world import ObstacleConfig
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
    # 属性
    # ──────────────────────────────────────────────────────────────

    @property
    def dt(self) -> float:
        return self._physics_dt

    @property
    def control_dt(self) -> float:
        return self._control_dt

    @property
    def model(self):
        """MuJoCo MjModel（用于直接访问）."""
        return self._model

    @property
    def data(self):
        """MuJoCo MjData（用于直接访问）."""
        return self._data

    @property
    def has_policy(self) -> bool:
        return self._policy is not None

    # ──────────────────────────────────────────────────────────────
    # 后台物理线程（可选，用于独立于主循环的异步步进）
    # ──────────────────────────────────────────────────────────────

    def start_background_sim(self) -> None:
        """在后台线程运行物理步进（headless 下与渲染分离）."""
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
        """停止后台物理线程."""
        self._stop_event.set()
        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=3.0)
        self._sim_thread = None

    def _background_sim_loop(self) -> None:
        """后台物理步进循环（50Hz policy + 500Hz physics）.
        # Extracted from sim/bridge/nova_nav_bridge.py — NavBridge.spin()
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
