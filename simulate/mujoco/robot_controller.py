"""ONNX 策略控制器 — PolicyRunner
# Extracted from sim/bridge/nova_nav_bridge.py

原始代码: sim/bridge/nova_nav_bridge.py
提取内容: PolicyRunner 类 + 所有相关常量
保留原始逻辑，无修改。
"""
from collections import deque
from typing import Optional

import numpy as np

# ── 原始常量 (直接从 nova_nav_bridge.py 搬运) ──────────────────────
# Extracted from sim/bridge/nova_nav_bridge.py

# Dart standingPose (直接用, 不取反)
STANDING_POSE = np.array([
    -0.1, -0.8,  1.8,     # FR: hip, thigh, calf
     0.1,  0.8, -1.8,     # FL
     0.1,  0.8, -1.8,     # RR
    -0.1, -0.8,  1.8,     # RL
     0.0,  0.0,  0.0, 0.0 # foot
], dtype=np.float64)

ACTION_SCALE = np.array([
    0.125, 0.25, 0.25,  # FR
    0.125, 0.25, 0.25,  # FL
    0.125, 0.25, 0.25,  # RR
    0.125, 0.25, 0.25,  # RL
    5.0, 5.0, 5.0, 5.0  # foot
], dtype=np.float64)

JOINT_VEL_SCALE = np.array([0.05, 0.05, 0.05, 0.05], dtype=np.float64)
IMU_GYRO_SCALE = 0.25
OBS_DIM = 57
HISTORY_LEN = 5

# MuJoCo ↔ Dart/ONNX 关节顺序转换
# MuJoCo (4+4+4+4): FR(hip,thigh,calf,foot), FL(...), RR(...), RL(...)
# Dart   (3+3+3+3+4): FR(hip,thigh,calf), FL(h,t,c), RR, RL, FR_f, FL_f, RR_f, RL_f
MJ_TO_DART = np.array([0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 3, 7, 11, 15])
DART_TO_MJ = np.array([0, 1, 2, 12, 3, 4, 5, 13, 6, 7, 8, 14, 9, 10, 11, 15])


class PolicyRunner:
    """ONNX 步态策略推理, 匹配 brainstem StandardObservationBuilder.

    # Extracted from sim/bridge/nova_nav_bridge.py — 原封不动保留逻辑
    """

    def __init__(self, onnx_path: str):
        import onnxruntime as ort
        self.session = ort.InferenceSession(
            onnx_path, providers=['CPUExecutionProvider'])
        inp = self.session.get_inputs()[0]
        out = self.session.get_outputs()[0]
        print(f'[Policy] Loaded {onnx_path}')
        print(f'  input:  {inp.name} {inp.shape}')
        print(f'  output: {out.name} {out.shape}')
        self.input_name = inp.name
        self.output_name = out.name

        # 自动检测 history 长度: policy 输入维度 / OBS_DIM
        policy_input_dim = inp.shape[1] if len(inp.shape) > 1 else inp.shape[0]
        self._history_len = max(1, policy_input_dim // OBS_DIM)
        self.history: deque = deque(maxlen=self._history_len)
        self.last_action = STANDING_POSE.copy()
        # 将在 warm_up() 中用真实传感器数据填充

    def warm_up(self, gyroscope: np.ndarray, projected_gravity: np.ndarray,
                joint_pos_16: np.ndarray, joint_vel_16: np.ndarray) -> None:
        """用真实传感器数据填充 history buffer (匹配 brainstem Memory 初始化)."""
        init_obs = self.build_obs(
            gyroscope, projected_gravity,
            np.zeros(3),  # direction = 0 (idle)
            joint_pos_16, joint_vel_16)
        self.history.clear()
        for _ in range(self._history_len):
            self.history.append(init_obs.copy())
        print(f'[Policy] History warmed up: pg={projected_gravity[:3]}')

    @staticmethod
    def clamp_action(action: np.ndarray) -> np.ndarray:
        """clampPerJoint — 匹配 brainstem Walk.clampAction()."""
        clamped = action.copy()
        for leg in range(4):
            base = leg * 3
            clamped[base + 0] = np.clip(clamped[base + 0], -0.5, 0.5)    # hip
            clamped[base + 1] = np.clip(clamped[base + 1], -1.5, 1.5)    # thigh
            clamped[base + 2] = np.clip(clamped[base + 2], -2.5, 2.5)    # calf
        for i in range(12, 16):
            clamped[i] = np.clip(clamped[i], -0.5, 0.5)  # foot
        return clamped

    def build_obs(self, gyroscope: np.ndarray, projected_gravity: np.ndarray,
                  direction: np.ndarray, joint_pos_16: np.ndarray,
                  joint_vel_16: np.ndarray) -> np.ndarray:
        """构建 57 维观测, 完全匹配 brainstem StandardObservationBuilder.

        注意: joint_pos_16/joint_vel_16 是 MuJoCo 顺序 (4+4+4+4),
        需要先转换到 Dart 顺序 (3+3+3+3+4) 再计算.
        """
        gyro = gyroscope * IMU_GYRO_SCALE
        pg = projected_gravity

        # MuJoCo → Dart 关节顺序转换
        jp_dart = joint_pos_16[MJ_TO_DART]
        jv_dart = joint_vel_16[MJ_TO_DART]

        # joint position: (current - standingPose), foot 置零
        jp = jp_dart - STANDING_POSE
        jp[12:] = 0.0  # discardFoot()

        # joint velocity: scale(hip, thigh, calf, foot)
        # brainstem: jointVelocity * (0.05, 0.05, 0.05, 0.05)
        jv = jv_dart.copy()
        for leg in range(4):
            base = leg * 3
            jv[base + 0] *= JOINT_VEL_SCALE[0]  # hip
            jv[base + 1] *= JOINT_VEL_SCALE[1]  # thigh
            jv[base + 2] *= JOINT_VEL_SCALE[2]  # calf
        for i in range(12, 16):
            jv[i] *= JOINT_VEL_SCALE[3]  # foot

        # last action: (action - standingPose) / actionScale
        # last_action 已经是 Dart 顺序
        act = (self.last_action - STANDING_POSE) / ACTION_SCALE

        obs = np.concatenate([gyro, pg, direction, jp, jv, act]).astype(np.float32)
        return obs

    def infer(self, obs: np.ndarray) -> np.ndarray:
        """推理: obs(57) → 加入 history → concat(285) → ONNX → action(16)."""
        self.history.append(obs)
        obs_history = np.concatenate(list(self.history)).reshape(1, -1).astype(np.float32)
        result = self.session.run([self.output_name],
                                  {self.input_name: obs_history})[0]
        raw_action = result[0]  # (16,)

        # realAction = clamp(onnxOutput * actionScale + standingPose)
        real_action = raw_action * ACTION_SCALE + STANDING_POSE
        real_action = self.clamp_action(real_action)
        self.last_action = real_action.copy()
        return real_action

    def reset(self) -> None:
        """重置 history buffer（仿真 reset 时调用）."""
        self.history.clear()
        self.last_action = STANDING_POSE.copy()
