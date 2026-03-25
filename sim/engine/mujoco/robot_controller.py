"""ONNX policy controller — PolicyRunner
# Extracted from sim/bridge/nova_nav_bridge.py

Original source: sim/bridge/nova_nav_bridge.py
Extracted content: PolicyRunner class + all related constants
Logic preserved exactly as-is.
"""
from collections import deque
from typing import Optional

import numpy as np

# ── Original constants (ported directly from nova_nav_bridge.py) ──────────────
# Extracted from sim/bridge/nova_nav_bridge.py

# Dart standingPose (use as-is, no sign flip)
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

# MuJoCo <-> Dart/ONNX joint order mapping
# MuJoCo (4+4+4+4): FR(hip,thigh,calf,foot), FL(...), RR(...), RL(...)
# Dart   (3+3+3+3+4): FR(hip,thigh,calf), FL(h,t,c), RR, RL, FR_f, FL_f, RR_f, RL_f
MJ_TO_DART = np.array([0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 3, 7, 11, 15])
DART_TO_MJ = np.array([0, 1, 2, 12, 3, 4, 5, 13, 6, 7, 8, 14, 9, 10, 11, 15])


class PolicyRunner:
    """ONNX gait policy inference, matching brainstem StandardObservationBuilder.

    # Extracted from sim/bridge/nova_nav_bridge.py — logic preserved exactly.
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

        # Auto-detect history length: policy input dim / OBS_DIM
        policy_input_dim = inp.shape[1] if len(inp.shape) > 1 else inp.shape[0]
        self._history_len = max(1, policy_input_dim // OBS_DIM)
        self.history: deque = deque(maxlen=self._history_len)
        self.last_action = STANDING_POSE.copy()
        # Will be filled with real sensor data in warm_up()

    def warm_up(self, gyroscope: np.ndarray, projected_gravity: np.ndarray,
                joint_pos_16: np.ndarray, joint_vel_16: np.ndarray) -> None:
        """Fill history buffer with real sensor data (matches brainstem Memory init)."""
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
        """clampPerJoint — matches brainstem Walk.clampAction()."""
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
        """Build 57-dim observation, fully matching brainstem StandardObservationBuilder.

        Note: joint_pos_16/joint_vel_16 are in MuJoCo order (4+4+4+4).
        They must be converted to Dart order (3+3+3+3+4) before computation.
        """
        gyro = gyroscope * IMU_GYRO_SCALE
        pg = projected_gravity

        # MuJoCo -> Dart joint order conversion
        jp_dart = joint_pos_16[MJ_TO_DART]
        jv_dart = joint_vel_16[MJ_TO_DART]

        # joint position: (current - standingPose), foot zeroed out
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
        # last_action is already in Dart order
        act = (self.last_action - STANDING_POSE) / ACTION_SCALE

        obs = np.concatenate([gyro, pg, direction, jp, jv, act]).astype(np.float32)
        return obs

    def infer(self, obs: np.ndarray) -> np.ndarray:
        """Inference: obs(57) -> append to history -> concat(285) -> ONNX -> action(16)."""
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
        """Reset history buffer (call on simulation reset)."""
        self.history.clear()
        self.last_action = STANDING_POSE.copy()
