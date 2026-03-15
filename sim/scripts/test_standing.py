#!/usr/bin/env python3
"""
NOVA Dog 站立姿态测试 — MuJoCo 可视化

用法:
  # 带 GUI 窗口 (需要 DISPLAY):
  DISPLAY=:0 python3 test_standing.py

  # 无头渲染, 输出图片:
  python3 test_standing.py --render /tmp/nova_standing.png
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import mujoco

SCRIPT_DIR = Path(__file__).resolve().parent
ROBOT_XML = SCRIPT_DIR.parent / "robot" / "robot.xml"

# ── 场景 XML: 写到 robot/ 目录旁, 让 meshdir 正确解析 ────────
SCENE_TEMPLATE = """\
<mujoco model="nova_standing_test">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <headlight ambient="0.4 0.4 0.4"/>
    <quality shadowsize="2048"/>
    <map znear="0.01" zfar="50"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1=".85 .85 .82" rgb2=".7 .7 .68"/>
    <material name="grid" texture="grid" texrepeat="10 10" texuniform="true"/>
  </asset>

  <include file="robot.xml"/>

  <worldbody>
    <light pos="2 -2 3" dir="-0.3 0.3 -1" diffuse="0.9 0.88 0.82" castshadow="true"/>
    <light pos="-2 2 3" dir="0.3 -0.3 -1" diffuse="0.4 0.42 0.5" castshadow="false"/>
    <geom name="floor" type="plane" size="5 5 0.1" material="grid"
          conaffinity="1" condim="3"/>
  </worldbody>
</mujoco>
"""

# ── Dart standingPose (直接用, 不取反) ────────────────────────
# actuator 顺序: arm(8) + FR(4) + FL(4) + RR(4) + RL(4) = 24
STANDING_CTRL = np.zeros(24)
STANDING_CTRL[8:12]  = [-0.1, -0.8,  1.8, 0.0]   # FR
STANDING_CTRL[12:16] = [ 0.1,  0.8, -1.8, 0.0]   # FL
STANDING_CTRL[16:20] = [ 0.1,  0.8, -1.8, 0.0]   # RR
STANDING_CTRL[20:24] = [-0.1, -0.8,  1.8, 0.0]   # RL

STANDING_QPOS = {
    'fr_hip_joint': -0.1, 'fr_thigh_joint': -0.8, 'fr_calf_joint':  1.8,
    'fl_hip_joint':  0.1, 'fl_thigh_joint':  0.8, 'fl_calf_joint': -1.8,
    'rr_hip_joint':  0.1, 'rr_thigh_joint':  0.8, 'rr_calf_joint': -1.8,
    'rl_hip_joint': -0.1, 'rl_thigh_joint': -0.8, 'rl_calf_joint':  1.8,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--render", type=str, default=None,
                        help="渲染到图片文件 (无头模式)")
    args = parser.parse_args()

    # 写场景文件到 robot/ 目录, 让 include + meshdir 都能正确解析
    scene_xml_path = ROBOT_XML.parent / "_test_scene.xml"
    scene_xml_path.write_text(SCENE_TEMPLATE)
    print(f"Loading {scene_xml_path}")
    m = mujoco.MjModel.from_xml_path(str(scene_xml_path))
    d = mujoco.MjData(m)
    print(f"  nq={m.nq} nv={m.nv} nu={m.nu} nbody={m.nbody} njnt={m.njnt}")

    # 设置初始关节角度
    for jname, val in STANDING_QPOS.items():
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
        d.qpos[m.jnt_qposadr[jid]] = val

    d.ctrl[:] = STANDING_CTRL
    mujoco.mj_forward(m, d)

    base_h = d.qpos[2]
    print(f"  Initial base height: {base_h:.3f}m")

    # 先仿真 1s 让机器人稳定
    for _ in range(500):
        mujoco.mj_step(m, d)
    print(f"  Settled base height: {d.qpos[2]:.3f}m")

    if args.render:
        # 无头渲染
        width, height = 1280, 960
        renderer = mujoco.Renderer(m, height, width)

        # 相机设置: 从前方斜 45 度俯视
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [0.0, 0.0, 0.3]
        cam.distance = 2.0
        cam.azimuth = 135
        cam.elevation = -25
        renderer.update_scene(d, camera=cam)
        pixels = renderer.render()

        # 保存 PNG
        try:
            from PIL import Image
            img = Image.fromarray(pixels)
            img.save(args.render)
            print(f"  Saved: {args.render}")
        except ImportError:
            # fallback: raw save
            out = Path(args.render)
            np.save(out.with_suffix('.npy'), pixels)
            print(f"  Saved (npy): {out.with_suffix('.npy')}")
        return

    # GUI 模式
    try:
        viewer_mod = __import__('mujoco.viewer', fromlist=['viewer'])
    except ImportError:
        print("ERROR: mujoco.viewer not available, use --render instead")
        sys.exit(1)

    print("  Opening viewer... (Ctrl+C to quit)")
    with viewer_mod.launch_passive(m, d) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.0, 0.3]
        viewer.cam.distance = 2.0
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25

        while viewer.is_running():
            d.ctrl[:] = STANDING_CTRL
            mujoco.mj_step(m, d)
            viewer.sync()


if __name__ == "__main__":
    main()
