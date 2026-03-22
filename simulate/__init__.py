"""
LingTu Simulation Platform — 仿真平台

分层架构:
  core/       引擎抽象 (SimEngine / SimWorld / SimRobot / SimSensor)
  mujoco/     MuJoCo 引擎实现
  bridge/     ROS2 桥接 (仿真 ↔ 导航栈)
  scenarios/  测试场景 (导航/语义/巡检)
  worlds/     环境模型 (MJCF/USD)

用法:
  lingtu sim                    # 默认 MuJoCo 工厂场景
  lingtu sim --engine mujoco    # 指定引擎
  lingtu sim --world factory    # 指定场景
  lingtu sim --scenario patrol  # 巡检测试
"""
