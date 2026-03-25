"""
LingTu Simulation Platform

Layered architecture:
  core/       Engine abstractions (SimEngine / SimWorld / SimRobot / SimSensor)
  mujoco/     MuJoCo engine implementation
  bridge/     ROS2 bridge (simulation <-> navigation stack)
  scenarios/  Test scenarios (navigation / semantic / inspection)
  worlds/     Environment models (MJCF)

Usage:
  lingtu sim                    # default MuJoCo factory scene
  lingtu sim --engine mujoco    # specify engine
  lingtu sim --world factory    # specify world
  lingtu sim --scenario patrol  # inspection test
"""
