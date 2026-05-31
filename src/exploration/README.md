# Exploration — 自主探索模块

## 概述

Exploration 模块为机器人提供在未知环境中自主探索的能力，基于 CMU TARE 层级式探索框架，实现无先验地图情况下的全覆盖探索。

| 模块 | 职责 |
|------|------|
| `tare_explorer_module.py` | TARE 层级式探索主模块，管理探索行为与决策 |
| `exploration_supervisor_module.py` | 探索监管器，监控探索进度、边界检测与安全 |
| `tare_ros2_bridge_module.py` | TARE 与 ROS2 通信桥接 |

## tare_planner/

TARE 规划器 C++ 实现（层级 BFS 探索规划），包含地图分区与边界决策算法。通过 `native_factories.py` 注册到框架。

## 依赖

- CMU TARE 层级探索算法
- Fast-LIO2 SLAM 提供实时里程计与点云
- 通过 `exploration(backend="tare")` 栈工厂组装
