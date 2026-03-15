# MapPilot 测试框架

本目录包含 MapPilot (灵途) 系统的所有测试。

## 目录结构

```
tests/
├── benchmark/              # 性能基准测试
│   ├── run_all.sh
│   ├── benchmark_slam.sh
│   ├── benchmark_planner.sh
│   └── benchmark_grpc.sh
├── integration/            # 集成测试 (T1-T8 + 原有)
│   ├── run_all.sh
│   ├── test_full_stack.sh
│   ├── test_planning_stub.sh        # Stub 模式规划流水线 (22/22 PASS)
│   ├── test_full_chain.sh           # T6 全链路启动脚本
│   ├── test_safety_signals.sh       # T7 安全信号启动脚本
│   ├── test_terrain_node.py         # T1 terrain_analysis 节点通信
│   ├── test_local_planner_node.py   # T2 localPlanner 节点通信
│   ├── test_path_follower_node.py   # T3 pathFollower 节点通信
│   ├── test_global_planner_node.py  # T4 全局规划器节点通信
│   ├── test_adapter_node.py         # T5 pct_path_adapter 节点通信
│   ├── test_full_chain.py           # T6 全链路闭环 (6 节点)
│   ├── test_safety_signals.py       # T7 安全信号链路
│   ├── test_dog_bridge.py           # T8 han_dog_bridge 驱动层
│   ├── test_planning_pipeline.py    # ROS2 注入假里程计/路径/地形
│   ├── test_grpc_endpoints.py       # gRPC 端点测试
│   ├── test_topic_hz.py             # 话题频率验证
│   ├── test_network_failure.py      # 网络故障容错
│   ├── test_far_planner.py          # FAR Planner 集成测试
│   └── test_mission_arc.py          # Mission Arc 集成测试
└── planning/               # 规划算法纯 Python 单元测试
    └── test_pct_adapter_logic.py    # 20 tests: waypoint tracking, stuck, goal_reached
```

## 快速开始

```bash
# 从项目根目录
make test              # 所有 colcon 测试
make test-integration  # bash tests/integration/run_all.sh
make benchmark         # 性能基准测试
```

## ROS2 节点集成测试 (T1-T8)

在 S100P 机器人 (192.168.66.190) 上验证真实 C++ 节点间的 ROS2 话题通信。

**全部通过** (2026-03-09):

| 测试 | 节点 | 验证点 | 状态 |
|------|------|--------|------|
| **T1** terrain_analysis | terrainAnalysis | PointCloud2 → terrain_map intensity | PASS |
| **T2** localPlanner | terrainAnalysis + localPlanner | terrain_map + way_point → /path + /stop | PASS 4/4 |
| **T3** pathFollower | pathFollower | /path → /nav/cmd_vel 方向正确 | PASS |
| **T4** global_planner | pct_planner_astar | goal_pose → global_path 可行 | PASS 5/5 |
| **T5** pct_path_adapter | pct_path_adapter | global_path + odom → waypoint 推进 | PASS |
| **T6** 全链路闭环 | 6 节点同时运行 | goal_pose → cmd_vel, 18s 内 goal_reached | PASS 5/5 |
| **T7** 安全信号 | localPlanner + pathFollower | stop/slow_down 传递和响应 | PASS 4/4 |
| **T8** han_dog_bridge | han_dog_bridge | cmd_vel → gRPC Walk(), watchdog 超时归零 | PASS 8/8 |

### 运行方式

测试通过 paramiko SSH 上传到 S100P 机器人执行：

```bash
# 上传并执行单个测试
python sim/scripts/upload_and_test.py --test T1

# 或 SSH 到机器人手动运行
ssh sunrise@192.168.66.190
source /home/sunrise/data/SLAM/navigation/install/setup.bash
python3 /tmp/test_terrain_node.py
```

每个测试输出 JSON 结果：
```json
{
  "test": "T1_terrain_analysis",
  "checks": [
    {"name": "terrain_map_published", "pass": true},
    {"name": "ground_intensity_near_zero", "pass": true}
  ],
  "duration_sec": 12.3,
  "pass": true
}
```

## 性能基准测试

用于建立系统性能基线，回归测试。

### benchmark_slam.sh
测试 FAST-LIO2 / Point-LIO 的处理速度和资源占用。

**输出指标**: 处理时间、CPU 使用率、内存使用率

### benchmark_planner.sh
测试 PCT Planner 的规划速度。

**输出指标**: 平均规划时间、规划成功率

### benchmark_grpc.sh
测试 gRPC Gateway 的吞吐量和延迟。

**输出指标**: QPS、平均延迟、并发性能

## 集成测试 (原有)

### test_full_stack.sh
验证所有关键节点能否正常启动。

### test_planning_stub.sh
Stub 模式规划流水线验证 (22/22 PASS)，不需要硬件。

### test_grpc_endpoints.py
测试所有关键 gRPC 端点 (GetSystemInfo, AcquireLease, GetMode 等)。

### test_topic_hz.py
验证关键话题以预期频率发布。

### test_network_failure.py
模拟网络故障，测试系统容错能力。

## 规划算法单元测试

```bash
# 纯 Python，无需 ROS2
python tests/planning/test_pct_adapter_logic.py
```

20 个测试覆盖: 路径下采样 (3D 距离)、航点推进、卡死检测 (mock time)、goal_reached 事件。

## 测试覆盖率

| 模块 | 覆盖 | 说明 |
|------|------|------|
| semantic_planner | ~90% | 13 个测试文件 (goal_resolver, fast_slow, action_executor 等) |
| nav_core (C++) | 73 gtest | 纯 C++ 算法，零 ROS2 依赖 |
| remote_monitoring | gtest | LeaseManager, SafetyGate, EventBuffer 等 |
| Flutter 客户端 | 56 tests | Widget + 集成测试 |
| planning (Python) | ~85% | pct_adapter_logic 20 tests |
| semantic_perception | ~40% | 6 个测试文件 |
| **ROS2 节点通信** | **T1-T8** | **全链路真实节点验证，全部 PASS** |

## 编写新测试

### ROS2 节点集成测试模板

```python
#!/usr/bin/env python3
"""T* — 节点通信验证"""
import json, time
import rclpy
from rclpy.node import Node

class TestHarness(Node):
    def __init__(self):
        super().__init__('test_harness')
        # 发布合成数据
        # 订阅输出话题
        # 验证结果

    def run_checks(self):
        results = []
        # ... 验证逻辑 ...
        return {"test": "T*_name", "checks": results, "pass": all(r["pass"] for r in results)}

def main():
    rclpy.init()
    harness = TestHarness()
    # spin + verify
    result = harness.run_checks()
    print(json.dumps(result, indent=2))
    rclpy.shutdown()
    exit(0 if result["pass"] else 1)

if __name__ == '__main__':
    main()
```

## 参考资料

- [ROS 2 Testing Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
- [gRPC Testing Best Practices](https://grpc.io/docs/guides/testing/)

---

**最后更新**: 2026-03-12
