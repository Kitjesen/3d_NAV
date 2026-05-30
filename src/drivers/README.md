# LingTu Drivers — 硬件抽象与可替换后端

> 本目录是「机器人本体 + 传感器」的接入层。上层 (nav / semantic / gateway)
> 只通过 `core.registry` 解析驱动，并依赖 `spec.py` 定义的端口契约，不直接 import 具体后端。
> sim 与 real 的差异**只应存在于本层**——这是 dimos 式「同一接口、可替换后端」的落地。

## sim / real / mock 边界（一眼看懂）

| 类别 | 位置 | 角色 | registry 名 |
|------|------|------|-------------|
| **real 机器人** | `real/thunder/` | ThunderDriver（gRPC → brainstem），真实四足 | `thunder` |
| **sim 后端** | `sim/mujoco_driver_module.py` | MuJoCo 全栈仿真（带相机/雷达/里程计） | `sim_mujoco` |
| **sim 后端** | `sim/ros2_sim_driver.py` | ROS2/Gazebo 仿真桥 | `sim_ros2` |
| **mock** | `core/blueprints/stub.py`（`StubDogModule`） | 无硬件，dead-reckoning，CI/测试 | `stub` |
| **real 传感器** | `real/lidar/lidar_module.py` | Livox MID-360 python 驱动 | — |
| **sim 传感器** | `sim/sim_pointcloud_provider.py` | 仿真点云源 | — |
| **遥操作** | `teleop_module.py` | WebSocket joystick → cmd_vel | — |

## 接口契约（`spec.py`）

所有 driver 后端必须满足 **MotionDriver** 契约，可选实现传感器能力：

```
MotionDriver   (必备): cmd_vel: In[Twist], stop_signal: In[int]  →  odometry: Out[Odometry]
CameraSource   (可选): (camera_image|color_image), depth_image, camera_info : Out
PointcloudSource(可选): map_cloud : Out[PointCloud2]
```

- `sim_mujoco` / `sim_ros2` = MotionDriver + CameraSource + PointcloudSource（全传感器源）
- `stub` / `thunder` = 仅 MotionDriver（传感器由独立的 camera-bridge / lidar 模块提供）
- 契约由 `src/core/tests/test_driver_spec.py` 机器校验，保证 sim 与 real **端口同形状**。

判断某后端能力：`from drivers.spec import driver_capabilities; driver_capabilities(Cls)`。

## ROS2 colcon 包（为何在这、为何不进对称子目录）

以下是 **ROS2 colcon 包**（含 `package.xml` + `CMakeLists.txt`），由 `colcon build`
管理，必须留在 workspace 可发现路径，**不纳入 sim/real python 后端的对称结构**：

| 包 | 位置 | 说明 |
|----|------|------|
| Livox ROS2 驱动 | `livox_ros_driver2/` | MID-360 的 ROS2 原生驱动 + Livox-SDK2 |
| GNSS ROS2 reader | `gnss/wtrtk980_ros2_reader/` | WTRTK980 RTK 读取节点 |

这些是「ROS2 工程产物」，与本层的「Python 可替换后端」是两类东西——刻意分开，
不强行塞进 `real/` 子目录（那样会破坏 colcon 发现并制造伪对称）。

## 注册与解析

```python
# 后端通过装饰器自注册
@register("driver", "sim_mujoco")
class MujocoDriverModule(Module, layer=1): ...

# blueprint / stack 通过 registry 解析，不直接 import 具体后端
from core.registry import get
DriverCls = get("driver", robot)          # robot ∈ {thunder, sim_mujoco, sim_ros2, stub}
```

切换 sim/real 只改 `robot` 选择（profile 的 `_default_robot`），上层栈零改动。
详见 `docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md` 与
`docs/superpowers/plans/2026-05-30-repo-structure-redesign.md`。
