# LingTu Module-First 准则

> Module 就是 Node。不需要两层。

## 核心原则

### 1. Module 是唯一运行单元

```
正确:  Module(算法 + In/Out 端口) → Blueprint 编排 → main_nav.py 启动
错误:  Module(算法) + ROS2 Node(适配器) + launch 文件 → 三层维护
```

每个功能只有一个实现，住在 Module 里。
不存在 "xxx.py (ROS2 Node)" + "xxx_module.py (Module)" 这种成对文件。

### 2. Blueprint 是可复用的系统组合，任何脚本都能启动

LingTu 是库，不是应用。Blueprint 就是入口。

```python
# 一行启动 (生产)
from drivers.thunder.blueprints import thunder_semantic
thunder_semantic(dog_host="192.168.66.190", llm="kimi").build().start()

# 一行启动 (测试)
from drivers.sim.stub import stub_blueprint
stub_blueprint().build().start()

# 自由组合
from core import autoconnect
autoconnect(
    ThunderDriver.blueprint(dog_host="192.168.66.190"),
    DetectorModule.blueprint(detector="bpu"),
    NavigationModule.blueprint(planner="astar"),
    SafetyRingModule.blueprint(),
).build().start()
```

`main_nav.py` 是一个带 CLI 参数的便利脚本，不是架构约束。
不使用 `ros2 launch`。不使用 `ros2 run`。

### 3. 通信走 In/Out + Transport，不走 ROS2 topic

```
Module A ──Out[Odometry]──wire──In[Odometry]──→ Module B

transport 选择:
  callback  — 同进程直调，零延迟 (默认)
  dds       — 跨进程解耦 (cyclonedds, 不是 rclpy)
  shm       — 大数据高带宽 (点云、图像)
```

不使用 rclpy.Publisher / rclpy.Subscription。
不使用 ROS2 QoS profile（我们的 backpressure policy 替代）。

### 4. C++ 节点走 NativeModule，不走 ros2 run

```python
# NativeModule 管理 C++ 子进程
class AutonomyModule(Module, layer=2):
    def setup(self):
        self._terrain = NativeModule(NativeModuleConfig(
            executable="terrain_analysis_node",
            parameters={"voxel_size": 0.1},
        ))
        self._terrain.start()  # watchdog + auto-restart
```

C++ 节点仍然是独立可执行文件，但由 Python Module 启动和管理。
不使用 launch 文件编排 C++ 节点。

### 5. 传感器直连，不绕 ROS2 driver

```
正确:  Module 直接调 SDK → publish Out[Image]
        ThunderDriver → gRPC → brainstem (已实现)
        CameraModule → cv2.VideoCapture / SDK → Out[Image]

错误:  ros2 run livox_driver → /livox/lidar topic → rclpy 订阅
```

如果传感器有 Python SDK，Module 直接调。
如果传感器只有 C++ driver，用 NativeModule 管子进程 + SHM/DDS 读数据。

### 6. 消息类型只有一套: core.msgs

```python
from core.msgs.nav import Odometry, Path
from core.msgs.geometry import Pose, Twist, PoseStamped
from core.msgs.semantic import SceneGraph, SafetyState
from core.msgs.sensor import Image, CameraIntrinsics
```

不 import `sensor_msgs.msg`、`nav_msgs.msg`、`std_msgs.msg`。
ROS2 消息类型只在 NativeModule 的 C++ 子进程内部使用。

### 7. 配置走构造函数参数，不走 ROS2 parameter

```python
# 正确: Blueprint 参数
bp.add(DetectorModule, detector="bpu", confidence=0.5)

# 正确: CLI 参数
python main_nav.py --detector bpu

# 错误: ROS2 declare_parameter / get_parameter
self.declare_parameter("detector", "yoloe")
```

### 8. 可插拔走 Registry，不走 if/else

```python
# 注册 (在实现文件里)
@register("detector", "bpu", platforms={"aarch64"})
class BPUDetector: ...

@register("detector", "yoloe")
class YOLOeDetector: ...

# 查找 (在 Module 里)
DetectorCls = get("detector", args.detector)
```

不在 Module 里写 `if name == "bpu": from ... import BPU`。

---

## 文件命名规范

```
src/nav/
  navigation_module.py      # Module (唯一实现)
  safety_ring_module.py     # Module (唯一实现)

src/semantic/perception/semantic_perception/
  detector_module.py        # Module
  encoder_module.py         # Module
  service.py                # Service (纯算法编排，Module 调用)
  instance_tracker.py       # 纯算法 (被 Service 调用)

src/drivers/thunder/
  han_dog_module.py         # Module (直连 brainstem gRPC)
```

**不再保留的文件模式:**
- `xxx_node.py` — ROS2 Node 入口，由 Module 替代
- `xxx.launch.py` — launch 文件，由 main_nav.py 替代
- `xxx_mixin.py` — ROS2 mixin，逻辑应在 Module 或 Service 里

### 保留 ROS2 Node 的唯一例外

SLAM (Fast-LIO2 / Point-LIO) — 这是 C++ 且深度耦合 ROS2 TF/PCL。
通过 NativeModule 启动，不需要 Python 适配器。

---

## 迁移判断流程

```
一个 ROS2 Node 文件:
  │
  ├─ 已有对应 *_module.py?
  │   ├─ YES → Module 是完整实现?
  │   │   ├─ YES → 删除 ROS2 Node 文件
  │   │   └─ NO  → 补全 Module，再删 Node
  │   └─ NO  → 新建 Module，迁移算法逻辑，删 Node
  │
  ├─ 是 C++ 可执行文件?
  │   └─ YES → NativeModule 管理，不需要 Python Node
  │
  └─ 是传感器驱动?
      ├─ 有 Python SDK → Module 直连
      └─ 只有 C++ → NativeModule + SHM/DDS
```

---

## 当前待清理文件

| 文件 | 状态 | 行动 |
|------|------|------|
| `nav/rings/nav_rings/safety_monitor.py` | 已有 SafetyRingModule | 删除 |
| `nav/rings/nav_rings/evaluator.py` | 已有 SafetyRingModule | 删除 |
| `nav/rings/nav_rings/dialogue_manager.py` | 已有 SafetyRingModule | 删除 |
| `nav/services/nav_services/map_manager.py` | 已有 MapManagerModule | 删除 |
| `nav/services/nav_services/patrol_manager.py` | 已有 PatrolManagerModule | 删除 |
| `nav/services/nav_services/geofence_manager.py` | 已有 GeofenceManagerModule | 删除 |
| `nav/services/nav_services/task_scheduler.py` | 已有 TaskSchedulerModule | 删除 |
| `nav/services/nav_services/mission_logger.py` | 已有 memory Module | 删除 |
| `semantic/.../perception_node.py` | 已有 DetectorModule + Service | 删除 |
| `semantic/.../planner_node.py` | 已有 SemanticPlannerModule | 删除 |
| `semantic/.../agent_node.py` | 被 MCPServerModule 替代 | 删除 |
| `semantic/reconstruction/reconstruction_node.py` | 已有 ReconstructionModule | 删除 |
| `drivers/thunder/driver_node.py` | 已有 ThunderDriver | 删除 |
| `memory/logging/mission_logger.py` | ROS2 Node 版本 | 新建 Module 后删除 |
| `launch/subsystems/*.launch.py` | 全部由 main_nav.py 替代 | 归档到 launch/legacy/ |
| `scripts/services/*.sh` | ros2 run 脚本 | 归档到 scripts/legacy/ |

---

## 验证标准

清理完成后:

1. `grep -r "import rclpy" src/` — 只出现在 C++ 包的 Python binding 和 legacy/ 目录
2. `grep -r "class.*Node" src/` — 只出现在 legacy/ 或 NativeModule 启动的 C++ 节点
3. `python main_nav.py --robot stub` — 全栈启动，零 ROS2 依赖
4. `python main_nav.py --robot thunder` — S100P 真机启动，SLAM 走 NativeModule
5. 644+ tests pass (不依赖 rclpy)
