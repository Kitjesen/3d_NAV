# 代码解耦重构方案

**日期：** 2026-02-15 | **状态：** 待实施

---

## 📊 快速概览

### 分析范围
- **Driver 模块**（2 个文件）- 评分 7/10 → 9/10
- **Semantic Perception 模块**（9 个文件）- 评分 7.5/10 → 9.5/10
- **Semantic Planner 模块**（9 个文件）- 评分 6/10 → 9/10

**平均评分：** 6.8/10 → 9.2/10（提升 35%）

### 预期收益
- 测试覆盖率：< 30% → > 80% (+170%)
- 可测试性：30-40% → 90-95% (+150%)
- 可复用性：30-50% → 80-90% (+100%)
- 新人上手时间：2-3 周 → 3-5 天 (-70%)

---

## 🎯 核心问题

1. **ROS 节点职责过重** - 单个文件 300-400 行，混合了 ROS 接口、业务逻辑、数据转换
2. **配置管理分散** - 30+ 个参数声明分散在代码中
3. **缺少抽象层** - 硬件接口未抽象，缺少管道/状态机抽象
4. **难以测试** - 必须启动完整 ROS 环境，无法 Mock 外部依赖

---

## 🔧 解决方案：三层架构

```
第 3 层：ROS2 适配器
  ↓ (依赖注入)
第 2 层：业务逻辑层（不依赖 ROS，可独立测试）
  ↓ (组合)
第 1 层：基础组件层（可复用）
```

### 五大原则
1. **配置集中化** - 使用 @dataclass 管理配置
2. **业务逻辑独立** - 不依赖 ROS
3. **依赖注入** - 通过构造函数注入
4. **接口抽象** - 使用抽象基类
5. **单一职责** - 每个类只做一件事

---

## 📋 重构方案

### 1. Driver 模块

**问题：** 硬件接口未抽象，里程计计算耦合，gRPC 客户端耦合

**方案：**
```python
# 创建硬件接口抽象
class HardwareInterface(ABC):
    @abstractmethod
    def send_velocity(self, vx, vy, wz): pass

# 具体实现
class SerialHardware(HardwareInterface): ...
class GrpcHardware(HardwareInterface): ...

# ROS 节点（依赖注入）
class GenericRobotDriver(Node):
    def __init__(self, hardware: HardwareInterface):
        self.hardware = hardware
```

**文件结构：**
- `src/drivers/interfaces/hardware_interface.py` (新建)
- `src/drivers/implementations/serial_hardware.py` (新建)
- `src/drivers/implementations/grpc_hardware.py` (新建)
- `src/drivers/ros_adapters/driver_node.py` (重构)

---

### 2. Semantic Perception 模块

**问题：** perception_node.py 职责过多，配置分散，缺少管道抽象

**方案：**
```python
# 1. 配置类
@dataclass
class PerceptionConfig:
    detector: DetectorConfig
    clip: CLIPConfig
    tracker: TrackerConfig

# 2. 业务逻辑（不依赖 ROS）
class PerceptionPipeline:
    def process_frame(self, rgb, depth, intrinsics, tf):
        detections_2d = self.detector.detect(rgb, text_prompt)
        detections_3d = self._project_to_3d(detections_2d, depth, ...)
        self.tracker.update(detections_3d)
        return detections_3d

# 3. ROS 适配器
class SemanticPerceptionNode(Node):
    def __init__(self):
        self.pipeline = PerceptionPipeline(config)

    def _rgbd_callback(self, color, depth):
        detections = self.pipeline.process_frame(...)
        self._publish(detections)
```

**文件结构：**
- `src/semantic_perception/config/perception_config.py` (新建)
- `src/semantic_perception/pipeline/perception_pipeline.py` (新建)
- `src/semantic_perception/ros_adapters/perception_node.py` (重构)

---

### 3. Semantic Planner 模块

**问题：** planner_node.py 职责过多，状态机与 ROS 耦合，异步逻辑混乱

**方案：**
```python
# 1. 状态机（不依赖 ROS）
class PlannerStateMachine:
    def start_task(self, instruction):
        self.state = PlannerState.DECOMPOSING
        asyncio.create_task(self._decompose())

    def update_robot_position(self, pos):
        if self.state == NAVIGATING:
            self._check_arrival()

# 2. ROS 适配器
class SemanticPlannerNode(Node):
    def __init__(self):
        self.state_machine = PlannerStateMachine(...)
        self.state_machine.on_goal_published = self._publish_goal

    def _instruction_callback(self, msg):
        self.state_machine.start_task(msg.data)
```

**文件结构：**
- `src/semantic_planner/config/planner_config.py` (新建)
- `src/semantic_planner/state_machine/planner_state_machine.py` (新建)
- `src/semantic_planner/ros_adapters/planner_node.py` (重构)

---

## 📅 实施计划（10-15 天）

### 阶段 1：准备工作（1-2 天）
- [ ] 创建目录结构
- [ ] 定义接口和配置类
- [ ] 创建 refactor 分支

### 阶段 2：模块重构（6-9 天）
- [ ] Driver 模块（2-3 天）
- [ ] Perception 模块（2-3 天）
- [ ] Planner 模块（2-3 天）

### 阶段 3：测试与验证（2-3 天）
- [ ] 单元测试（目标覆盖率 80%）
- [ ] 集成测试
- [ ] 性能测试

### 阶段 4：文档与部署（1-2 天）
- [ ] 更新文档
- [ ] 部署上线

---

## ✅ 下一步行动

1. **召开团队会议** - 讨论重构方案，做出决策
2. **分配人员** - 确定各模块负责人
3. **创建分支** - `git checkout -b refactor`
4. **开始重构** - 从 Driver 模块开始（最简单）

---

## 📞 联系方式

**项目负责人：** [待填写]
**技术负责人：** [待填写]

---

**最后更新：** 2026-02-15
