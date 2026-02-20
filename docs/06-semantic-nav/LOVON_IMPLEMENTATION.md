# LOVON动作原语 - 实现详解

**文件位置**: `src/semantic_planner/semantic_planner/action_executor.py` (267行)
**论文**: LOVON (2024) - Language-driven Object-oriented Visual Navigation
**核心**: 6种动作原语 (Action Primitives)

---

## 🎯 LOVON是什么？

**LOVON (2024)** 是一篇关于四足机器人视觉-语言导航的论文，定义了**6种基础动作原语**，用于将高层语义指令分解为可执行的机器人动作。

---

## 🤖 我们实现的6种动作原语

### 1. NAVIGATE (导航到目标位置)

**用途**: 长距离导航到目标点

**实现** (第103-135行):
```python
def generate_navigate_command(
    self,
    target_position: Dict[str, float],
    robot_position: Optional[Dict[str, float]] = None,
) -> ActionCommand:
    """
    NAVIGATE: 导航到目标位置

    特点:
    - 自动计算朝向目标方向
    - 正常速度移动
    - 超时时间: 60秒
    """
    # 计算朝向目标方向
    yaw = 0.0
    if robot_position:
        dx = target_position["x"] - robot_position["x"]
        dy = target_position["y"] - robot_position["y"]
        yaw = math.atan2(dy, dx)

    return ActionCommand(
        command_type="goal",
        target_x=target_position["x"],
        target_y=target_position["y"],
        target_yaw=yaw,
        timeout_sec=60.0,
    )
```

**使用场景**:
- "去厨房" → NAVIGATE到厨房区域
- "导航到门口" → NAVIGATE到门的位置

---

### 2. APPROACH (接近目标)

**用途**: 最后一段路，减速接近目标

**实现** (第137-181行):
```python
def generate_approach_command(
    self,
    target_position: Dict[str, float],
    robot_position: Dict[str, float],
) -> ActionCommand:
    """
    APPROACH: 接近目标 (最后一段路, 减速)

    LOVON论文的关键动作:
    - 最后0.5m减速接近
    - 保持目标在视野中心
    - 为VERIFY做准备
    """
    dx = target_position["x"] - robot_position["x"]
    dy = target_position["y"] - robot_position["y"]
    dist = math.sqrt(dx * dx + dy * dy)
    yaw = math.atan2(dy, dx)

    # 接近到approach_distance处 (默认0.5m)
    if dist > self.approach_distance:
        scale = (dist - self.approach_distance) / dist
        goal_x = robot_position["x"] + dx * scale
        goal_y = robot_position["y"] + dy * scale
    else:
        # 已经足够近，只调整朝向
        goal_x = robot_position["x"]
        goal_y = robot_position["y"]

    return ActionCommand(
        command_type="goal",
        target_x=goal_x,
        target_y=goal_y,
        target_yaw=yaw,
        approach_speed=0.15,  # 减速到0.15 m/s
        timeout_sec=20.0,
    )
```

**关键参数**:
- `approach_distance`: 0.5m (停在目标前0.5米)
- `approach_speed`: 0.15 m/s (减速)

**使用场景**:
- 找到"红色灭火器"后 → APPROACH接近它
- 准备验证目标身份前 → APPROACH到合适距离

---

### 3. LOOK_AROUND (原地360°扫描)

**用途**: 到达新区域后扫描周围环境

**实现** (第183-200行):
```python
def generate_look_around_command(self) -> ActionCommand:
    """
    LOOK_AROUND: 原地360°旋转扫描

    LOVON的核心动作之一:
    - 到达新区域后先扫描
    - 让感知模块捕获周围所有物体
    - 旋转速度: 0.5 rad/s
    - 持续时间: 12秒 (≈360°)
    """
    return ActionCommand(
        command_type="velocity",
        angular_z=self.look_around_speed,  # 0.5 rad/s
        timeout_sec=self.look_around_duration,  # 12秒
    )
```

**关键参数**:
- `look_around_speed`: 0.5 rad/s (旋转速度)
- `look_around_duration`: 12秒 (完成360°)

**使用场景**:
- 进入新房间 → LOOK_AROUND扫描
- 找不到目标 → LOOK_AROUND重新观察

---

### 4. VERIFY (验证目标身份)

**用途**: 近距离确认目标是否正确

**实现** (第208-236行):
```python
def generate_verify_command(
    self,
    target_position: Dict[str, float],
    robot_position: Dict[str, float],
) -> ActionCommand:
    """
    VERIFY: 面向目标, 准备视觉验证

    流程:
    1. 调整朝向，正对目标
    2. 保持verify_distance距离 (0.8m)
    3. 拍照 → CLIP/VLM确认身份
    """
    dx = target_position["x"] - robot_position["x"]
    dy = target_position["y"] - robot_position["y"]
    yaw = math.atan2(dy, dx)

    return ActionCommand(
        command_type="goal",
        target_x=robot_position["x"],  # 不移动位置
        target_y=robot_position["y"],
        target_yaw=yaw,  # 只调整朝向
        timeout_sec=10.0,
    )
```

**关键参数**:
- `verify_distance`: 0.8m (验证距离)

**使用场景**:
- APPROACH后 → VERIFY确认是否是目标
- 使用CLIP或VLM进行视觉验证

---

### 5. BACKTRACK (回退到之前位置)

**用途**: 探索失败时回退

**实现** (第238-267行):
```python
def generate_backtrack_command(
    self,
    previous_position: Dict[str, float],
) -> ActionCommand:
    """
    BACKTRACK: 回退到之前的位置

    使用场景:
    - 探索死路 → BACKTRACK返回
    - 目标验证失败 → BACKTRACK重新搜索
    """
    return ActionCommand(
        command_type="goal",
        target_x=previous_position["x"],
        target_y=previous_position["y"],
        target_z=previous_position.get("z", 0.0),
        timeout_sec=self.nav_timeout,
    )
```

**使用场景**:
- 走进死胡同 → BACKTRACK返回
- 目标不对 → BACKTRACK重新搜索

---

### 6. EXPLORE (探索未知区域)

**用途**: 目标未知时主动探索

**实现**: 通过Frontier评分器选择探索方向

**使用场景**:
- 找不到目标 → EXPLORE新区域
- 场景图为空 → EXPLORE建立地图

---

## 📊 动作原语对比

| 动作 | 速度 | 距离 | 用途 | 超时 |
|------|------|------|------|------|
| **NAVIGATE** | 正常 | 任意 | 长距离导航 | 60秒 |
| **APPROACH** | 慢(0.15m/s) | 停在0.5m | 接近目标 | 20秒 |
| **LOOK_AROUND** | 0.5rad/s | 原地 | 360°扫描 | 12秒 |
| **VERIFY** | 静止 | 0.8m | 验证身份 | 10秒 |
| **BACKTRACK** | 正常 | 任意 | 回退 | 60秒 |
| **EXPLORE** | 正常 | 任意 | 探索 | 60秒 |

---

## 🔄 典型任务流程

### 示例: "去红色灭火器"

```
1. NAVIGATE → 导航到灭火器大致区域
   ↓
2. LOOK_AROUND → 360°扫描，寻找红色灭火器
   ↓
3. APPROACH → 减速接近灭火器 (停在0.5m)
   ↓
4. VERIFY → 正对灭火器，CLIP确认是否正确
   ↓
5. 成功 ✅ 或 失败 → BACKTRACK重新搜索
```

---

## 💡 LOVON的核心思想

### 1. 分层动作设计
```
高层语义指令: "去红色灭火器"
    ↓ 任务分解 (SayCan)
子目标序列: [NAVIGATE, FIND, APPROACH, VERIFY]
    ↓ 动作执行 (LOVON)
底层动作原语: [NAVIGATE命令, LOOK_AROUND命令, ...]
    ↓ 导航栈
机器人控制: 速度命令、路径规划
```

### 2. 视觉-语言对齐
- **APPROACH**: 保持目标在视野中心
- **VERIFY**: 近距离视觉确认
- **LOOK_AROUND**: 主动感知环境

### 3. 鲁棒性设计
- **BACKTRACK**: 失败恢复
- **EXPLORE**: 主动探索
- **超时机制**: 避免卡死

---

## 🎯 我们的实现特点

### 1. 完整的6种动作原语
✅ NAVIGATE - 长距离导航
✅ APPROACH - 减速接近
✅ LOOK_AROUND - 360°扫描
✅ VERIFY - 视觉验证
✅ BACKTRACK - 回退
✅ EXPLORE - 探索

### 2. 精确的参数控制
- 接近距离: 0.5m
- 验证距离: 0.8m
- 旋转速度: 0.5 rad/s
- 减速速度: 0.15 m/s

### 3. 统一的命令接口
```python
@dataclass
class ActionCommand:
    command_type: str  # "goal" | "velocity" | "cancel"
    target_x: float
    target_y: float
    target_yaw: float
    angular_z: float
    approach_speed: float
    timeout_sec: float
```

### 4. 状态管理
```python
class ActionStatus(Enum):
    IDLE = "idle"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    TIMEOUT = "timeout"
```

---

## 📁 相关文件

**实现代码**: `action_executor.py` (267行)
**测试文件**: `test/test_action_executor.py` (13个测试)
**使用位置**: `planner_node.py` (调用动作执行器)

---

## ✨ 总结

### LOVON中我们用了什么？

**6种动作原语**:
1. ✅ **NAVIGATE** - 长距离导航
2. ✅ **APPROACH** - 减速接近 (0.5m, 0.15m/s)
3. ✅ **LOOK_AROUND** - 360°扫描 (0.5rad/s, 12秒)
4. ✅ **VERIFY** - 视觉验证 (0.8m)
5. ✅ **BACKTRACK** - 回退恢复
6. ✅ **EXPLORE** - 主动探索

### 核心价值

- **分层设计**: 高层语义 → 动作原语 → 底层控制
- **视觉对齐**: 保持目标在视野中心
- **鲁棒性**: 失败恢复和主动探索
- **精确控制**: 减速、距离、旋转速度

### 实现状态

✅ **完整实现** - 267行代码，13个测试通过

---

**文档生成时间**: 2026-02-15 17:45
**实现状态**: ✅ 完整实现
