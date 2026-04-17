# 全局路径规划器选型决策

> 决策日期: 2026-04-03
> 最近更新: 2026-04-17(补充运行时链路图)
> 状态: 已实施

---

## TL;DR — 外场快速查阅

**Web 点击"发送目标"走的是什么规划器?**
→ **看当前 profile**。`lingtu.py nav`(生产默认) = **PCT** (基于 pcd 3D 地形规划)。
→ 永远**不要**默认说 A*。A* 只在 `map / sim / dev / stub / sim_nav` 里用。

快速判断命令:
```bash
ssh sunrise@192.168.66.190 "ps aux | grep 'lingtu.py' | grep -v grep"
# 输出含 "lingtu.py nav" → pct
# 输出含 "lingtu.py map" → astar (建图不规划,但即使规划也是 astar)
```

---

## 运行时链路:Web 点击 → cmd_vel

```
[Web: scene tab]
  用户点击 3D 场景地面
  → pendingGoal = {x, y}
  → 弹"发送/取消"确认面板
  用户点"发送"
  → POST /api/v1/goto  body: {x, y}

[Gateway: gateway_module.py]
  /api/v1/goto 路由
  → PlannerService.send_instruction(kind="goto", x, y)
  → SemanticPlannerModule.goal_pose Out[PoseStamped]

[NavigationModule: src/nav/navigation_module.py]
  goal_pose In[PoseStamped] 接收
  → mission FSM: IDLE → PLANNING
  → GlobalPlannerService.plan(start=robot_pos, goal=user_goal)

[GlobalPlannerService: src/nav/global_planner_service.py]
  根据 planner= 参数选后端:
    "pct"   → _PCTBackend (ele_planner.so, C++ aarch64)   ← nav profile 实际走这里
    "astar" → _AStarBackend (pure Python, 8-连通)
  输入: tomogram.pickle (从 map_cloud + elevation_map 烘的分层断层图)
  输出: path np.ndarray (N, 3) 世界坐标 [x, y, z]

[WaypointTracker: src/nav/waypoint_tracker.py]
  按距离下采样 path → waypoints
  → NavigationModule.waypoint Out[PoseStamped] 逐点发出
  → 监控到达/卡住,触发下一个 waypoint 或 stuck recovery

[PathFollower: src/nav/core/path_follower_core.hpp (Pure Pursuit)]
  waypoint In → 当前 robot_pos
  → 计算前瞻点 → Pure Pursuit 曲率 → cmd_vel

[CmdVelMux: src/nav/cmd_vel_mux_module.py]
  优先级仲裁(0.5s 超时):
    teleop 100 > visual_servo 80 > recovery 60 > path_follower 40
  → 发到 driver_cmd_vel Out

[Driver: src/drivers/thunder/thunder_driver.py]
  driver_cmd_vel → gRPC Walk → brainstem
  → 电机 torque
```

**关键事实**:
- nav profile 下规划器 = **PCT (C++ ele_planner.so)**,地图源是 tomogram(pcd → elevation → 分层断层)
- 不经过 ROS2 `/nav/goal_pose` 话题。Module-First 架构下,goal 在 Python 进程内直接 Port→Port
- C++ autonomy 包(terrain_analysis + local_planner + path_follower ROS2 node)在 `enable_native=False` 时**不参与**,由 Python autonomy chain 全程替代

---

## 决策

**S100P 生产环境使用 PCT 官方 `ele_planner.so`(`planner="pct"`)。**
**仿真/CI/开发机使用纯 Python A*(`planner="astar"`)作跨平台替代。**

---

## 规划器清单

### 1. PCT `ele_planner.so` — **生产环境，S100P 专用**

| 属性 | 值 |
|------|-----|
| 注册名 | `"pct"` |
| 实现 | C++，由 HKUST Bowen Yang / Jie Cheng 开发，GPLv2 |
| 入口 | `_PCTBackend` → `TomogramPlanner.plan()` |
| 文件 | `src/global_planning/PCT_planner/planner/lib/src/ele_planner/` |
| .so 路径 | `planner/lib/ele_planner.so`（aarch64 编译，仅 S100P 可用） |
| 地图格式 | Tomogram `.pickle`（多层 traversability + elevation + gradient） |
| 规划能力 | 3D 地形感知（楼梯/坡道）、GPMP 轨迹优化、坡度约束 |
| 输出格式 | `np.ndarray (N, 3)`，世界坐标 [x, y, z] |

**使用场景**: `nav`、`explore`、`s100p` profiles（真机导航）

---

### 2. 纯 Python A* — **替代方案，开发机/CI/仿真**

| 属性 | 值 |
|------|-----|
| 注册名 | `"astar"` |
| 实现 | 我们自己写的，`_AStarBackend`，8-连通 A* |
| 文件 | `src/global_planning/pct_adapters/src/global_planner_module.py` |
| 地图格式 | 同 Tomogram `.pickle`，只取 ground-floor slice |
| 规划能力 | 2D 平面，无轨迹优化，无坡度感知 |
| 存在理由 | `ele_planner.so` 是 aarch64 二进制，开发机/CI 无法运行 |

**使用场景**: `sim`、`dev`、`stub`、`map` profiles

---

## 为什么不用我们自己的 Python A*（`astar`）做生产

我们自己实现的 `_astar` 和 `_astar_3d`（在 `pct_planner_astar.py`）存在以下问题：

1. **`_AStarBackend.plan()` 路径重建 bug** — `(si,sj)` 起点不加入 came_from，路径丢失起点
2. **`pct_planner_astar.py` 直线兜底** — A* 失败时 `cells = [(si,sj),(gi,gj)]`，同时发 FAILED，但继续执行
3. **仅 2D** — `_AStarBackend` 只用 ground-floor slice，无法处理楼梯/坡道
4. **无轨迹优化** — 输出锯齿格栅路径，四足执行抖动大
5. **慢** — 纯 Python `dict` g_score，200×200 格子可能数十秒

**结论**: 这两套代码存在的唯一理由是跨平台兼容，不是质量。

---

## Profile 映射

| Profile | 机器人 | 规划器 | 理由 |
|---------|--------|--------|------|
| `nav` | S100P | **`pct`** | 真机导航，需要 3D 地形感知 |
| `explore` | S100P | **`pct`** | 真机探索，同上 |
| `s100p` | S100P | **`pct`** | 真机，同上 |
| `sim` | MuJoCo | `astar` | ele_planner.so 无法在 x86 运行 |
| `map` | S100P | `astar` | 建图阶段无需高质量规划 |
| `dev` | stub | `astar` | 无硬件，无 tomogram |
| `stub` | stub | `astar` | CI 测试，无硬件 |

---

## PCT 后端接口说明

```python
# global_planner_module.py 中的 _PCTBackend

backend = _PCTBackend(tomogram_path="/path/to/building.pickle", obstacle_thr=49.9)

# 规划（返回 np.ndarray (N,3) 或 [] 如果失败）
path = backend.plan(
    start=np.array([x, y, z]),
    goal=np.array([x, y, z]),
)
if not path:
    # 真实失败，触发 LERa 恢复
    ...
```

`TomogramPlanner.plan()` 内部流程：
1. 世界坐标 → grid index（`pos2idx`）
2. 高度 → slice index（`pos2slice`）
3. C++ `ele_planner` 做 3D A*（`a_star.so`）
4. C++ `GPMP` 做轨迹优化（`traj_opt.so`）
5. Grid index → 世界坐标（`transTrajGrid2Map`）

---

## 相关文件

| 文件 | 作用 |
|------|------|
| `src/global_planning/pct_adapters/src/global_planner_module.py` | `_PCTBackend` / `_AStarBackend` 注册 |
| `src/global_planning/PCT_planner/planner/scripts/planner_wrapper.py` | `TomogramPlanner` 包装 C++ .so |
| `src/global_planning/PCT_planner/planner/lib/src/ele_planner/` | C++ 规划器源码 |
| `src/nav/global_planner_service.py` | `GlobalPlannerService`，`NavigationModule` 使用 |
| `cli/profiles_data.py` | Profile 规划器配置 |
| `src/core/blueprints/full_stack.py` | Blueprint 默认 `planner_backend="astar"` |
