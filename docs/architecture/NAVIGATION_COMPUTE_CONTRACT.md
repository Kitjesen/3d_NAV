# 导航计算契约 (Navigation Compute Contract) v1.0

> 权威文档。任何涉及"全局规划 / 局部规划 / 代价分析 / cmd_vel 产出"的改动，
> 必须先与本契约对齐。契约由 `src/core/tests/test_profile_graph_snapshots.py`
> 中的 `test_navigation_compute_contract_*` 锁定。

## 1. 为什么需要这份契约

系统里曾经同时存在两套"代价语义"——`TraversabilityCostModule.fused_cost`
与 `LocalPlannerModule` 的点云体素评分——但没有明确谁主谁辅，导致"到底是谁
在决定机器人局部行为"含糊不清。本契约把计算职责正式分成 **四层**，每层只有
一个主输入、一个主产出，禁止跨层抢职责。

## 2. 四层职责 (单一职责，强约束)

```
L5 全局规划   NavigationModule + GlobalPlannerService(PCT/A*)
              主输入: tomogram / saved_map (+ odometry)
              主产出: global_path (战略路径) + waypoint (阶段目标)
              语义:   "去哪里、走哪条主路"

L2 安全门控   TraversabilityCostModule -> NavigationModule.costmap
              主输入: occupancy + slope + ESDF 融合成 fused_cost (风险栅格)
              主产出: 对全局规划的"拦截 / 降级 / 触发重规划"决定
              语义:   "这条全局路当前还安全吗"；它【不产出局部轨迹】

L2 局部规划   LocalPlannerModule (nav_core / cmu)
              主输入: terrain_map (+ boundary + added_obstacles 点云)
              主产出: local_path (可执行局部轨迹) + control_hint
              语义:   "当前这一小段怎么走、怎么绕、要不要减速/停"

L2 控制跟踪   PathFollowerModule
              主输入: local_path (+ control_hint)
              主产出: cmd_vel
              语义:   "把局部轨迹跟成速度指令"
```

层间只能高层→低层流动。`waypoint/global_path` 从 L5 流向 L2 是**指令派发**，
不是依赖反转。

## 3. 数据语义统一 (强命名约束)

| 名称 | 语义 | 唯一主消费方 |
|------|------|--------------|
| `tomogram` / `saved_map` | 静态全局可通行体 (长期结构真值) | 全局规划 (L5) |
| `global_path` | 战略路径 (稀疏航点) | 局部规划入口 |
| `fused_cost` (= costmap) | **风险栅格** (occupancy+slope+ESDF 融合) | 全局安全门控 + 可视化 |
| `terrain_map` | **局部几何障碍点云** | 局部规划 (L2) |
| `local_path` | 执行轨迹 (稠密) | 控制跟踪 (L2) |
| `cmd_vel` | 速度指令 | CmdVelMux -> Driver |

**铁律**：
- `fused_cost`/`costmap` 是"全局安全/可视化"语义，**不得**作为局部规划的主评分输入。
- `terrain_map` 是"局部几何"语义，**不得**用于全局战略决策。
- 局部规划主算法是 **CMU/nav_core 点云体素评分**，不是 costmap 滚动优化 (非 DWA/TEB 范式)。

## 4. costmap 在全局层的确切用途

`NavigationModule._on_costmap` 只做两件事，二者都属于"安全门控"，不产出轨迹：

1. `GlobalPlannerService.update_map(grid)` —— 把实时风险叠加进 PCT/A* 的
   `_find_safe_goal` / `path_safety` 检查 (动态障碍过滤)。
2. 满足节流条件 (默认 3s) 时触发一次重规划 `_plan()`。

**PCT 默认不按 costmap 高频重规划** (`replan_on_costmap_update` 对 `pct` 默认
`False`)：PCT 基于 tomogram 有自己的地形代价模型，长期结构由 tomogram 负责，
costmap 只在安全门控层兜底。A* (dev/sim) 默认 `True`，因为它没有离线地形真值。

## 5. ESDF 现状 (重要：避免误解)

wiring 中存在 `TraversabilityCostModule.esdf_field -> LocalPlannerModule.esdf`，
但 **C++ `LocalPlannerCore` 当前没有 `set_esdf` 绑定**，Python 侧 `_on_esdf`
仅缓存字段、不下发给评分。

结论：**ESDF 目前是局部规划的预留扩展点，不参与局部主评分。** ESDF 当前的
真实生效路径是经 `fused_cost` 的 proximity 软代价进入"全局安全门控层"。
后续若要让 ESDF 影响局部轨迹，必须：
1. 在 `nav_core` 增加 `set_esdf` 绑定与评分项；
2. 在本契约升级版本号并补测试；
否则不得声称"局部规划用了 ESDF"。

## 6. 决策优先级 (运行时仲裁)

cmd_vel 最终由 `CmdVelMux` 按优先级仲裁，但**计算层的安全否决顺序**是：

```
1. SafetyRing / stop_signal            (硬急停，最高)
2. LocalPlanner safety_stop/near_field_stop  (局部不可行 -> 停)
3. 全局重规划 / fallback_astar          (全局路不安全 -> 换路/降级)
4. PathFollower tracking               (正常跟踪，最低)
```

## 7. 关键线缆契约 (由测试锁定)

必须存在：
- `TraversabilityCostModule.fused_cost -> NavigationModule.costmap` (门控)
- `TraversabilityCostModule.fused_cost -> GatewayModule.costmap` (可视化)
- `OccupancyGridModule.costmap -> TraversabilityCostModule.costmap`
- `ESDFModule.esdf -> TraversabilityCostModule.esdf`
- `TerrainModule.terrain_map -> LocalPlannerModule.terrain_map` (局部主输入)
- `NavigationModule.global_path -> LocalPlannerModule.global_path`
- `NavigationModule.waypoint -> LocalPlannerModule.waypoint`
- `LocalPlannerModule.local_path -> PathFollowerModule.local_path`
- `LocalPlannerModule.control_hint -> PathFollowerModule.control_hint`

必须**不**存在 (防止职责漂移)：
- `TraversabilityCostModule.fused_cost -> LocalPlannerModule.*` (costmap 不得做局部主评分)
- `TerrainModule.terrain_map -> NavigationModule.*` (局部几何不得做全局战略)

## 8. 扩展点 (受控演进)

未来若要更"ROS-style"的局部代价闭环，约定**叠加而非替换**：
保留 CMU/nav_core 局部主链，新增 costmap/ESDF 作为**约束项**，并通过显式
profile 开关启用，同时升级本契约版本。
