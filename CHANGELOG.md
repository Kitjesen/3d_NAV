# Changelog

All notable changes to MapPilot (灵途) navigation system.

Format: [Semantic Versioning](https://semver.org/) — `MAJOR.MINOR.PATCH`

---

## [Unreleased] — 2026-03-01

### 第三批迭代 R21-R30 (进行中)

---

## [1.5.x-dev] — 2026-02-28 (R11-R20)

### Added
- **[R14] pathFollower: GOAL_REACHED 信号** — 到达目标时向 `/nav/planner_status` 发布 `"GOAL_REACHED"`，带防重发 flag，路径切换时自动复位
- **[R15] han_dog_bridge: 里程计发布降频** — 新增 `odom_pub_rate` 参数（默认 10Hz），避免 50Hz IMU 频率淹没 SLAM 里程计
- **[R12] han_dog_bridge: 协方差矩阵** — Odometry 消息补全 36 元素 pose/twist 协方差（对角阵，诊断工具可读）
- **[R11] localPlanner: Joy 轴参数化** — `joy_axis_fwd/left/autonomy/obstacle` 4 轴均可在 robot_config/launch 覆盖，适配不同手柄型号
- **[R13] robot_config: 集中化参数** — `autonomy.joy_to_speed_delay` / `joy_to_check_obstacle_delay` / `pct_planner.smooth_min_dist` 写入 robot_config.yaml single source of truth

### Changed
- **[R16]** costmap 验证：确认 `/nav/costmap` 是 semantic_perception 内部接口，topic_contract 注释已同步
- **[R17] pct_planner_astar: 障碍物起终点修正** — A* 前自动搜索最近可行格（起点 radius=3，终点 radius=5），避免规划失败
- **[R18] health_check.sh: 节点存活检查** — 第 7 节添加 4 个必须节点验证 + dog/generic driver 自动识别

### Fixed
- **[R20]** 修复第一批迭代（R1-R10）遗漏提交的 7 个文件

---

## [1.5.x-dev] — 2026-02-27 (R1-R10)

### Added
- **[R3] pct_planner_astar: 路径降采样** — 新增 `_downsample_path()` 等距降采样函数 + `smooth_min_dist` ROS2 参数，减少航点密度、缓解跟踪抖动
- **[R6] driver_node: 串口实现** — `connect_hardware()` 使用 `serial.Serial`，`send_to_motors()` 发送 `vx,vy,wz\n` 协议，仿真模式 fallback
- **[R14 precursor] pathFollower: Joy 轴参数化** — `joy_axis_fwd/left/yaw/autonomy/obstacle` 5 轴均可配置
- **[R8] navigation_run: planner_profile 暴露** — launch 参数说明更新，包含 `pct_py` 选项

### Fixed
- **[R1] semantic_planner cmd_vel 类型修复** — `Twist` → `TwistStamped`，remap 补充 `("cmd_vel", "/nav/cmd_vel")`；修复孤立话题问题
- **[R2] han_dog_bridge: 位置航位推算** — 接入 IMU yaw + cmd_vel 积分位置 (x, y)，Odometry 消息携带有效位置字段
- **[R5] adapter_status 消费者迁移** — 发现 3 处隐藏的 `/nav/planner_status` JSON 消费者（task_manager.cpp / pct_path_adapter.py / planner_node.py），全部迁移到 `/nav/adapter_status`

### Infrastructure
- **[R7] health_check.sh** — 添加 `/nav/dog_odometry` / `/nav/adapter_status` 频率检查
- **[R9] ROBOT_DEPLOYMENT.md** — 更新节点链图 + 故障排查表
- **[R10] CLAUDE.md** — 话题合约表新增 dog_odometry / adapter_status / GOAL_REACHED

---

## [1.5.x-dev] — 2026-02-26 (话题审计 + 双发布者修复)

### Fixed
- **han_dog_bridge 里程计冲突** — `/Odometry` remap 从 `/nav/odometry` 改为 `/nav/dog_odometry`，Fast-LIO2 成为唯一 `/nav/odometry` 发布者
- **pct_path_adapter 状态话题冲突** — `/nav/planner_status` 双发布者修复：pct_path_adapter JSON 状态迁移到 `/nav/adapter_status`；global_planner 纯字符串保留原话题

### Added
- `topic_contract.yaml`: 新增 `dog_odometry` / `adapter_status` / `costmap` 条目，澄清各话题唯一发布者

---

## [1.5.0] — 2026-02-25 (nav_core Core/Shell 分离)

### Changed
- **nav_core 包** — 提取 3 个 ROS2 节点的纯算法到 header-only 库 (`path_follower_core.hpp` / `local_planner_core.hpp` / `pct_adapter_core.hpp`)
- **pathFollower / localPlanner / pct_path_adapter** — 成为 Thin Shell，核心逻辑改为调用 nav_core
- **gtest** — 53 个 gtest 覆盖核心算法，Windows MSVC + Linux gcc 9.4 通过
- **pybind11** — `py_nav_core.cpp` 绑定，Python 测试可调用真实 C++ 算法

---

## [1.4.x] — 2026-02-24 (E2E 建筑点云导航验证)

### Verified
- building2_9.pcd 真实点云 3 场景全部到达（13.3s / 13.1s / 11.9s，path RMS < 0.05m）
- 修复 localPlanner near-field E-stop 导致 safetyStop_ 永久卡死问题
- 输出论文图 (`tools/e2e_building_traj.png`, `tools/e2e_building_nav.gif`)

---

## [1.3.x] — 2026-02-22 (Python A* PCT 规划器)

### Added
- `pct_planner_astar.py` — 纯 Python 8-连通 A* 全局规划器，零 `.so` 依赖，适合 x86_64 开发机
- `planner_pct_py.launch.py` — 对应 launch profile (`planner_profile:=pct_py`)
- `ROBOT_DEPLOYMENT.md` — 机器人部署指南（初版）
