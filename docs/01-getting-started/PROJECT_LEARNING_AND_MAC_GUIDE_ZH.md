# LingTu 项目学习与 Mac 开发指南

> 适合第一次阅读项目、准备后续改进时使用。本文按当前仓库代码与 README/CHANGELOG 重新梳理；仓库里部分历史文档仍保留旧版本、旧路径或旧产品名，遇到冲突时优先看 `README.md`、`CHANGELOG.md`、`cli/profiles_data.py`、`src/core/blueprints/full_stack.py` 和 `web/README.md`。

## 1. 项目一句话

LingTu/MapPilot 是一套四足机器人自主导航系统。它把机器狗底盘、LiDAR SLAM、地图管理、地形感知、路径规划、语义导航、远程 Web 控制台和 AI Agent 接口组装成一个可运行的完整系统。

目标运行环境是 S100P/RDK X5 或类似 Ubuntu 22.04 + ROS2 Humble 的机器人计算板；Mac 更适合做前端、Python 业务逻辑、配置、文档、测试和部分仿真开发。

## 2. 系统主要功能

### 2.1 建图

入口 profile: `python lingtu.py map`

主要功能：

- 启动 LiDAR/SLAM 管线，采集环境点云。
- 通过地图管理模块保存地图。
- 保存结果通常包括 `map.pcd`、`tomogram.pickle`、`occupancy.npz`。
- `tomogram.pickle` 给全局规划器使用，`occupancy.npz` 给 A* 和 Dashboard 预览使用。

相关代码：

- `cli/profiles_data.py`: `map` profile 配置。
- `src/nav/services/nav_services/map_manager_module.py`: 地图保存、列表、激活、重命名、删除、构建 tomogram/occupancy。
- `src/gateway/gateway_module.py`: Web API 暴露地图操作。
- `web/src/components/MapView.tsx`: 前端地图管理界面。

### 2.2 定位与导航

入口 profile: `python lingtu.py nav`

主要功能：

- 使用已保存地图进行定位。
- 接收坐标目标或语义目标。
- 全局规划生成路径，局部规划跟随航点。
- 任务状态机管理 `IDLE / PLANNING / EXECUTING / SUCCESS / FAILED / STUCK / CANCELLED / PATROLLING` 等状态。
- 卡住后会尝试恢复动作和重新规划。

相关代码：

- `src/nav/navigation_module.py`: 导航任务状态机、全局规划调用、航点推进、卡死恢复。
- `src/nav/global_planner_service.py`: A* / PCT 规划后端封装。
- `src/nav/waypoint_tracker.py`: 航点到达、路径完成、卡死检测。
- `src/base_autonomy/modules/`: 地形、局部规划、路径跟随的模块封装。
- `src/nav/cmd_vel_mux_module.py`: 多来源速度指令优先级仲裁。

### 2.3 自主探索

入口 profile: `python lingtu.py explore`

主要功能：

- 在没有预建地图或目标未知时，使用 frontier 探索策略生成探索目标。
- 支持 wavefront frontier；配置中也预留了 TARE 后端。
- 语义规划找不到已知目标时，也会转向前沿探索。

相关代码：

- `src/nav/frontier_explorer_module.py`
- `src/core/blueprints/stacks/exploration.py`
- `src/semantic/planner/semantic_planner/frontier_scorer.py`
- `config/robot_config.yaml` 的 `exploration` 段。

### 2.4 语义导航

入口 profile: `python lingtu.py dev`、`python lingtu.py nav`、`python lingtu.py explore`

主要功能：

- 用户输入“去餐桌”“找上次放背包的地方”等自然语言指令。
- 感知模块从 RGB-D 图像中检测物体、投影到 3D，并维护场景图。
- 语义规划模块先用 Fast Path 在场景图/记忆中快速匹配目标。
- 快速匹配失败后，查询向量记忆、选择 frontier 探索点，最后可退化到视觉伺服。
- LLM 后端支持 Kimi/Qwen/OpenAI/Claude/mock 等配置思路；开发时默认可用 mock。

相关代码：

- `src/semantic/perception/semantic_perception/perception_module.py`: 语义感知模块外壳。
- `src/semantic/perception/semantic_perception/yoloe_detector.py`、`yolo_world_detector.py`、`bpu_detector.py`: 检测后端。
- `src/semantic/perception/semantic_perception/projection.py`: 2D 检测转 3D 坐标。
- `src/semantic/perception/semantic_perception/instance_tracker.py`: 物体跟踪和场景图构建。
- `src/semantic/planner/semantic_planner/semantic_planner_module.py`: 指令处理、目标解析、frontier、恢复逻辑统一入口。
- `src/semantic/planner/semantic_planner/goal_resolver.py`: Fast-Slow 目标解析。
- `src/semantic/planner/semantic_planner/task_decomposer.py`: 任务分解。
- `src/semantic/planner/semantic_planner/llm_client.py`: LLM 客户端。

### 2.5 Web Dashboard

入口：

- 机器人端由 `GatewayModule` 在 `:5050` 提供。
- 前端源码在 `web/`，本地开发可用 Vite。

主要功能：

- 控制台页：相机直播、定位/状态卡片、MiniMap、智能体聊天。
- 场景页：类似 RViz 的 2D/3D 地图视图，图层控制，点击发送目标。
- 地图页：保存、激活、重命名、删除地图，查看点云。
- SLAM 页：建图/定位/停止等模式切换。
- 通过 REST、SSE、WebSocket 连接后端。

相关代码：

- `web/src/App.tsx`: 前端应用入口、Tab 路由、登录/控制台切换。
- `web/src/services/api.ts`: 前端 API 封装。
- `web/src/hooks/useSSE.ts`: 实时事件流。
- `web/src/components/`: Dashboard 组件。
- `src/gateway/gateway_module.py`: REST/SSE/WebSocket 后端。

### 2.6 远程控制与安全

主要功能：

- WebSocket 摇杆遥控。
- 紧急停止。
- 多路速度命令优先级仲裁：人工遥控 > 视觉伺服 > 恢复动作 > 自主路径跟随。
- 安全环监控 odometry、cmd_vel、定位状态和路径执行质量。

相关代码：

- `src/drivers/teleop_module.py`: 远程遥控和相机帧。
- `src/nav/cmd_vel_mux_module.py`: 速度来源仲裁。
- `src/nav/safety_ring_module.py`: 安全反射、执行评估、对话状态。
- `src/drivers/thunder/han_dog_module.py`: 真机 gRPC 底盘驱动。

### 2.7 MCP / AI Agent 接口

主要功能：

- 向 Claude Code 或其他支持 MCP 的 Agent 暴露机器人能力。
- 模块中用 `@skill` 标注的方法会被自动发现为 MCP tool。
- 典型能力包括健康检查、模块列表、导航、查询场景图、停止等。

相关代码：

- `src/gateway/mcp_server.py`: MCP JSON-RPC 服务。
- `src/core/module.py`: `@skill`、技能 schema 自动生成。
- `src/core/blueprints/full_stack.py`: MCP 与 Gateway/Navigation/SemanticPlanner 的接线。

### 2.8 仿真

入口：

- `python lingtu.py sim`
- `python lingtu.py sim_nav`
- `python sim/scripts/...`

主要功能：

- MuJoCo 物理环境。
- LiDAR/RGB-D 仿真。
- 人物跟随、语义搜索、室内/工厂/开放场景。
- 可走 ROS2 桥，也可走纯 Python LingTu 模块栈。

相关代码：

- `sim/README.md`
- `sim/engine/`
- `sim/worlds/`
- `sim/scripts/`
- `src/drivers/sim/`

## 3. 运行入口和 profile

最重要的入口是：

```bash
python lingtu.py
python lingtu.py <profile>
```

profile 定义在 `cli/profiles_data.py`：

| Profile | 用途 | 是否适合 Mac 初学 |
| --- | --- | --- |
| `stub` | 无硬件、框架测试、启用 Gateway | 适合 |
| `dev` | 无机器人，启用语义管线和 mock LLM | 适合，但感知模型依赖可能需要额外安装 |
| `sim_nav` | 纯 Python 导航仿真，不依赖 ROS2/C++ | 适合 |
| `sim` | MuJoCo 仿真，启用更多完整栈 | 条件适合，需要 MuJoCo 等依赖 |
| `map` | 真机建图 | 不适合直接在 Mac 跑 |
| `nav` | 真机定位导航 | 不适合直接在 Mac 跑 |
| `explore` | 真机自主探索 | 不适合直接在 Mac 跑 |
| `tare_explore` | TARE 探索后端 | 不适合直接在 Mac 跑 |

常用生命周期命令：

```bash
python lingtu.py show-config dev --json
python lingtu.py status
python lingtu.py log -f
python lingtu.py stop
python lingtu.py doctor
```

## 4. 代码结构速读

```text
.
├── lingtu.py                  # 主入口，调用 cli/main.py
├── cli/                       # profile、REPL、daemon、状态、日志
├── src/
│   ├── core/                  # Module/Blueprint/Transport/Registry 框架
│   ├── nav/                   # 导航 FSM、规划服务、安全、地图服务
│   ├── semantic/              # 语义感知、语义规划、任务分解、LLM
│   ├── memory/                # 语义记忆、拓扑记忆、向量/时间序列存储
│   ├── drivers/               # 真机、相机、LiDAR、仿真、遥控
│   ├── gateway/               # FastAPI 网关、MCP、Dashboard 辅助页
│   ├── base_autonomy/         # C++ 地形/局部规划/path follower 封装
│   ├── global_planning/       # PCT planner、tomogram、A* 相关
│   └── slam/                  # Fast-LIO2、Point-LIO、localizer、GNSS 融合
├── web/                       # React + TypeScript + Vite Dashboard
├── sim/                       # MuJoCo 仿真与测试场景
├── config/                    # 机器人、话题、语义、DDS、规划配置
├── calibration/               # 相机/IMU/LiDAR 标定工具箱
├── launch/                    # ROS2 launch，偏传统/兼容路径
├── tests/                     # 集成、规划、实验测试
├── tools/                     # Dashboard、可视化、转换、诊断工具
└── docs/                      # 文档
```

## 5. 核心架构怎么串起来

LingTu 的新架构围绕 `Module` 和 `Blueprint`：

- 每个功能模块继承 `core.module.Module`。
- 输入输出通过类型标注声明成 `In[T]` / `Out[T]`。
- `Blueprint` 负责实例化模块、显式接线、自动匹配同名同类型端口。
- `full_stack_blueprint()` 根据 profile 参数把 driver、slam、maps、perception、memory、planner、navigation、safety、gateway 组装成完整系统。

关键文件：

- `src/core/module.py`: 模块基类、生命周期、端口扫描、`@skill`。
- `src/core/blueprint.py`: 模块注册、接线、build、auto_wire。
- `src/core/blueprints/full_stack.py`: 完整系统的实际装配逻辑。
- `src/core/blueprints/stacks/`: 各子系统的工厂函数。

简化数据流：

```text
用户 / Web / MCP / REPL
        |
        v
GatewayModule / MCPServerModule / LingTuREPL
        |
        +--> instruction ------------------> SemanticPlannerModule
        |                                      |
        |                                      v
        |                                goal_pose
        |                                      |
        +--> goal_pose ----------------------> NavigationModule
                                               |
                                               v
                                        global_path / waypoint
                                               |
                                               v
                          LocalPlannerModule -> PathFollowerModule
                                               |
                                               v
                            CmdVelMux -> ThunderDriver / SimDriver
```

感知和地图数据流：

```text
Camera / LiDAR / Odometry
        |
        +--> PerceptionModule -> scene_graph -> SemanticPlanner / Gateway / MCP
        |
        +--> SLAM / map_cloud -> OccupancyGrid / ElevationMap / Terrain / Gateway
        |
        +--> NavigationModule / SafetyRing / Memory
```

## 6. 学习顺序建议

第一步：先看运行入口和 profile

- `README.md`
- `lingtu.py`
- `cli/main.py`
- `cli/profiles_data.py`
- `cli/repl.py`

理解目标：知道 `python lingtu.py dev` 最终会组装哪些模块。

第二步：看模块框架

- `src/core/module.py`
- `src/core/blueprint.py`
- `src/core/blueprints/full_stack.py`

理解目标：知道一个模块如何声明端口，模块之间如何连线。

第三步：看导航主链路

- `src/nav/navigation_module.py`
- `src/nav/global_planner_service.py`
- `src/nav/waypoint_tracker.py`
- `src/nav/cmd_vel_mux_module.py`
- `src/nav/safety_ring_module.py`

理解目标：知道目标点如何变成路径、航点和速度命令。

第四步：看 Web 和 Gateway

- `web/README.md`
- `web/src/App.tsx`
- `web/src/services/api.ts`
- `web/src/hooks/useSSE.ts`
- `src/gateway/gateway_module.py`

理解目标：知道前端按钮/页面最终调用哪个后端接口。

第五步：看语义链路

- `src/semantic/perception/semantic_perception/perception_module.py`
- `src/semantic/planner/semantic_planner/semantic_planner_module.py`
- `src/semantic/planner/semantic_planner/goal_resolver.py`
- `src/memory/`

理解目标：知道自然语言目标如何解析成 `goal_pose`。

第六步：最后看真机和 ROS2

- `src/drivers/thunder/han_dog_module.py`
- `src/drivers/livox_ros_driver2/`
- `src/slam/`
- `src/base_autonomy/`
- `launch/`
- `docs/04-deployment/`

理解目标：知道哪些代码只能在机器人或 Linux/ROS2 环境验证。

## 7. 适合优先改进的方向

### 7.1 前端体验

推荐从这里开始。前端相对独立，Mac 上最容易跑通。

可改内容：

- 页面布局、主题、响应式、组件交互。
- 地图页的筛选、空状态、加载状态、错误提示。
- SLAM/Session 状态展示。
- ChatPanel 的命令补全、历史记录、快捷指令。
- SceneView/MiniMap 的可视化层、目标点击交互。
- 登录页和 Dashboard 的可访问性。

主要路径：

- `web/src/components/`
- `web/src/App.tsx`
- `web/src/App.css`
- `web/src/index.css`
- `web/src/services/api.ts`
- `web/src/types/index.ts`

### 7.2 Gateway API 与前后端契约

可改内容：

- 增加 REST API。
- 调整 `/api/v1/state`、`/api/v1/events` 的字段。
- 给前端补充更稳定的错误码和状态说明。
- 增加 mock API 或离线开发模式。
- 整理地图、session、bag、health 等接口的返回结构。

主要路径：

- `src/gateway/gateway_module.py`
- `web/src/services/api.ts`
- `web/src/types/index.ts`

注意：如果接口涉及真实 ROS2 服务、地图保存、SLAM 切换，需要在机器人或 Linux/ROS2 环境做最终验证。

### 7.3 CLI 和开发者体验

可改内容：

- profile 说明、参数命名、错误提示。
- `show-config` 输出。
- REPL 命令体验。
- 本地 dev/stub/sim_nav 启动流程。
- doctor/health 的提示。

主要路径：

- `cli/main.py`
- `cli/profiles_data.py`
- `cli/repl.py`
- `cli/ui.py`
- `cli/runtime_extra.py`

### 7.4 纯 Python 导航逻辑

可改内容：

- 任务状态机边界条件。
- 卡死检测和恢复策略。
- A* 规划 fallback。
- WaypointTracker 阈值和测试。
- CmdVelMux 优先级、超时和显示信息。
- SafetyRing 状态评估和对话状态。

主要路径：

- `src/nav/navigation_module.py`
- `src/nav/global_planner_service.py`
- `src/nav/waypoint_tracker.py`
- `src/nav/cmd_vel_mux_module.py`
- `src/nav/safety_ring_module.py`
- `src/core/tests/test_nav_modules.py`
- `src/core/tests/test_cmd_vel_mux.py`
- `src/core/tests/test_planner_safety.py`

### 7.5 语义规划和记忆

可改内容：

- 中文指令解析。
- Fast Path 匹配规则和阈值。
- TaskDecomposer 规则。
- Frontier 评分策略。
- LERa 恢复策略。
- Tagged locations、VectorMemory、EpisodicMemory 的查询体验。

主要路径：

- `src/semantic/planner/semantic_planner/chinese_tokenizer.py`
- `src/semantic/planner/semantic_planner/task_decomposer.py`
- `src/semantic/planner/semantic_planner/goal_resolver.py`
- `src/semantic/planner/semantic_planner/frontier_scorer.py`
- `src/semantic/planner/semantic_planner/semantic_planner_module.py`
- `src/memory/`

### 7.6 文档、配置和测试

可改内容：

- 统一版本号和旧文档路径。
- 给 profile、API、前端组件补示例。
- 增加 Mac 本地开发说明。
- 增加无需硬件的测试。
- 给关键配置加注释和校验。

主要路径：

- `docs/`
- `config/`
- `pyproject.toml`
- `requirements.txt`
- `web/README.md`
- `tests/`
- `src/core/tests/`

## 8. Mac 上可以改什么

### 8.1 强烈适合在 Mac 上改

| 内容 | 路径 | 原因 | 本地验证方式 |
| --- | --- | --- | --- |
| Web 前端 | `web/` | React/Vite 跨平台，不依赖 ROS2 | `npm run dev`、`npm run build`、`npm run lint` |
| 前端 API 类型 | `web/src/services/api.ts`、`web/src/types/` | 纯 TypeScript | TypeScript build |
| UI 组件 | `web/src/components/` | 独立度高 | Vite 预览 + 浏览器 |
| CLI/profile | `cli/` | 纯 Python | `python lingtu.py show-config dev --json` |
| Core Module 框架 | `src/core/` | 大部分测试不依赖硬件 | `python -m pytest src/core/tests -q` |
| Python 导航逻辑 | `src/nav/*.py` | 大部分可用 mock/unit test | 对应 pytest |
| 语义规划规则 | `src/semantic/planner/` | 可用 mock LLM 和离线数据测试 | planner tests |
| memory/storage | `src/memory/` | 多数为 Python/SQLite/数据结构 | memory tests |
| 文档 | `docs/`、`README.md` | 无平台限制 | Markdown review |
| 配置样例 | `config/*.yaml` | 可编辑，但真机前要谨慎 | `tests/scripts/validate_config.py` |

### 8.2 有条件适合在 Mac 上改

| 内容 | 路径 | 条件 |
| --- | --- | --- |
| MuJoCo 仿真 | `sim/` | 安装 MuJoCo 和 Python 依赖；不走 ROS2 的脚本更适合 Mac |
| 纯 Python `sim_nav` | `cli/profiles_data.py`、`src/drivers/sim/` | 依赖安装齐后可试；硬件链路仍要远端验证 |
| C++ `nav_core` | `src/nav/core/` | 理论上不依赖 ROS2，但 Mac 需要 CMake/编译器/nanobind 环境 |
| Gateway 普通 REST | `src/gateway/gateway_module.py` | 不调用 ROS2/systemd 的接口可以本地测；SLAM/地图保存等要远端验证 |
| 感知算法外壳 | `src/semantic/perception/` | 可改结构和测试；真实模型/BPU/相机需要 Linux/机器人验证 |
| Docker 配置 | `docker-compose*.yml` | 可编辑；但设备透传和 host network 行为主要面向 Linux |

### 8.3 不建议只在 Mac 上改完就认为完成

这些内容可以在 Mac 上阅读和编辑，但必须在 Ubuntu/ROS2/机器人环境验证：

- `src/slam/`: Fast-LIO2、Point-LIO、localizer、GNSS 融合。
- `src/drivers/livox_ros_driver2/`: Livox 驱动、SDK、ROS2 topic。
- `src/drivers/thunder/han_dog_module.py`: 真机 brainstem gRPC 控制。
- `src/drivers/thunder/camera_bridge_module.py`: Orbbec 相机、QoS、相机 watchdog。
- `src/base_autonomy/`: C++ terrain/local planner/path follower 真机链路。
- `launch/`: ROS2 launch 文件。
- `calibration/`: 相机、LiDAR、IMU 标定流程。
- `scripts/deploy/`、`docs/04-deployment/`: systemd、OTA、生产部署。
- BPU 检测、`.hbm` 模型、S100P 性能优化。

## 9. Mac 本地开发建议命令

前端：

```bash
cd web
npm install
npm run dev
npm run build
npm run lint
```

Python 基础环境：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
pip install -r requirements.txt
```

无需硬件的检查：

```bash
python lingtu.py show-config stub --json
python lingtu.py show-config dev --json
python -m pytest src/core/tests -q
python -m pytest tests/test_goal_resolver.py tests/test_chinese_tokenizer.py -q
```

尝试本地启动无硬件 profile：

```bash
python lingtu.py stub
python lingtu.py dev --llm mock
python lingtu.py sim_nav
```

注意：如果本地没有完整依赖，启动可能会因为模型、FastAPI、uvicorn、MuJoCo 或系统服务缺失失败。优先用 `show-config`、前端 build、单元测试建立理解，再逐步补依赖。

## 10. 建议的第一批改进任务

如果目标是在 Mac 上快速上手并产生可见改进，建议按这个顺序：

1. 给 Web Dashboard 增加 mock 数据模式，让前端不连接机器人也能完整预览。
2. 整理 `web/src/types/index.ts`，让 SSE/API 字段和 `GatewayModule` 返回结构对齐。
3. 给 `MapView`、`SceneView`、`SlamPanel` 增加更明确的 loading/error/empty 状态。
4. 给 `cli/profiles_data.py` 的 profile 写一张更准确的用户说明表。
5. 统一版本信息：当前 `README.md`、`CHANGELOG.md`、`VERSION` 和部分 docs 存在版本不一致。
6. 给 `NavigationModule`、`CmdVelMux`、`SafetyRingModule` 的边界场景补单元测试。
7. 给语义指令解析加更多中文测试样例。

## 11. 判断一个改动是否需要真机验证

可以用这个规则：

- 只改显示、布局、文案、前端状态：Mac 验证基本够。
- 改 REST/SSE 字段：Mac 可初验，但要用机器人或 stub profile 验一次前后端。
- 改 NavigationModule/SafetyRing/CmdVelMux：必须跑对应 Python 单测；涉及速度输出时要做仿真或真机安全验证。
- 改 SLAM、LiDAR、相机、底盘、systemd、OTA：必须在目标 Linux/ROS2/机器人环境验证。
- 改配置阈值：能本地校验格式，但真实效果必须上机器人或至少仿真。

## 12. 最短理解路径

如果只有半天时间：

1. 读 `README.md` 的功能和 Quick Start。
2. 读 `cli/profiles_data.py` 看有哪些运行模式。
3. 读 `src/core/blueprints/full_stack.py` 看系统怎么组装。
4. 读 `src/nav/navigation_module.py` 看目标如何变成导航任务。
5. 读 `web/README.md` 和 `web/src/App.tsx` 看 Dashboard 怎么组织。
6. 最后回到本文第 8 节，选一个能在 Mac 上闭环验证的改动开始。
