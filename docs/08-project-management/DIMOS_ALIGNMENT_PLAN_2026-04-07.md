# LingTu 对标 DimOS 补全规划（2026-04-07）

## 1. 已验证事实

- 2026-04-07 在 `D:\inovxio\brain\lingtu\research\semantic_map_refs\dimos` 执行了 `git pull --ff-only`，结果为 `Already up to date.`。
- 当前本地 DimOS `main` 头部提交是 `a035fb31`。
- 当前最新正式 release 仍是 `v0.0.11`，发布时间是 2026-03-12。
- `v0.0.11` 之后 `main` 上还有 4 个提交，公开可见变化以 README/docs 为主，没有出现新的 release 代际切换。

这意味着：我们现在对标的不是一个又发生大换血的上游，而是一个已经把产品表面收拢得比较完整的运行时。

## 2. LingTu 能不能变得像 DimOS

可以，但不能做成 "Agent 直接绕过安全层控制机器人" 的版本。

对 LingTu 来说，应该对标的是 DimOS 的这几类能力：

- 统一的安装和启动入口
- 统一的 runfile/blueprint 目录
- 统一的 CLI 生命周期：`run / status / log / stop / restart`
- agent-native 的 MCP/skill 接口
- replay/simulation/hardware 三种一致的运行形态
- 多硬件适配和最小 fleet 运维面

不应该照搬的部分：

- 不把 LingTu 变成顶层任务仲裁和安全执行者
- 不让 agent 直接下发电机、步态、关节级控制
- 不把 `voice / askme / arbiter / safety / control` 的产品边界抹掉

推荐目标定位：

> LingTu 从 "四足导航子系统" 升级为 "受安全边界约束的 agent-native autonomy runtime/provider"。

也就是：

- LingTu 负责感知、建图、定位、规划、语义、MCP、replay/sim/hardware 统一运行时
- NOVA Dog runtime 继续负责 mission、safety、control、operator plane
- Agent 只能向 LingTu 提交目标、查询状态、调用白名单技能
- 任何会进入执行链的动作都必须再经过 `arbiter / safety / control`

## 3. 当前差距和补全方向

### 3.1 产品层级过窄

当前状态：

- README 仍把 LingTu定义为 quadruped navigation system。
- README 也明确写了 LingTu 不是 top-level product control plane。

缺的不是单个算法，而是一层完整的 "autonomy OS surface"：

- 统一运行入口
- 标准 product boundary
- 明确 owned surface
- 明确 external contracts

补全动作：

- 把 LingTu 文案从 "navigation system" 升级为 "autonomy runtime/provider"
- 明确划分四层：
  - `LingTu Core`: module/transport/blueprint/runtime
  - `LingTu Adapters`: robot drivers, sensors, replay, sim
  - `LingTu Interfaces`: CLI, MCP, Gateway, observability
  - `NOVA Runtime Integration`: nav/sense contracts to runtime services
- 在 README 和 docs-site 中补一张系统边界图：
  - LingTu owns: localization, planning, scene truth, semantic memory, agent tool surface
  - Runtime owns: mission, safety policy, actuation, operator workflow

### 3.2 硬件抽象和 runfile 生态不够通用

当前状态：

- `ROBOT_PRESETS` 主要还是 `stub / sim / ros2 / s100p / thunder`
- driver 代码明显围绕 `Thunder`/`nova_dog` 打磨
- 缺少像 DimOS 那样成体系的 `basic / perceptive / agentic / replay / sim` runfile 目录

补全动作：

- 引入统一的 `runfile` 概念，建议目录：
  - `src/runfiles/thunder/`
  - `src/runfiles/go2/`
  - `src/runfiles/g1/`
  - `src/runfiles/drone/`
  - `src/runfiles/mock/`
- 每个 runfile 只做三件事：
  - 选 driver/sensor stack
  - 选 planner/perception/memory/agent stack
  - 导出一个可直接运行的 blueprint
- 统一命名：
  - `thunder-basic`
  - `thunder-nav`
  - `thunder-agentic-mcp`
  - `thunder-replay`
  - `g1-sim`
  - `drone-basic`
- 在 driver 层补 capability registry，而不是继续把平台差异写死在 profile 里

### 3.3 安装和默认运行路径偏重、偏脆

当前状态：

- README 仍要求 ROS 2 + colcon build
- `nav` / `explore` profile 默认 `enable_native=False`
- 注释已经说明原因是 native 依赖没有稳定打包完成

要达到 DimOS 体验，LingTu 至少需要：

- `uv` 可安装的 Python package
- 可选 extras
- 原生依赖的 preflight 检查
- daemon 化生命周期
- 不依赖人工查日志的失败诊断

补全动作：

- 保留 ROS2/native 支持，但把它们收敛为安装层，不暴露成用户主入口
- 引入：
  - `lingtu doctor`
  - `lingtu show-config`
  - `lingtu status`
  - `lingtu log -f`
  - `lingtu stop`
- 将 native 依赖收成三层：
  - required: 纯 Python / stub / replay / basic sim
  - optional-native: Fast-LIO2 / local planner / PCT native backend
  - optional-hardware: Thunder/S100P/robot-specific SDK
- 目标是让以下命令默认可跑：
  - `lingtu --simulation run thunder-nav`
  - `lingtu --replay run thunder-office`
  - `lingtu run thunder-nav --robot thunder`

### 3.4 agent-native 能力弱一层

当前状态：

- LingTu 已经有 `GatewayModule` 和 `MCPServerModule`
- 但 agent 还没有成为标准运行入口
- 更关键的是，agent 路径没有被清晰地放进安全闭环里

补全动作：

- CLI 上补出标准 agent 生命周期：
  - `lingtu run <runfile> --daemon`
  - `lingtu agent-send "<text>"`
  - `lingtu mcp list-tools`
  - `lingtu mcp call <tool>`
  - `lingtu stop`
- 增加 per-run JSONL context logging：
  - agent message
  - tool call
  - tool result
  - safety gate decision
  - action execution result
- Agent 侧不直接发布执行速度，而是调用白名单接口：
  - `navigate_to(goal)`
  - `inspect_target(target)`
  - `follow_person(id)`
  - `query_scene(...)`
- 这些接口后面统一进入 runtime contracts，再由 `arbiter / safety / control` 做最后执行裁决

### 3.5 fleet / 运维面明显不足

当前状态：

- OTA 文档已承认 >5 台机器人时需要 fleet dashboard
- 说明单机 OTA 和单机 app 还不是 fleet 产品

补全动作：

- Phase 1 先做 lightweight fleet，不做重平台：
  - device registry
  - online/offline
  - software version
  - current runfile
  - health summary
- Phase 2 再补 fleet commands：
  - `lingtu fleet ps`
  - `lingtu fleet run <runfile> --targets ...`
  - `lingtu fleet stop --targets ...`
  - `lingtu fleet deploy --artifact ...`
- Phase 3 再把 OTA 变成真正的 Update Orchestrator：
  - staged rollout
  - canary
  - rollback
  - failure attribution

### 3.6 文档和上游跟踪不稳定

当前状态：

- 代码里多处写明 `dimos-style` / `Adapted from DimOS`
- 研究文档里却还指向旧的 `dimos-robotics/dimos`
- README 里仍有 `gRPC Gateway + Flutter client` 的旧叙述
- CHANGELOG 已经写明网关已切到 FastAPI，Flutter client 已移除

补全动作：

- 立刻清理失效上游链接
- 建一个单独的 upstream tracking 文档：
  - 当前对标仓库
  - 当前对标 tag/release
  - 本地借鉴点
  - 我们选择不跟的点
- 给 docs 加 link validation 和 drift checklist
- 每次架构级变更后，必须同时更新：
  - `README.md`
  - `docs-site`
  - `docs/AGENTS.md`
  - `CHANGELOG.md`

## 4. 目标状态

如果补全完成，LingTu 应该具备下面这组用户可感知表面：

```bash
uv pip install 'lingtu[base,sim]'
lingtu --simulation run thunder-nav
lingtu --simulation run thunder-agentic-mcp --daemon
lingtu status
lingtu log -f
lingtu agent-send "inspect the transformer room"
lingtu mcp list-tools
lingtu stop
```

以及下面这组工程可维护表面：

- runfile registry
- hardware capability registry
- replay/sim/hardware 统一 contracts
- per-run logs and config tracing
- safety-gated agent toolchain
- fleet-ready health and deploy surface

## 5. 分阶段路线图

## Phase 0: 文档和基线收口（1 周）

目标：

- 清理文档漂移
- 固定对标对象
- 明确边界，不做危险重构

交付物：

- 修正失效的 DimOS 上游链接
- 一份正式的对标规划文档
- README drift issue list
- `LingTu owns / runtime owns` 边界图

验收：

- 所有 DimOS 上游链接可访问
- 团队对 LingTu 的产品定位只有一套说法

## Phase 1: 运行时表面统一（2 到 3 周）

目标：

- 先把 "像 DimOS" 的 CLI 和生命周期补出来

交付物：

- `lingtu run/status/log/stop/show-config/doctor`
- daemon mode
- per-run log directory
- run registry
- resolved config tracing

建议改动面：

- `lingtu_cli.py`
- `cli/main.py`
- `cli/run_state.py`
- `cli/runtime_extra.py`
- `docs/AGENTS.md`

验收：

- 不进 REPL 也能完整运行和停止
- 出问题时不需要手工找 PID 和端口

## Phase 2: runfile 和硬件抽象统一（3 到 5 周）

目标：

- 把 profile 驱动模式升级为 runfile 驱动模式

交付物：

- runfile registry
- `basic / nav / agentic / replay / sim` 变体
- hardware capability registry
- Thunder 之外至少再打通 1 个 humanoid 或 drone 目标

建议改动面：

- `cli/profiles_data.py`
- `src/core/blueprints/`
- 新增 `src/runfiles/`
- `src/drivers/`

验收：

- `lingtu --simulation run <runfile>`
- `lingtu --replay run <runfile>`
- `lingtu run <runfile>`

三种入口都能复用同一套 contracts

## Phase 3: agent-native 但受安全边界约束（4 到 6 周）

目标：

- 把 agent 变成标准入口
- 但不放弃现有安全架构

交付物：

- `agent-send`
- MCP tool catalog
- tool allowlist
- agent context JSONL logging
- agent -> nav/sense/arbiter contract

建议改动面：

- `src/gateway/`
- `src/semantic/planner/`
- runtime service contracts

验收：

- agent 可以下达高层任务
- agent 不能直接写 `cmd_vel`
- 每个执行动作都有安全裁决日志

## Phase 4: temporal memory 产品化（3 到 4 周）

目标：

- 把已经存在的 memory 能力从内部模块变成外部可见能力

交付物：

- temporal query API
- replay-backed memory demos
- scene + temporal summary tools
- memory-aware agent skills

建议优先复用：

- EpisodicMemory
- TemporalMemoryModule
- VectorMemoryModule
- TaggedLocations

验收：

- 可以回答 "上午 9 点谁在配电间"
- 可以回答 "上次看到目标设备是什么时候"

## Phase 5: fleet / OTA orchestration（4 到 6 周）

目标：

- 从单机工具升级到小规模 fleet 可运营

交付物：

- fleet registry
- health dashboard
- deploy orchestration
- multi-device command fanout
- OTA metrics

验收：

- 可以管理 5 到 20 台机器人
- 支持分批发布、回滚、失败归因

## 6. 优先级建议

P0 立刻做：

- 文档收口
- 旧 DimOS 链接修正
- README/CHANGELOG/AGENTS 漂移清单
- CLI 生命周期统一方案

P1 最值得投入：

- runfile registry
- daemon/status/log/stop
- native preflight 和 packaging
- safety-gated agent interface

P2 再做：

- fleet dashboard
- temporal memory 产品 API
- 多机器人 orchestration

## 7. 建议的实施原则

- 不为了 "像 DimOS" 去破坏现有安全边界
- 优先补产品表面和运行时一致性，而不是先追求更多算法堆叠
- 先让 replay/sim/hardware 三态统一，再扩更多硬件
- 每补一层功能，都要同步补 CLI、日志、文档和验收命令

## 8. 一句话结论

LingTu 完全可以向 DimOS 靠拢，但正确方向不是把它做成 "更大的导航包"，而是把它做成 "受 safety/runtime 边界约束的 autonomy OS surface"。
