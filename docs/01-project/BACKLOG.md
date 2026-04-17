# LingTu Backlog — 单一真源看板

> 所有特性、回归、技术债的唯一登记表。每个 PR / 每次实机验证都必须引用这里的条目编号(`[BACKLOG-##]`)。
>
> **状态枚举**:`todo` · `doing` · `verified` · `blocked`
>
> **最近真机**:YYYY-MM-DD 或 `—`(从未)
>
> 纪律:
> 1. 新 PR 先在这里加/改条目,再开 PR
> 2. 实机验证结束后更新状态 + 日期
> 3. 状态跃迁 → `doing` 要有负责人;→ `verified` 要有对应 `vault/实机记录/` 链接

---

## P0 必须正确(客户交付阻塞线)

| # | 条目 | 验收脚本 | 单测 | 状态 | 最近真机 | 负责 |
|---|------|---------|------|------|---------|------|
| P0-01 | S100P 冷启动 | `docs/07-testing/p0_cold_boot.sh` | 1522 tests | `todo` | — | — |
| P0-02 | 建图 → 保存 → 激活 → 定位 | `docs/07-testing/p0_mapping.sh` | `test_map_occupancy.py` | `todo` | — | — |
| P0-03 | 点击目标 → 到达 | `docs/07-testing/p0_goto.sh` | `test_new_modules.py::Navigation` | `todo` | — | — |
| P0-04 | 紧急停止反射 | `docs/07-testing/p0_estop.sh` | `test_new_modules.py::SafetyRing` | `todo` | — | — |
| P0-05 | Wave 1 硬护栏生产路径 | 嵌入 P0-01 | 17 tests | `verified` (单测) / `todo` (真机) | — | — |
| P0-06 | scipy / _nav_core.so S100P 就绪 | P0-01 首次拉起即验 | — | `todo` | — | — |

---

## P1 应该正确(差异化卖点)

| # | 条目 | 验收脚本 | 单测 | 状态 | 最近真机 | 负责 |
|---|------|---------|------|------|---------|------|
| P1-01 | TARE 自主探索 | `docs/07-testing/p1_tare_explore.sh` | `test_exploration_supervisor.py` | `todo` | — | — |
| P1-02 | GNSS RTK 融合(WTRTK-980) | `docs/07-testing/p1_gnss_fusion.sh` | GNSS 集成测试已有 | `todo` | — | — |
| P1-03 | NTRIP 差分 RTCM 注入 | 嵌入 P1-02 | `ntrip_client_module` | `todo` | — | — |
| P1-04 | OSNet Re-ID 跟人(BPU) | `docs/07-testing/p1_follow_person.sh` | 14 tests | `todo` | — | — |
| P1-05 | 安全退化(传感器拔线) | `docs/07-testing/p1_degraded.sh` | `test_degeneracy_fusion.py` | `todo` | — | — |
| P1-06 | SLAM 漂移守卫 + 自动 relocalize | 嵌入 P0-02 | `test_drift_guard.py` | `todo` | — | — |
| P1-07 | Worker 子进程隔离(重 ML 模块) | 嵌入 P0-01 (不阻塞 Gateway) | `test_worker_isolation.py` | `todo` | — | — |
| P1-08 | Wave 2 12 项精度升级 | 各自单测 | 39 tests | `verified` (单测) / `todo` (真机) | — | — |
| P1-09 | Wave 3 算法(OSNet/PD/TSDF/Bayesian/config) | 各自单测 | 28 tests | `verified` (单测) / `todo` (真机) | — | — |

---

## P2 能用即可(长尾功能)

| # | 条目 | 验收 | 状态 | 备注 |
|---|------|------|------|------|
| P2-01 | LLM 多轮 AgentLoop(Kimi) | 手工点 chat 跑一轮 | `doing` | 需要 `MOONSHOT_API_KEY` |
| P2-02 | 语义记忆查询(VectorMemory) | MCP `query_memory` 能返回 tag | `todo` | W3-1 要求 sentence-transformers 或 hard fail |
| P2-03 | 重定位服务 | REPL `relocalize` 命令 | `todo` | — |
| P2-04 | LLM 后端切换 (Kimi / OpenAI / Claude / Qwen) | 改 `robot_config.yaml` llm.backend | `todo` | 切换后端未实机验证 |
| P2-05 | 局部路径 Web 渲染(amber) | Dashboard 场景页 | `verified` | 2026-04-15 commit `14b14b2` |
| P2-06 | 智能体对话动画(thinking / agent_message) | 手工 chat | `verified` | 2026-04-16 commit `17718a4` |

---

## P3 技术债(不阻塞客户)

| # | 条目 | 预估 | 状态 | 备注 |
|---|------|------|------|------|
| P3-01 | ROS2 Phase 1:CameraBridge 去 rclpy | 1 天 | `todo` | Windows dev 不能跑相机 |
| P3-02 | ROS2 Phase 1:ROS2SimDriver 去 rclpy | 1 天 | `todo` | 同上 |
| P3-03 | ROS2 Phase 2:Fast-LIO2 nanobind 库化 | 1-2 周 | `todo` | 最大价值改造,但先放 |
| P3-04 | ROS2 Phase 2:Point-LIO 同上 | 1 周 | `todo` | — |
| P3-05 | OTA 后端 agent → `/api/v1/ota/*` | 2 天 | `todo` | 前端 Modal 已 ready |
| P3-06 | Reconstruction TSDF 真跑(Open3D GPU) | 2 天 | `todo` | open3d S100P 安装状态未知 |
| P3-07 | 语义评分权重学习(sgnav/fast_path/frontier) | 2 周 | `todo` | 需收标注数据 |
| P3-08 | Askme 语音交互接入 | 1 周 | `todo` | 独立产品,作为 voice agent 集成 |
| P3-09 | 14 项 Wave 1/2/3 剩余简化 | — | `todo` | 🟢 / 需算法研究,见 Plan 历史 |
| P3-10 | Dashboard 导出诊断包后端 | 0.5 天 | `doing` | 本周 Day 3 要做 |

---

## 最近交付(commit 参考,映射到上表)

| Commit | 内容 | BACKLOG 条目 |
|--------|------|--------------|
| `9f7171c` | Dashboard UI 统一 + 设置菜单 + 场景气泡 | P2-05 / 新 UI |
| `13cbd35` | TARE Supervisor + Gateway SSE | P1-01 |
| `b903ee9` | Wave recovery: MapManager 层析 + tests | P0-02 / P0-05 |
| `c440b8c` | Wave 3: OSNet/PD/TSDF/Bayesian/config | P1-04 / P1-09 |
| `d1dbc78` | Wave 2 C/D: perception + motion | P1-08 |
| `14d3bee` | Wave 2 A/B: config + bbox/vlm/agent | P1-08 |
| `a26bf24` | Wave 1 硬护栏 | P0-05 |
| `b3c93e8` | TARE C++ 代码搬入 | P1-01 |
| `439fac2` | TARE 集成 | P1-01 |
| `0a44163` | GNSS Dashboard + NTRIP | P1-02 / P1-03 |
| `66a9696` | GNSS 运行期控制面 + MCP skill | P1-02 |
| `612ac25` | GNSS 天线杆臂补偿 | P1-02 |
| `c0d47d0` | GNSS fusion_health 诊断 | P1-02 |
| `643ee96` / `b4919a8` | SLAM 漂移守卫 | P1-06 |
| `dbc7507` | SLAM ENU 长期锚定 | P1-06 |
| `c09fd11` | Worker 子进程隔离 | P1-07 |
| `336413a` | GIL 背压(latest policy) | P1-07 |
| `0d816b5` | DeviceRegistry 框架 | P0-01 |
| `525d1ee` | UI 调色板收敛 | P2-05 |
| `17718a4` | agent_message SSE | P2-06 |

---

## 更新纪律

1. **开 PR 前**:在此文件加/改条目 → 本次 commit 信息 `[BACKLOG-XX]`
2. **单测通过** → 状态更新 `doing → verified (单测)`
3. **实机通过** → 状态 `verified (真机)` + 最近真机日期 + `vault/实机记录/YYYY-MM-DD.md` 链接
4. **Blocker** 出现 → `blocked` + 写原因(如"等 NTRIP 源账号"、"等 S100P 网络")

**审核频率**:每周五 L3 回归后刷新一次状态。
