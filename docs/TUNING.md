# MapPilot 调参速查表

> 修改参数后重启对应节点即可生效，无需重新编译。

---

## 1. 语义探索 — `config/semantic_exploration.yaml`

### Frontier 评分权重（四项之和 ≈ 1.0）

| 参数 | 默认值 | 范围 | 影响 |
|------|--------|------|------|
| `frontier_distance_weight` | 0.25 | 0.1–0.4 | 偏好近处 frontier（越高→越不愿远探） |
| `frontier_novelty_weight` | 0.35 | 0.2–0.5 | 偏好未探索区域（越高→探索范围更广） |
| `frontier_language_weight` | 0.20 | 0.1–0.3 | 语言指令相关性（越高→更贴近目标描述） |
| `frontier_grounding_weight` | 0.20 | 0.1–0.3 | 场景图空间线索（越高→更依赖已知对象位置） |

**典型调参场景**：
- 机器人原地打转 → 降低 `frontier_distance_weight`，提高 `frontier_novelty_weight`
- 目标附近但找不到 → 提高 `frontier_grounding_weight` 和 `frontier_language_weight`
- 探索效率低 → 提高 `frontier_novelty_distance`（当前 5.0 m）

### 其他探索参数

| 参数 | 默认值 | 影响 |
|------|--------|------|
| `frontier_score_threshold` | 0.15 | 最低接受分，降低→接受更多候选 frontier |
| `frontier_min_size` | 5 cells | 过小的 frontier 被过滤，降低→探索细缝隙 |
| `frontier_max_count` | 10 | 评分前保留的候选数，增大→评分开销增加 |
| `confidence_threshold` | 0.5 | 低于此置信度触发 Frontier 探索 |
| `arrival_radius` | 1.0 m | 到达判定半径 |

---

## 2. 语义规划 — `config/semantic_planner.yaml`

### LLM 后端

| 参数 | 默认值 | 影响 |
|------|--------|------|
| `backend` | `kimi` | 切换 LLM：`kimi` / `openai` / `claude` / `qwen` / `mock` |
| `model` | `kimi-k2.5` | 对应后端的模型名 |
| `timeout_sec` (主) | 15.0 s | Kimi 响应超时，超时→触发备用或报错 |
| `timeout_sec` (备) | 10.0 s | 备用 LLM（qwen-turbo）超时 |
| `confidence_threshold` | 0.6 | 低于此置信度触发探索而非直接导航 |

**无网络/离线运行**：
```bash
export MOONSHOT_API_KEY=""
# 修改 config/semantic_planner.yaml: backend: "mock"
ros2 launch launch/navigation_explore.launch.py target:="找到充电桩"
```

### Frontier 评分（语义规划模式）

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `frontier_distance_weight` | 0.2 | 0.1–0.4 | 同上，此处用于语义规划模式 |
| `frontier_novelty_weight` | 0.3 | 0.2–0.5 | |
| `frontier_language_weight` | 0.2 | 0.1–0.3 | |
| `frontier_grounding_weight` | 0.3 | 0.1–0.4 | |
| `frontier_grounding_keyword_bonus` | 0.4 | 0.2–0.6 | 指令关键词命中加分，最影响语义导航精度 |
| `frontier_cooccurrence_bonus` | 0.25 | 0.1–0.4 | 共现先验加分（目标常与某物体同现时）|

---

## 3. 机器人参数 — `config/robot_config.yaml`

| 参数 | 默认值 | 范围 | 影响 |
|------|--------|------|------|
| `stuck_timeout` | 10.0 s | 5–20 s | 卡死检测灵敏度。降低→更快恢复，但可能误触 |
| `max_linear_vel` | — | — | 最大线速度（查文件确认当前值）|
| `max_angular_vel` | — | — | 最大角速度 |

**卡死调参**：
- 频繁误判卡死 → 增大 `stuck_timeout`（≥12s）
- 真实卡死恢复慢 → 减小 `stuck_timeout`（≤8s）

---

## 4. 语义感知 — `config/semantic_perception.yaml`

| 参数 | 默认值 | 影响 |
|------|--------|------|
| `confidence_threshold` | — | YOLO 检测置信度门槛，降低→更多检测但噪声增加 |

---

## 5. 快速验证命令

```bash
# 检查当前参数是否生效
ros2 param get /semantic_planner confidence_threshold
ros2 param get /frontier_explorer frontier_novelty_weight

# 在线热更新（部分参数支持）
ros2 param set /frontier_explorer frontier_novelty_weight 0.4

# 查看所有可设参数
ros2 param list /semantic_planner
```

---

## 6. 参数调优流程

```
1. 修改 config/*.yaml
2. 重启节点: ros2 launch launch/navigation_explore.launch.py target:="..."
3. 观察日志: ros2 topic echo /nav/semantic_status
4. 定量评估: tests/benchmark/eval_navigation.sh
```

---

*最后更新: 2026-03-20*
