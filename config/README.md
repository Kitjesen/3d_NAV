# config/ — LingTu 配置文件

所有运行时配置的单一来源 (Single Source of Truth)。修改这里的文件会影响全部运行模式（真机、仿真、开发）。

## 配置文件索引

### 核心配置

| 文件 | 用途 | 关键字段 |
|------|------|---------|
| `robot_config.yaml` | 机器人物理参数和传感器配置 | 相机内参/外参、LiDAR 外参、运动控制增益、端口、标定参数 |
| `topic_contract.yaml` | ROS2 话题名称合约 | 所有模块间通信的标准话题名，launch 时 remap 到此 |
| `layer_contract.yaml` | 模块层次架构合约 | L0-L6 六层架构定义、模块依赖关系、端口类型 |

### 导航与规划

| 文件 | 用途 | 关键字段 |
|------|------|---------|
| `far_planner.yaml` | 远距离全局规划参数 | 搜索半径、路径平滑、采样密度 |
| `pointlio.yaml` | Point-LIO SLAM 参数 | IMU 噪声 (na/ng/nba/nbg)、LiDAR-IMU 外参、分辨率 |
| `qos_profiles.yaml` | ROS2 QoS 配置 | 各话题的 reliability/durability/depth |

### 语义导航

| 文件 | 用途 | 关键字段 |
|------|------|---------|
| `semantic_planner.yaml` | 语义规划器配置 | LLM backend (kimi/openai/claude/qwen)、目标解析阈值、agent_loop 参数 |
| `semantic_perception.yaml` | 感知模块配置 | 检测器 backend (bpu/yoloe)、CLIP 编码器、置信度阈值 |
| `semantic_exploration.yaml` | 探索模式覆盖 | frontier 评分权重、探索半径、信息增益参数 |

### DDS 传输

| 文件 | 用途 |
|------|------|
| `cyclonedds.xml` | CycloneDDS 配置 (S100P 默认 DDS 实现) |
| `fastdds_no_shm.xml` | FastDDS 禁用共享内存模式 (Docker 兼容) |

## robot_config.yaml 关键段落

这是最常修改的文件，以下是各段落说明：

### 相机

```yaml
camera:
  position_x/y/z:     # 相机在机体坐标系的安装位置 (m)
  roll/pitch/yaw:      # 相机光轴旋转 (rad)
  fx/fy/cx/cy:         # 内参 (会被 ROS2 CameraInfo 覆盖)
  width/height:        # 分辨率
  depth_scale:         # 深度值转米的系数
  rotate:              # 安装旋转补偿 (0/90/180/270 度)
  dist_k1..k3/p1/p2:  # 畸变系数
```

### LiDAR

```yaml
lidar:
  offset_x/y/z:       # LiDAR-IMU 平移外参 (m)
  roll/pitch/yaw:      # LiDAR-IMU 旋转外参 (rad)
```

### 服务端口

```yaml
gateway:
  port: 5050           # Dashboard HTTP/WS/SSE
  mcp_port: 8090       # MCP JSON-RPC (AI agent 接口)
brainstem:
  host: "127.0.0.1"
  port: 13145          # Brainstem gRPC (运动控制)
```

### 运动控制

```yaml
control:
  yaw_rate_gain:       # 偏航角速度增益
  max_yaw_rate:        # 最大偏航角速度 (deg/s)
  max_accel:           # 最大加速度 (m/s^2)
```

## 标定参数写入流程

传感器标定结果通过 `calibration/apply_calibration.py` 自动写入 `robot_config.yaml`：

1. 相机内参 → `camera.fx/fy/cx/cy + dist_*`
2. IMU 噪声 → `pointlio.yaml` 的 `na/ng/nba/nbg`
3. LiDAR-IMU 外参 → `lidar.offset_* + roll/pitch/yaw`
4. 相机-LiDAR 外参 → `camera.position_* + roll/pitch/yaw`

验证：`python calibration/verify.py`

## LLM 配置

语义导航使用的 LLM 通过环境变量 + `semantic_planner.yaml` 配置：

```bash
# 环境变量 (API Key)
export MOONSHOT_API_KEY="..."           # Kimi (默认)
export DASHSCOPE_API_KEY="..."          # Qwen (备选)
export OPENAI_API_KEY="sk-..."          # OpenAI
export ANTHROPIC_API_KEY="sk-ant-..."   # Claude
```

```yaml
# semantic_planner.yaml 选择 backend
llm:
  backend: "kimi"
  model: "kimi-k2.5"
```

## 修改建议

- 修改后运行 `python lingtu.py doctor` 自检
- 标定参数不要手改，用 `calibration/apply_calibration.py`
- `topic_contract.yaml` 和 `layer_contract.yaml` 是架构合约，修改前确认影响范围
- DDS 配置只在跨机通信或 Docker 部署时需要调整
