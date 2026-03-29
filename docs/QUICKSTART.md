# LingTu 快速上手

## 安装

```bash
cd ~/data/SLAM/navigation
pip install -e .
```

安装后 `lingtu` 命令全局可用。

## 环境准备（S100P）

```bash
# 每次开机后执行一次
source /opt/ros/humble/setup.bash
source /opt/nova/lingtu/v1.8.0/install/setup.bash
```

建议加到 `~/.bashrc` 中。

## 六种运行模式

| 命令 | 用途 | 需要硬件 |
|------|------|---------|
| `lingtu map` | 建图 | LiDAR + IMU |
| `lingtu nav` | 导航（有地图） | LiDAR + IMU + 相机 |
| `lingtu explore` | 探索（无地图） | LiDAR + IMU + 相机 |
| `lingtu sim` | MuJoCo 仿真 | 无 |
| `lingtu dev` | 语义管道开发 | 无 |
| `lingtu stub` | 框架测试 | 无 |

## 典型工作流

### 第一步：建图

首次到新环境，插上 LiDAR，运行：

```bash
lingtu map
```

遥控机器人走一圈后：

```
> map save building_a
```

系统自动保存点云 + 生成导航用的栅格地图。

### 第二步：导航

```bash
lingtu nav
```

选择地图并开始导航：

```
> map use building_a
> go 体育馆
```

### 其他导航指令

```
> go 红色椅子旁边          # 一句话导航
> go 上次放背包的地方       # 向量记忆搜索
> agent 找到灭火器然后标记   # 多步 Agent（自动规划执行）
> stop                      # 急停
> cancel                    # 取消当前任务
> status                    # 查看系统状态
```

## REPL 命令速查

启动后输入 `help` 查看所有命令。

### 导航

```
go <目标>              一句话导航（自然语言）
navigate <x> <y>       坐标导航
stop                   急停
cancel                 取消任务
status                 系统状态
```

### 地图管理

```
map list               列出所有地图（* 标记当前）
map save <名称>        保存当前 SLAM 地图
map use <名称>         切换活跃地图
map build <名称>       重建栅格地图
map delete <名称>      删除地图
```

### 语义地图

```
smap status            KG + 拓扑图统计
smap rooms             列出所有已知房间
smap save              立即保存语义地图
smap load <目录>       加载语义地图
smap query <文本>      查询哪个房间有该物体
```

### 向量记忆

```
vmem query <文本>      模糊搜索历史位置
vmem stats             记忆统计
```

### Agent

```
agent <多步指令>       多轮 Agent（自动执行多个步骤）
```

示例：
```
> agent 找到红色椅子，走过去，标记为休息区
```

### 遥控器

```
teleop status          遥控状态（客户端数、端口）
teleop release         释放手动控制
```

手机/浏览器连接：`ws://<机器人IP>:5060/teleop`

发送 JSON：`{"type": "joy", "lx": 0.5, "ly": 0.0, "az": -0.3}`

3 秒无操作自动释放，恢复自主导航。

### 监控

```
health                 详细健康检查
watch                  实时状态刷新
module <名称>          查看单个模块详情
connections            显示所有端口连线
log debug|info         切换日志级别
config                 显示当前配置
```

## MCP（AI Agent 控制）

系统启动后，MCP 服务在 `http://<机器人IP>:8090/mcp` 自动运行。

### 连接 Claude Code

```bash
claude mcp add --transport http lingtu http://192.168.66.190:8090/mcp
```

连接后 Claude Code 可以直接控制机器人：
- 导航：navigate_to, navigate_to_object, stop, get_navigation_status
- 感知：get_room_summary, query_room_for_object, find_object
- 记忆：query_location, tag_location, get_memory_stats
- 伺服：follow_person, stop_servo
- 系统：emergency_stop, get_teleop_status

### 19 个 MCP Tools

| 类别 | Tools |
|------|-------|
| 导航 | navigate_to, stop_navigation, cancel_mission, get_navigation_status, start_patrol |
| 语义地图 | get_room_summary, query_room_for_object, get_exploration_target, get_semantic_status |
| 向量搜索 | query_location, get_memory_stats |
| 视觉伺服 | find_object, follow_person, stop_servo, get_servo_status |
| 安全 | get_safety_status, emergency_stop |
| 遥控 | get_teleop_status, force_release |

## 网络端口

| 端口 | 服务 | 说明 |
|------|------|------|
| 5050 | Gateway | HTTP/WebSocket/SSE API |
| 8090 | MCP | AI Agent JSON-RPC |
| 5060 | Teleop | 手机遥控 WebSocket |

## 参数覆盖

任何 profile 的参数都可以用 `--` 覆盖：

```bash
lingtu nav --llm mock           # 用 mock LLM（不需要 API key）
lingtu nav --detector yoloe     # 换检测器
lingtu nav --planner pct        # 换规划器
lingtu nav --daemon             # 后台运行
lingtu stop                     # 停止后台进程
```

## 故障排查

| 问题 | 解决 |
|------|------|
| `ros2 not found` | `source /opt/ros/humble/setup.bash` |
| `Port already in use` | 系统会自动清理，或手动 `fuser -k 5050/tcp` |
| `No map loaded` | 先 `lingtu map` 建图，再 `map use <名称>` |
| `GoalResolver not available` | 检查 `DASHSCOPE_API_KEY` 环境变量 |
| CLIP 加载失败 | 检查 `~/.cache/huggingface/` 模型文件 |
| SSH 断开 | 用 `lingtu nav --daemon` 后台运行 |
