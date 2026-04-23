# Lingtu Operations CLI

单一入口, 取代分散的 curl / systemctl / journalctl / Web 按钮。
Script: `scripts/lingtu`, 部署在 S100P `/home/sunrise/data/SLAM/navigation/scripts/lingtu`。

## 本机 alias 配置 (一次性)

在本地 `~/.bashrc` (Linux/Mac) 或 `~/.zshrc` 加:

```bash
alias lingtu='ssh sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/lingtu"'
alias lingwatch='ssh -t sunrise@192.168.66.190 "bash ~/data/SLAM/navigation/scripts/lingtu watch"'
```

`source ~/.bashrc` 后:

```bash
lingtu status       # 一屏状态
lingwatch           # 副屏放着实时监控
```

## 子命令全表

### `status` — 一屏 8 区快照

```
=== Lingtu @ 17:26:52 ===
[1] Session   mode=idle  map=corrected_20260406_224020  can_map=True
[2] SLAM      hz=0.0Hz  live_pts=0  loc=-
[3] Robot     xy=(-,-) z=- yaw=-°  v=-m/s w=-rad/s
[4] Mission   state=IDLE  wp=0/0  replan=0  speed=1.0  deg=NONE
[5] Path      (no active plan)
[6] Ctrl      teleop=False(0)  safety=STOP
[7] Map       active  @02:57:26  map.pcd=2.6M  predufo=-  patches=105
[8] Log       (recent drift/dufomap/error)
```

**每一区读什么**:

| 区 | 字段 | 正常值 | 异常信号 |
|---|---|---|---|
| [1] Session | `mode` | idle / mapping / navigating | - |
| [2] SLAM | `hz` | 20-47 Hz (mapping), 10 Hz (navigating) | 0 Hz = 服务没起 |
| [2] SLAM | `loc` | GOOD / OK | DEGRADED / LOST |
| [3] Robot | `xy` | 合理范围 (< ±50m 室内) | `ODOM DIVERGED` = IEKF 飞了 |
| [4] Mission | `state` | IDLE / EXECUTING | FAILED |
| [4] Mission | `deg` | NONE | CRITICAL |
| [5] Path | `pts/len/next/goal` | 导航时有值 | - |
| [6] Ctrl | `safety` | OK (0) / WARN (1) | STOP (2/3) |
| [7] Map | `dufo -Npts (X%)` | 有动态物体图 > 0% | - |
| [8] Log | drift / dufomap / error | - | - |

### `watch [interval]` — 持续监控

```bash
lingtu watch       # 默认每 1 秒
lingtu watch 2     # 每 2 秒
```

内部 = `watch -c -n <interval> bash <script> status`。

**建图/导航时副屏开这个**, 所有状态变化 1 秒刷新一次。

### `map` — 建图 session

```bash
lingtu map start              # 进 mapping 模式 (启动 slam + slam_pgo)
lingtu map save lab_0423      # PGO 保存 + DUFOMap 清洗 + tomogram/grid 重建
lingtu map end                # 结束回 idle (全停 SLAM)
lingtu map list               # 列已保存地图
```

`save` 预计 15-60 秒。Toast 或返回 JSON 包含 `dynamic_filter` 统计:
```json
{
  "success": true,
  "name": "lab_0423",
  "dynamic_filter": {
    "success": true,
    "orig_count": 170086,
    "clean_count": 169512,
    "dropped": 574,
    "elapsed_s": 0.8
  }
}
```

### `nav` — 导航 session

```bash
lingtu nav start corrected_20260406_224020    # 进 navigating + 加载地图 (启动 slam + localizer)
lingtu nav goal 3.5 2.1 0.0                   # 发 map-frame 目标 (x, y, yaw)
lingtu nav stop                               # 同 map end
```

### `svc` — systemd 服务控制

```bash
lingtu svc status                     # 4 个核心服务的 enable / active 状态
lingtu svc restart slam               # 重启 Fast-LIO2
lingtu svc restart lingtu             # 重启 gateway (清 odom 缓存)
lingtu svc restart all                # 重启所有 SLAM + gateway
lingtu svc stop slam                  # 停单个服务
```

### `log` — journalctl 过滤

```bash
lingtu log drift      # drift_watchdog 触发历史
lingtu log dufomap    # DUFOMap 保存统计 (最近 1 小时)
lingtu log error      # 最近 30 分钟 ERROR 级报错 (过滤 VectorMemory/webrtc 噪声)
lingtu log tail       # 实时滚动 (Ctrl+C 退出)
lingtu log all        # 最近 10 分钟全部
```

### `health` — REST 原样

```bash
lingtu health         # GET /api/v1/health | python3 -m json.tool
```

完整模块清单 + 各传感器状态 + 细粒度 hz/fps 统计。

## 典型工作流

### 工作流 A: 建图 (含动态物体验证)

```bash
# 1. 副屏开监控
lingwatch

# 2. 主屏操作
lingtu status                         # 确认 idle + can_map=True
lingtu map start                      # 进 mapping
# 等 5-10 秒, lingwatch 看 mode=mapping, hz>20, robot xy=(0,0)
# 手机/Web 推狗走, 中间让人走过 15 秒
lingtu map save lab_0423_test         # 保存 (~30s)
# lingwatch 看 [7] Map 出现 "dufo -N pts (X%)" 就成功了
lingtu log dufomap | tail -5          # 再看一眼日志确认
```

### 工作流 B: 加载地图 + 导航

```bash
lingtu map list                       # 找地图名
lingtu nav start lab_0423_test        # 进 navigating
# lingwatch 看 mode=navigating, loc=GOOD
lingtu nav goal 3.5 2.1               # 点一个目标
# lingwatch 看 [5] Path 出现 pts/len/goal,  [4] Mission=EXECUTING
# wp 递增, 到 wp=N/N 后 state=SUCCEEDED
```

### 工作流 C: 故障排查

```bash
# 情况 1: Robot 显示 ODOM DIVERGED
#   → 等 60s watchdog 自动救, 或:
lingtu svc restart slam

# 情况 2: 保存失败
lingtu log error | tail -20
lingtu log dufomap

# 情况 3: SLAM 启动不起来
lingtu svc status
# 如果 slam=inactive 且 mode=mapping, 尝试:
lingtu map end && sleep 2 && lingtu map start
```

## 退出码约定

- 0 成功
- 1 参数错 (用法错)
- 2 运行时错 (curl/ssh/systemctl 失败)

## 相关文档

- `docs/05-specialized/dynamic_obstacle_removal.md` — DUFOMap Phase 1 + 2 实装
- `docs/05-specialized/slam_drift_watchdog.md` — IEKF 飞值兜底机制
