# SLAM Drift Watchdog

> 后台守护线程, 自动 detect + recover Fast-LIO2 IEKF 长期静置时协方差发散
> 导致的 odom 飞值问题 (xy 飘到 10¹² 米级)。

## 问题背景

**Fast-LIO2 = IEKF (Iterated EKF) 融合激光 + IMU**, 卡尔曼滤波靠新观测
压协方差矩阵。狗静置时:
- 激光扫到**不变的墙和地面**, 信息增益 ≈ 0
- IMU 只有白噪声, 无系统性运动
- P 矩阵持续膨胀 → 数值发散 → xy 飙到 10¹² m

实测案例:
- 昨天: `xy=(1528868, 276184)` (150 万米)
- 今天: `xy=(-397555399720, 2464983295258)` (四千亿米 / 2.4万亿米)

下游影响:
- `dialogue.localization = DEGRADED (slam_weak)`
- `mission.degeneracy = CRITICAL`
- `safety.level = WARN`
- NavigationModule 拿到飞值 → PCT 规划 start_idx 越界 → 路径全废
- 直接建图会把飞掉的 odom 带进地图 → 保存的 map.pcd 全废

## 历史处置 (没 watchdog 之前)

人工介入:
1. SSH 上去
2. `curl -X POST http://localhost:5050/api/v1/session/end` 结束 session
3. `sudo systemctl stop slam slam_pgo localizer`
4. `sudo systemctl restart lingtu.service` (清 gateway 缓存飞值)
5. 等 10s 重启完成
6. Web 重新点「开始建图」拉起 slam + slam_pgo

问题: **谁知道要去 ssh**? 森哥不在场 = 狗就一直"装死"。

## Watchdog 设计

### 定时检测 (每 60s tick)

```python
# src/gateway/gateway_module.py:_drift_watchdog_loop
time.sleep(60)
with self._state_lock:
    odom = self._odom
x, y, z = abs(odom["x"]), abs(odom["y"]), abs(odom["z"])
v = abs(odom["vx"])
if x > 1000 or y > 1000 or z > 1000 or v > 10:
    restart()
```

阈值依据:
- 室内最大活动范围 ~100m, 1000m 是 10× 安全边界
- 四足狗极限速度 3 m/s, 10 m/s 远超物理范围

### 恢复流程

```python
# _drift_restart_do_restart
svc = get_service_manager()
mode_before = self._session_mode      # 记住 restart 前的 mode
svc.stop("slam", "slam_pgo", "localizer")   # 杀所有 SLAM 进程 → IEKF 死透
with self._state_lock:
    self._odom = {}                           # 清 gateway 缓存,避免下游继续吃飞值
    self._odom_timestamps.clear()
self.push_event({"type":"slam_drift","level":"error",...})  # Web 可加 banner
time.sleep(2.0)                              # 等进程完全退出
if mode_before == "mapping":
    svc.ensure("slam", "slam_pgo")            # 按 session 拉起对应服务
elif mode_before == "navigating":
    svc.ensure("slam", "localizer")
# idle: 留着不起, 用户进模式时再起
```

### 防抖冷却 (300s)

```python
if time.time() - self._drift_last_restart_ts < COOLDOWN:
    logger.warning("cooldown not elapsed, skipping")
    continue
```

如果 SLAM 真坏了 (硬件问题 / 驱动问题), 不陷入每 60s 重启的循环;
WARN 日志会持续, 让人看到后手动介入。

## Env 变量

| 变量 | 默认 | 含义 |
|---|---|---|
| `LINGTU_DRIFT_WATCHDOG` | 1 | 总开关 (=0 关闭整个 watchdog) |
| `LINGTU_DRIFT_WATCHDOG_INTERVAL` | 60 | 检查周期 (秒) |
| `LINGTU_DRIFT_WATCHDOG_XY_LIMIT` | 1000 | xy 绝对值阈值 (米) |
| `LINGTU_DRIFT_WATCHDOG_V_LIMIT` | 10 | 线速度阈值 (米/秒) |
| `LINGTU_DRIFT_WATCHDOG_COOLDOWN` | 300 | 连续重启最小间隔 (秒) |

systemd 修改: `/etc/systemd/system/lingtu.service` 加
`Environment=LINGTU_DRIFT_WATCHDOG_XY_LIMIT=500` → restart lingtu.service。

## 观测点

### 监控 CLI

```bash
lingtu status        # 看 [3] Robot 区是否 "ODOM DIVERGED"
lingtu log drift     # 看 drift_watchdog 最近触发历史
lingtu log tail      # 实时滚动 journalctl
```

### journalctl 关键行

**启动时**:
```
drift_watchdog: enabled, interval=60s, |xy|<1000m, |v|<10.0m/s
```

**触发时** (ERROR 级):
```
drift_watchdog: IEKF DIVERGED xy=(1528868,276184) z=5 v=0.1 — restarting slam.service
drift_watchdog: restart complete (mode=navigating)
```

**冷却期**:
```
drift_watchdog: still diverged (xy=1528868,276184 v=0.1) but cooldown (245s) not elapsed
```

### SSE 事件

Web 前端可订阅 `type=slam_drift` 事件:
```json
{
    "type": "slam_drift",
    "level": "error",
    "xy": 1528868,
    "v": 0.1,
    "action": "slam_restart",
    "count": 1
}
```

(前端 banner 未实装 — P2 todo)

## 关键文件

| 文件 | 角色 |
|---|---|
| `src/gateway/gateway_module.py:__init__` | `_drift_*` 实例变量 + env 解析 |
| `src/gateway/gateway_module.py:start` | 启动 `_drift_watchdog_thread` |
| `src/gateway/gateway_module.py:_drift_watchdog_loop` | 定时检测 |
| `src/gateway/gateway_module.py:_drift_restart_do_restart` | stop+clear+ensure 恢复 |
| `memory/slam_static_drift_bug.md` | 历史问题记录 (人工修法时代) |

## 局限 (P3+ 才解决)

- **治标不治本**: IEKF 还是会发散, 只是事后被救; 10-20s 停机窗口
- **不能阻止建图途中飞**: 如果建图过程中飞一次, 中间几秒 patches 带错
- **对 Fast-LIO2 C++ 源码零改动**: 要预防必须改 IEKF 加静止检测 + 协方差 clamp

### 如果要根治 (Phase 3)

方向: 改 Fast-LIO2 C++ 源码, 加"静止检测 + 协方差上限":

```cpp
// fastlio2/src/IKFOM_toolkit/esekfom.hpp 附近
if (imu_still_for_seconds(30)) {      // IMU 静止 30s
    P = P.clamp_max(1e-3);            // 协方差上限
    // 可选: reset twist portion
}
```

需要改 HKU-MARS 的 Fast-LIO2 上游代码或 fork。不是本 repo 范围。
