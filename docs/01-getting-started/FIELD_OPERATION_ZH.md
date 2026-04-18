# LingTu 外场操作手册

> 对象:一个人带笔记本进场地,远程控制 S100P 跑建图 / 导航 / 避障 / 录 bag。
> 假设:机器人已开机,S100P 在同一 LAN,你能 `ssh sunrise@192.168.66.190`(密码 `sunrise`)。
> 最后更新:2026-04-18

---

## 0. 两种操作方式 — 选一个

| 方式 | 适合场景 | 入口 |
|---|---|---|
| **A. SSH + `lt` CLI**(本手册主推) | 全程命令行,一台笔记本就行,适合熟手 | `ssh sunrise@192.168.66.190` 然后 `lt` |
| **B. 浏览器 Dashboard** | 想看点云/图像/地图,视觉反馈强 | `http://192.168.66.190:5050/` |

两者共用同一套后端 API,你可以同时开 — 浏览器看画面,`lt` 发命令。

---

## 1. 5 秒健康检查(每次上场第一件事)

```bash
ssh sunrise@192.168.66.190
lt
```

输出例:
```
─── Lingtu Status ───
    "mode": "navigating"
    "active_map": "corrected_20260406_224020"
    "icp_quality": 0.05
    "localizer_ready": true
    "error": ""

─── Recording ───
    "recording": false

─── Process ───
16875 python3 lingtu.py nav --llm mock --no-repl

─── SLAM topics (5s sample) ───
  /nav/odometry: 9.981 Hz
  /nav/map_cloud: 0.498 Hz
  /localization_quality: 9.317 Hz
```

**健康判断**:
| 指标 | 绿 | 黄 | 红 |
|---|---|---|---|
| mode | navigating/mapping | idle | 空 |
| icp_quality | 0 < x < 0.3 | 0.3–0.5 | 0 或 -1 |
| localizer_ready | true | — | false |
| /nav/odometry | 8-10 Hz | 3-8 | <3 或空 |
| /localization_quality | ~10 Hz | — | 没数据 |
| process | 有 lingtu.py | — | 空 |

**红了怎么办** → 见第 7 节故障处理。

---

## 2. 导航流程(90% 外场任务)

### Step 1 — 确认在导航模式

```bash
lt                    # 看 mode
# 如果 mode != navigating:
lt session nav        # 切到导航模式(默认加载 corrected_20260406_224020 地图)
lt session nav my_custom_map   # 或指定地图
```

### Step 2 — 重定位(让机器人知道自己在地图里哪里)

1. **目测**机器人在地图中的位置(X, Y),朝向(0 = 东,π/2 = 北)
2. 发重定位:
   ```bash
   lt reloc 0 0 0            # 如果机器人就在地图原点
   lt reloc 2.5 -1.3 0       # 如果在 (2.5, -1.3) 朝东
   lt reloc 5 0 1.57         # 朝北(π/2)
   ```
3. **等 3 秒,再看**:
   ```bash
   lt
   ```
   `icp_quality < 0.3` + `localizer_ready: true` → 成功。
   `icp_quality > 0.5` 或 `-1` → 失败,换个位置再试(初始误差小于 2 米 ICP 才能收敛)。

### Step 3 — 发导航目标

```bash
lt nav 5 3               # 去 (5, 3)
lt nav -2 -1             # 去 (-2, -1)
```

机器人会自动:
1. 用 **PCT 规划器**(基于 3D 点云 tomogram)算出路径
2. **Pure Pursuit 跟踪**
3. 遇到障碍**局部重规划**(CMU LocalPlanner)
4. 到达或失败时 mission status 更新

看进度:
```bash
lt                       # 看 mission 状态
```

### Step 4 — 中途紧急停

```bash
lt stop                  # cmd_vel 归零,mission cancelled
```

---

## 3. 建图流程

### 开始建图

```bash
lt session end           # 如果当前在 navigating 模式,先结束
lt session map           # 切到 mapping 模式(启 fastlio2 + pgo)
```

此时 Fast-LIO2 开始建图,SLAM 轨迹累积。

### 采集数据

- 手动遥控 或 `lt nav X Y` 让机器人走完场地
- 全程保持 LiDAR 有遮挡物可以匹配(空旷/纯玻璃会退化)
- 看状态:
  ```bash
  lt                     # /nav/map_cloud Hz 应该有数字
  ```

### 保存地图

```bash
lt map save my_lab_20260418
```

系统会触发完整 pipeline:
- `/nav/save_map` → pcd + pose graph
- `_build_tomogram` → `tomogram.pickle`(规划器用)
- `_build_occupancy_snapshot` → `map.pgm` + `map.yaml`(RVIZ 兼容)
- 存放:`~/data/inovxio/data/maps/my_lab_20260418/`

### 切到导航用新地图

```bash
lt session end
lt map use my_lab_20260418
lt session nav my_lab_20260418
# 然后 reloc + nav
```

---

## 4. 录制 rosbag(故障复现用)

```bash
lt record start         # 开始录(默认 10 分钟上限,13 个话题)
# ... 做动作 ...
lt record               # 查状态(看 size/duration)
lt record stop          # 手动结束
```

录制话题清单(已硬编码在 `scripts/record_bag.sh`):
- `/nav/{lidar_scan, imu, odometry, map_cloud, registered_cloud, cmd_vel, goal_pose}`
- `/localization_quality`
- `/exploration/{way_point, path, runtime, finish}`
- `/camera/camera_info`(不含相机 raw,避免数据爆炸)

存放:`~/data/bags/web_<时间戳>/`

---

## 5. 避障测试

避障是**自动的**,不需要你主动开。只要:
1. Session 在 `navigating` 模式
2. `icp_quality` 正常(定位不漂)
3. 你发了导航目标

系统会:
- `LocalPlannerModule`(CMU 预采样路径)每帧重新打分
- `TerrainModule` 输出可通行性图
- `CmdVelMux` 按优先级仲裁最终速度

测试方法:
1. `lt nav 5 0` — 发个远目标
2. 你站在机器人和目标之间
3. 机器人应该绕开你 — 看 SLAM tab 或 `lt status`(`/vo/state` 变化)

---

## 6. 看实时日志

```bash
lt log                   # 最近 30 行
lt logf                  # tail -f(Ctrl+C 退出,不杀 lingtu)
```

关键日志行:
- `[INFO] nav.navigation_module: Goal accepted` — 目标已收
- `[INFO] nav.global_planner_service: PCT planned N waypoints` — 规划成功
- `[WARNING] slam.slam_bridge_module: Localization DIVERGED` — **SLAM 爆炸,见第 7 节**
- `[ERROR] slam.localizer: ICP failed` — 重定位未收敛

---

## 7. 故障处理速查表

| 症状 | 判断命令 | 修复 |
|---|---|---|
| 浏览器打不开 Dashboard | `ping 192.168.66.190` 不通 | 检查 WiFi / 机器人是否开机 |
| `lt` 任何子命令都 "connection refused" | `ps -ef \| grep lingtu` 没结果 | `lt restart` |
| `icp_quality = -1` 或 `0` 且 localizer 跑着 | ICP 从 (0,0,0) 对不上 | `lt reloc X Y YAW`(你估计的真实位置) |
| 位置显示 `(-5000, 20000)` 之类爆炸数字 | Fast-LIO2 静置漂移 bug | `sudo systemctl restart slam` 然后 `lt reloc` |
| 浏览器摄像头黑屏 / 话题没数据 | `lt cam` 重启相机;若 `lsusb \| grep Bootloader` 说明卡在 bootloader | **物理拔插 USB 线** |
| 导航规划失败 | `lt log` 看错误,可能是目标在障碍里 | 换个目标点 |
| 点云和 saved map 错位 | `icp_quality` 没数 → localizer 没 ready | 先重定位 |
| `lt` 命令不存在 | 装错了 | 重新部署:见第 8 节 |

---

## 8. 附录 A — 从头部署一次(机器人重装后用)

```bash
ssh sunrise@192.168.66.190

# 1. 代码同步
cd ~/data/SLAM/navigation && git pull origin main

# 2. 如果相机异常
sudo systemctl restart camera
# 验证
ros2 topic hz /camera/color/image_raw     # 应 ~7-10 Hz

# 3. 启 lingtu(tmux 里跑,断线不掉)
tmux kill-session -t lingtu 2>/dev/null
tmux new-session -d -s lingtu -x 200 -y 50 \
  'cd ~/data/SLAM/navigation && python3 lingtu.py nav --llm mock --no-repl 2>&1 | tee /tmp/lingtu_nav.log'

# 4. 等 50 秒,健康检查
sleep 50
lt
```

---

## 9. 附录 B — `lt` 命令完整列表

```
lt                        一眼看全系统(模式/ICP/话题 Hz)
lt status                 (等价)

lt nav X Y [Z]            发导航目标
lt reloc X Y YAW          重定位(初始位姿,弧度 yaw)
lt stop                   急停

lt session nav [MAP]      切导航模式
lt session map            切建图模式
lt session end            结束当前 session
lt session show           看 session 状态

lt map list               列出已保存地图
lt map save <name>        保存当前地图
lt map use <name>         切换加载地图

lt record start           开始录 bag
lt record stop            停止录 bag
lt record                 看 bag 状态

lt log                    最近 30 行日志
lt logf                   实时跟日志 (Ctrl+C 退)
lt restart                重启 lingtu
lt cam                    重启相机

lt help                   看帮助
```

---

## 10. 附录 C — 关键文件路径(排障用)

```
~/data/SLAM/navigation/         符号链接 → ~/data/inovxio/lingtu/
~/data/inovxio/lingtu/          lingtu 源码根目录
/tmp/lingtu_nav.log             lingtu 运行日志
~/data/inovxio/data/maps/       建图输出目录
~/data/bags/                    rosbag 存放
/opt/ros/humble/                ROS2 Humble
/etc/systemd/system/camera.service.d/  相机 drop-in 配置
/usr/local/bin/lt               CLI 脚本本体
```

---

## 11. 附录 D — HTTP API(不用 lt 时的 raw 命令)

所有 `lt` 命令最终走 HTTP,路径参考 `http://192.168.66.190:5050/api/v1/`:
- `GET /session`、`POST /session/start`、`POST /session/end`
- `POST /goal { x, y, z }`
- `POST /slam/relocalize { map_name, x, y, yaw }`
- `POST /cmd/stop`
- `POST /bag/start { duration, prefix }`、`POST /bag/stop`、`GET /bag/status`
- `GET /maps`、`POST /maps/save`、`POST /maps/use`

`lt` 源码:`/usr/local/bin/lt`(bash 脚本,可以直接看 / 改)。

---

## 常见问题 (FAQ)

**Q: 我 ssh 登进去 `lt` 提示 command not found?**
A: 说明机器人没装 `lt`,跑一次安装脚本(见 `scripts/install_lt.sh` 或问开发者)。

**Q: `lt reloc` 填的 yaw 是什么单位?**
A: **弧度**。`0` = 东,`π/2 ≈ 1.57` = 北,`π ≈ 3.14` = 西,`-π/2 ≈ -1.57` = 南。

**Q: 机器人跑飞了能不能远程急停?**
A: `lt stop` 立即发零速。但**物理急停按钮永远是第一优先级**,身上按一下最稳。

**Q: Dashboard 和 `lt` 能同时用吗?**
A: 能。后端是同一个 FastAPI,两条路径共存。

**Q: 建图和导航能同时跑吗?**
A: 不能。Session 机制保证互斥,切换必须 `session end` 再 `session nav/map`。

**Q: 断网重连 lingtu 会挂吗?**
A: 不会。lingtu 在 tmux 里跑,SSH 断线不影响。重连后 `lt` 继续可用。
