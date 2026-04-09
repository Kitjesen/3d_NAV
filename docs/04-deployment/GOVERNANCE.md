# LingTu 工业级部署治理

> 这不是技术问题，是组织问题。技术方案再漂亮，没有治理也会再次失控。

## 0. 为什么要写这份文档

2026-04-09，在 S100P 上发现**三套导航栈同时在跑**：
- `~/data/SLAM/navigation/` — 开发版本（手工启动，开机自起）
- `/opt/nova/cortex/` — NOVA Cortex 老部署（开机自起）
- `/opt/nova/lingtu/v1.8.0/` — OTA Agent 最新部署

三套栈互相打架：两个 livox 驱动抢同一个 LiDAR，两个独立 SLAM 链路在跑，load average = 9.36。
`/etc/systemd/system/` 里装了 12 个 `nav-*.service` 文件，**但全部 disabled+dead**——真正在跑的进程从没走过 systemd。

这份文档定义工业级治理框架，避免再次失控。

---

## 1. 根因分析

早期创业团队的混乱三大来源：

1. **没有 Single Source of Truth**  
   代码散落在多个路径，每个开发者觉得自己的路径最方便。

2. **没有 Immutable Deployment**  
   SSH 上去手改代码、手启服务、手停进程——每次都是活的，没有"冻结版本 + 原子切换"的概念。

3. **没有治理边界**  
   谁能装新 service？谁能改 systemd？谁能杀进程？没答案。每个人都能动，每个人都不负责。

---

## 2. 六条工业级原则

### 原则 1：不可变部署 (Immutable Deployment)

```
/opt/lingtu/
  ├── releases/
  │   ├── v1.7.5/          ← 冻结的历史版本
  │   ├── v1.8.0/          ← 当前版本
  │   └── v1.8.1-rc1/      ← 下一个候选
  ├── current -> releases/v1.8.0   ← symlink, 原子切换
  └── config/                      ← 可变配置, 独立于版本
      ├── robot.yaml               ← 本机参数 (标定, IP 等)
      └── secrets/
```

**规矩**：
- 每个版本一个不可变目录，部署后**不许改里面任何文件**
- 升级 = 创建新版本目录 + 切换 symlink + 重启服务
- 回滚 = 切换 symlink 回旧版本 + 重启服务
- **绝对禁止 SSH 上去改代码**。要改代码 → 发新版本。

### 原则 2：声明式服务 (Declarative Services)

所有长期运行的进程**必须**是 systemd service。没有例外。

```
/etc/systemd/system/
├── lingtu.target              ← 入口 target, 启一个就全起来
├── lingtu-brainstem.service   ← brainstem dart server (:13145)
├── lingtu-slam.service        ← Fast-LIO2 + localizer
├── lingtu-lidar.service       ← Livox driver
├── lingtu-camera.service      ← 相机 bridge
├── lingtu-nav.service         ← 导航栈主进程 (:5050, :8090)
├── lingtu-gateway.service     ← Gateway API (如果独立)
└── lingtu-ota.service         ← OTA Agent
```

**规矩**：
- 开机启动 = `systemctl enable lingtu.target`
- 查看全景 = `systemctl list-dependencies lingtu.target`
- 启 = `systemctl start lingtu.target`
- 停 = `systemctl stop lingtu.target`
- 日志 = `journalctl -u 'lingtu-*' -f`
- **绝对禁止**手动 `ros2 launch` / `python lingtu.py` / `dart run`
- 任何绕开 systemd 的启动行为视为 bug

### 原则 3：依赖图必须显式

```systemd
# lingtu-nav.service
[Unit]
After=lingtu-brainstem.service lingtu-slam.service lingtu-lidar.service
Requires=lingtu-brainstem.service lingtu-slam.service
BindsTo=lingtu-brainstem.service    # brainstem 挂了 nav 也停

[Service]
ExecStartPre=/usr/local/bin/lingtu-preflight   # 启动前检查
ExecStart=/opt/lingtu/current/bin/lingtu-nav
ExecStopPost=/usr/local/bin/lingtu-cleanup
Restart=on-failure
RestartSec=5s
```

**规矩**：
- 每个 service 明确写 `After` / `Requires` / `BindsTo`
- 启动顺序由 systemd 推导，不由脚本决定
- 失败自动重启，有退避
- `ExecStartPre` 做启动前检查（端口占用、配置合法性、硬件可达）

### 原则 4：可观测性是一等公民

一个命令看整个系统：

```bash
$ lingtu status
LingTu v1.8.0 @ S100P  (uptime 2h 15m)
─────────────────────────────────────────
STACK
  lingtu.target           ● active
  ├─ brainstem            ● active (13145, ok)
  ├─ slam                 ● active (odom 10.2 Hz)
  ├─ lidar                ● active (scan 10.0 Hz)
  ├─ camera               ● active (rgb 15.0 Hz)
  ├─ nav                  ● active (5050, mcp 8090)
  ├─ gateway              ● active
  └─ ota                  ● active (last check: 5m ago)

HEALTH
  CPU 42%  MEM 38%  LOAD 2.1
  DISK /: 45%  /opt: 12%
  NET: LAN ✓  Internet ✓

MAP
  active: warehouse_2026_04_07
  localization: CONVERGED (0.08m drift)
```

**规矩**：
- 一个命令看全貌
- 每个 service 有 `/health` 和 `/ready` 端点
- 日志统一到 `journalctl`，禁止写到乱七八糟的地方
- `/var/log/lingtu/` 是唯一允许的项目日志目录

### 原则 5：部署流程唯一、可审计

```
Developer
    ↓
git push → CI (tests + build)
    ↓
GitHub Release v1.8.1
    ↓
OTA Agent (在机器人上) 轮询 GitHub
    ↓
Download + 签名验证
    ↓
Install to /opt/lingtu/releases/v1.8.1/
    ↓
systemctl reload lingtu.target  (symlink 切换 + 滚动重启)
    ↓
健康检查 → 失败自动回滚
    ↓
记录到 /var/log/lingtu/deployments.log
```

**规矩**：
- **唯一的部署路径是 OTA Agent**。SSH 上去手改代码一律视为未授权操作。
- 每次部署留痕：版本、时间、触发者、健康检查结果
- 回滚是一等公民，不是"出事才想起来"

### 原则 6：清理是纪律

**Legacy 必须显式 decommission**：

```bash
lingtu legacy list              # 查看所有 legacy 组件
lingtu legacy disable <name>    # 停用但保留
lingtu legacy archive <name>    # 归档到 /opt/lingtu/archive/
lingtu legacy purge <name>      # 永久删除 (需 --force)
```

**规矩**：
- 发现 legacy → 登记 → disable → 观察一周 → archive → 一个月后 purge
- 不允许"先留着万一有用"——留着就是新的混乱源
- 每月一次清理扫描，输出报告

---

## 3. 治理边界：谁能做什么

| 操作 | 谁能做 | 怎么做 |
|------|--------|--------|
| 写代码 | 开发者 | PR → CI → merge |
| 发版本 | 项目负责人 | `git tag v1.8.1 && git push --tags` |
| 部署到机器人 | OTA Agent（自动）| 拉 GitHub Release |
| SSH 改代码 | **没有人** | 禁止 |
| 手动重启服务 | 运维（有记录）| `lingtu restart <svc>` 会写日志 |
| 添加新 service | 走 PR | 禁止 SSH 上去 `systemctl` |
| 清理 legacy | 走流程 | `lingtu legacy archive` |

### 红线

- **机器人上不允许直接写代码**
- **不允许手动 `apt install` / `pip install`** — 依赖写 requirements.txt，通过部署流程
- **不允许手动跑 `ros2 launch`** — 调试也要用 `lingtu debug start <svc>` 走记录
- **不允许有"暂时的"** — 任何临时操作必须有 decommission 计划和截止日期

---

## 4. 分阶段推进路线

### 阶段 0：止血（今天）

**目标**：让机器人只跑一套栈，先停止雪上加霜。

```bash
# 1. 快照当前状态
lingtu snapshot create pre-cleanup

# 2. 停掉所有非 OTA 的后台进程（先不删）
systemctl stop ota-agent
pkill -STOP -f "lingtu.py nav"
pkill -STOP -f "/opt/nova/cortex"
pkill -STOP -f "data/SLAM/navigation"

# 3. 只保留基础（brainstem + 安全层），其他全停
```

不删任何代码，只停进程。先让机器人回到可控状态，再决定保留什么。

### 阶段 1：摸清楚（1 天）

**目标**：搞清三套栈谁是对的、谁是 legacy、谁能扔。

调查清单：
- [ ] `/opt/nova/cortex/` — 这是 nova-dog 项目的，LingTu 用得上吗？
- [ ] `/opt/nova/lingtu/v1.8.0/` — OTA 部署的版本，对应哪个 git tag？
- [ ] `~/data/SLAM/navigation/` — 开发版本，和 OTA 版本差异多大？
- [ ] brainstem (pid 1729) — 哪个目录启动的？用的哪版 profile？
- [ ] 开机自启机制 — 是 rc.local / init / systemd generator / bashrc？

**产出**：`docs/04-deployment/S100P_STACK_INVENTORY.md` —— 哪个留、哪个删、决策依据。

### 阶段 2：立规矩（3 天）

1. **代码归位**
   - 只保留 `/opt/lingtu/releases/v1.8.0/` 一个位置
   - `~/data/` 清空（备份到 archive）
   - `/opt/nova/cortex/` 如不依赖，archive

2. **Systemd 重建**
   - 写 `lingtu.target`
   - 清理 `/etc/systemd/system/` 里 12 个 nav-* service（大部分是摆设）
   - 重写为 7 个真正用的 service
   - 明确 dependencies

3. **写 `DEPLOYMENT.md`**
   - 开机启动顺序
   - 健康检查方式
   - 更新流程
   - 回滚流程
   - 常见故障排查

4. **开机自启干净化**
   - 禁掉所有 legacy 启动脚本
   - 只留 `systemctl enable lingtu.target`

### 阶段 3：自动化（1 周）

1. **`lingtu` CLI 工具**
   - `lingtu status` / `start` / `stop` / `deploy` / `rollback` / `snapshot`
   - 机器人上**唯一**的运维入口

2. **部署流水线**
   - OTA Agent 从 GitHub Release 拉包
   - 安装到 `/opt/lingtu/releases/<version>/`
   - 原子切换 symlink
   - 健康检查 → 失败自动回滚
   - 记录到 deployments.log

3. **监控告警**
   - Gateway `/health` 每 30s 自检
   - 超过 3 次失败 → 告警
   - 磁盘 > 80% / 内存 > 90% 告警

### 阶段 4：守规矩（长期）

每周一次 5 分钟检查：

```bash
ssh sunrise@<robot> lingtu doctor
```

输出应该永远是：

```
✓ single stack running (lingtu.target)
✓ no unauthorized processes
✓ no legacy services
✓ version matches latest release
✓ all health checks green
```

任何 ✗ 都是一个 issue。

---

## 5. 每月例行

第一周周五：
1. 跑 `lingtu doctor` 全量检查
2. 回顾 deployments.log，看有没有异常
3. 清理超过 30 天的 legacy archive
4. 更新本文档如果有新发现

---

## 附录 A：当前 S100P 现状（2026-04-09 快照）

### 并行运行的三套栈

**栈 1：`~/data/SLAM/navigation/`**（开发版，13:54 开机自起）
- pid 1729 `dart:server.dart` — brainstem，监听 13145
- pid 1730 `livox_ros_driver2` — LiDAR 驱动（第一份）
- pid 1731 `fastlio2 lio_node` — Fast-LIO2 SLAM
- pid 1733 `localizer_node` — ICP 定位

**栈 2：`/opt/nova/cortex/current/services/`**（NOVA Cortex，13:56 自起）
- pid 5449 telemetry-hub
- pid 5480 dog-safety-service
- pid 5510 dog-control-service
- pid 5536 nav-gateway
- pid 5557 askme-edge-service

**栈 3：`/opt/nova/lingtu/v1.8.0/`**（OTA 部署，14:31 启动）
- pid 9765 `python3 lingtu.py nav --daemon` — 监听 5050 + 8090
- pid 9887 `livox_ros_driver2` — LiDAR 驱动（**第二份，冲突**）

### Systemd 装了但没用的 services

```
nav-autonomy.service      disabled (摆设)
nav-driver.service        disabled
nav-grpc.service          disabled
nav-lidar.service         disabled
nav-monitor.service       disabled
nav-perception.service    disabled
nav-planning.service      disabled
nav-semantic.service      disabled
nav-slam.service          disabled
nav-lidar-network.service enabled  (只是配网，跑完退出)
ota-agent.service         enabled  (唯一真正在跑的)
ota-daemon.service        disabled
```

### 关键观察

- **端口监听正常**：13145 (brainstem) / 5050 (gateway) / 8090 (mcp) 都通
- **ROS2 节点在跑**：`/camera_bridge` `/lingtu_ros2_driver` `/livox_driver_node`
- **但全部绕开 systemd** — 开机启动机制未知（不是 systemd 不是 bashrc 不是 tmux）
- **load average 9.36** — 明显过载，因为进程重复
