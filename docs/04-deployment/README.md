# LingTu S100P 部署手册

> 迷路了？从这里开始。

---

## 一眼看全局

```bash
systemctl status robot-brainstem robot-camera lingtu
```

或者看日志：
```bash
journalctl -u robot-slam -u lingtu -f --since "5 min ago"
```

---

## 三层架构

```
硬件层  robot-camera     orbbec gemini330  → /camera/...  [systemd]

控制层  robot-brainstem  Dart gRPC:13145   → 腿部运动控制  [systemd]

算法层  lingtu           python lingtu.py  → HTTP:5050 MCP:8090  [systemd]
          └─ 内部管理:
               livox_ros_driver2  → /nav/lidar_scan /nav/imu
               fastlio2 lio_node  → /nav/odometry /nav/registered_cloud
               localizer_node     → /nav/odometry (有地图时覆盖)
```

**原则**：robot-* 是地基，lingtu 是住在上面的房子。OTA 只更新 lingtu。

> **注**：`robot-lidar` / `robot-fastlio2` / `robot-localizer` service 文件已备份在
> `docs/04-deployment/services/`，当前 disabled。lingtu 以 managed 模式自行启动这三个
> 节点。未来切换到 bridge 模式后可独立启用。

---

## 服务清单（当前启用）

| 服务 | 作用 | 进程 | enabled |
|------|------|------|---------|
| `robot-camera` | Orbbec Gemini 330 相机 | `component_container` | ✓ |
| `robot-brainstem` | 腿部运动控制 gRPC:13145 | `dart han_dog/bin/server.dart` | ✓ |
| `lingtu` | 导航算法主栈 (含 SLAM) | `python3 lingtu.py nav` | ✓ |

---

## 常用命令

### 启动全部
```bash
sudo systemctl start robot-brainstem robot-camera lingtu
```

### 只停算法层（保留腿控 + 相机）
```bash
sudo systemctl stop lingtu
```

### 停止全部
```bash
sudo systemctl stop lingtu robot-brainstem robot-camera
```

### 重启单个服务
```bash
sudo systemctl restart lingtu
```

### 看某服务日志
```bash
journalctl -u lingtu -f
journalctl -u robot-fastlio2 -n 100
```

### 检查端口
```bash
ss -tnlp | grep -E "13145|5050|8090"
```

---

## 目录布局

```
/opt/lingtu/
  current -> releases/v2.0.0/    ← symlink，OTA 切换这个
  releases/
    v2.0.0/                      ← 当前版本（不可变）
  config/
    ros2-env.sh                  ← ROS2 环境变量（所有服务共用）
  logs/                          ← lingtu 运行日志
  models/                        ← BPU .hbm 模型

/etc/systemd/system/
  robot-lidar.service
  robot-camera.service
  robot-brainstem.service
  robot-fastlio2.service
  robot-localizer.service
  lingtu.service

/home/sunrise/data/
  SLAM/navigation/               ← ROS2 工作区（colcon build 产物在此）
  brainstem/                     ← Dart brainstem 源码
  dart-sdk/                      ← Dart SDK
  nova/maps/                     ← 预建地图 (.pcd)
    active/map.pcd               ← localizer 使用的地图
```

---

## OTA 更新 LingTu

1. 推新代码到 GitHub，打 tag `v2.x.x`
2. OTA Agent 自动拉取，安装到 `/opt/lingtu/releases/v2.x.x/`
3. 切 symlink：`ln -sfn /opt/lingtu/releases/v2.x.x /opt/lingtu/current`
4. 重启：`sudo systemctl restart lingtu`
5. 健康检查：`curl http://localhost:5050/health`

**只有 `/opt/lingtu/current` 被更新，robot-* 服务不受影响。**

---

## 回滚

```bash
# 查看可用版本
ls /opt/lingtu/releases/

# 切回旧版本
sudo ln -sfn /opt/lingtu/releases/v1.9.0 /opt/lingtu/current
sudo systemctl restart lingtu
```

---

## 故障排查

### lingtu 启动失败
```bash
journalctl -u lingtu -n 50
# 常见原因：ROS2 topic 没准备好 → 检查 robot-fastlio2 是否在跑
systemctl status robot-fastlio2
```

### LiDAR 没数据
```bash
journalctl -u robot-lidar -n 30
# 检查 USB 连接 + MID360 IP (192.168.1.1xx)
ros2 topic hz /nav/lidar_scan
```

### SLAM 漂移 / 定位丢失
```bash
# 重启定位栈
sudo systemctl restart robot-fastlio2 robot-localizer
# 查看定位频率
ros2 topic hz /nav/odometry
```

### brainstem 无响应（gRPC:13145）
```bash
journalctl -u robot-brainstem -n 30
sudo systemctl restart robot-brainstem
```

### 端口冲突（老进程残留）
```bash
# 查找占用端口的进程
ss -tnlp | grep -E "13145|5050|8090"
# 如果有不属于 systemd 管理的进程
ps aux | grep -E "lingtu|fastlio|localizer|dart"
# 清理后重启服务
sudo systemctl restart lingtu
```

---

## 开机自启配置

```bash
# 启用开机自启（已配置好）
sudo systemctl enable robot-brainstem robot-camera lingtu

# 查看当前 enable 状态
systemctl list-unit-files | grep -E "robot-|^lingtu"
```

---

## service 文件位置

所有 service 文件模板在 git 仓库：
```
lingtu/docs/04-deployment/services/
  robot-lidar.service
  robot-camera.service
  robot-brainstem.service
  robot-fastlio2.service
  robot-localizer.service
  lingtu.service
  ros2-env.sh          ← 环境变量脚本（所有 ROS2 服务共用）
```

安装到机器人：
```bash
# 在 lingtu 仓库根目录执行
bash docs/04-deployment/services/install.sh
```
