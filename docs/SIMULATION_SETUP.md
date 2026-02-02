# 机器狗仿真测试搭建指南

在仿真里复现「Livox + FastLIO + Localizer + PCT 规划」整条链路，大致需要以下部分。

---

## 1. 需要准备的内容概览

| 模块 | 作用 | 你当前栈的依赖 |
|------|------|----------------|
| **URDF / xacro** | 机器狗模型、连杆、关节、传感器连杆 | 定义 `body`、`base_link`、`lidar_link`、`imu_link` 等 |
| **仿真环境** | 物理/世界、时间、碰撞 | Gazebo / Ignition 或 Isaac Sim 等 |
| **传感器仿真** | 提供点云、IMU 话题 | Livox CustomMsg + Imu（与 FastLIO 一致） |
| **TF** | 各 frame 关系 | odom→body、map→odom（Localizer）或 fake_localization |
| **控制器/底盘** | 把 cmd_vel 或关节指令作用到仿真 | 与 path_follower / robot_driver 对接 |

---

## 2. URDF / xacro（机器人描述）

- **必须有的 link**（和当前栈一致）：
  - `base_link` 或 `body`：机体质心/底盘，规划器查的是 **map → body**。
  - `lidar_link`：Livox 安装位置，点云和 IMU 的 frame_id 一般挂在这或 body。
  - `imu_link`（可选）：若 IMU 单独一个 link，需在 URDF 里写好与 body 的固定关系。
- **可选**：四足关节、足端 link（若仿真里要控制腿）。
- **建议**：用 **xacro** 写，便于参数化（轮距、传感器位姿等），再生成 urdf 给仿真和 `robot_state_publisher` 用。

示例结构（仅作参考）：

```xml
<!-- 示例：body + lidar_link + imu_link -->
<link name="body"/>
<link name="lidar_link"/>
<link name="imu_link"/>
<joint name="body_to_lidar" type="fixed"> ... </joint>
<joint name="body_to_imu" type="fixed"> ... </joint>
```

- 实际尺寸、质量、惯性、外观（mesh）按你真机或设计图来填。

---

## 3. 仿真环境选择（不必非用 Gazebo）

| 方案 | 特点 | 适合场景 | 备注 |
|------|------|----------|------|
| **Gazebo (Ignition/Fortress)** | 开源、ROS2 官方常用、文档多；Lidar/IMU 插件现成 | 本机跑、中等场景、不想上显卡 | `ros_gz_*` 桥接，Livox 需插件或 PointCloud2→CustomMsg 转换 |
| **Isaac Sim** | NVIDIA、高保真、Livox/多传感器、ROS2 支持好 | 要逼真、有 NVIDIA 显卡、做算法/真机复现 | 资源占用大，学习曲线陡 |
| **MuJoCo + Isaac Lab** | 轻量、物理准、四足/强化学习常用 | 四足控制、RL、不需要复杂场景 | 和 ROS2 要自己桥接或走 Isaac Lab |
| **Webots** | 跨平台、自带多种传感器、ROS2 有官方接口 | 快速原型、多机器人、不想折腾 Gazebo | 有 Lidar/IMu，Livox 需转或替代 |
| **CoppeliaSim (formerly V-REP)** | 脚本/API 丰富、多机器人 | 研究、复杂机械结构 | ROS2 有社区/第三方桥接 |
| **仅 RViz + fake_localization** | 无物理、无传感器仿真 | **只测规划、TF、路径显示** | 你已有 planner_only_launch，零额外依赖 |
| **ros2 bag 回放** | 真机数据、真实点云/IMU | 测 Localizer、FastLIO、规划闭环 | 不仿真物理，只复现传感器流 |

**推荐取舍**（按你当前需求）：

- **只验证规划/路径**：继续用 **RViz + planner_only_launch** 或 **bag 回放**，不必上 Gazebo。
- **要带物理的仿真、本机跑、开源**：**Gazebo Fortress** 或 **Webots**。
- **要高保真、有 NVIDIA 显卡、愿意配环境**：**Isaac Sim**。
- **专注四足控制/RL、轻量**：**MuJoCo / Isaac Lab**（再考虑和 ROS2 的桥接）。

---

## 4. 传感器配置（和 FastLIO 对齐）

FastLIO2 当前接口（你仓库里）：

- **IMU**：`sensor_msgs/Imu`，话题名由 `lio.yaml` 配置（如 `/imu/data` 或 `/livox/imu`）。
- **Lidar**：`livox_ros_driver2/CustomMsg`，话题名由配置（如 `/livox/lidar`）。

因此仿真里需要：

- **IMU 仿真**  
  - Gazebo：`libgazebo_ros_imu_sensor.so` 等 IMU 插件，发布 `sensor_msgs/Imu` 到指定话题。  
  - 在 lio.yaml 里把 `imu_topic` 指到该话题。
- **Lidar 仿真（难点）**  
  - 方案 A：用 **Livox 官方或社区仿真**（若有 Gazebo/Isaac 的 Livox 插件），直接发 `CustomMsg`，话题名与真机一致。  
  - 方案 B：用 Gazebo 的 **通用 Lidar** 插件发 `sensor_msgs/PointCloud2`，再写一个 **转换节点**：PointCloud2 → `livox_ros_driver2/CustomMsg`（按 CustomMsg 的字段填），这样 FastLIO 不用改。  
  - 方案 C：改 FastLIO 或换一个支持 PointCloud2 的 LIO 节点，仿真只发 PointCloud2（改动大，不优先）。

建议：先确认你更倾向 Gazebo 还是 Isaac，再选「Livox 插件」或「Gazebo Lidar + CustomMsg 转换节点」。传感器在 URDF 里挂在 `lidar_link`，仿真里该 link 的位姿要和 TF 一致（由 `robot_state_publisher` + 仿真引擎共同维护）。

---

## 5. TF 与定位（和当前 launch 一致）

- **仿真里**：  
  - 仿真器通常提供 **odom → base_link（或 body）**。  
  - **map → odom** 仍由 **Localizer** 或 **fake_localization** 提供，和真机一致。  
- 因此：
  - 若只测规划：仿真里只起 **fake_localization** + **global_planner** 即可（与 planner_only_launch 一致）。  
  - 若测「定位 + 规划」：仿真里起 **FastLIO + Localizer**，再起 **global_planner**；仿真器提供 odom→body，Localizer 提供 map→odom。

URDF 里 `body` / `base_link` 的命名要和 FastLIO、Localizer、global_planner 的 `robot_frame`、`body_frame` 一致（你当前是 `body`）。

---

## 6. 控制器与底盘（若仿真里要“走起来”）

- path_follower 一般发布 **cmd_vel** 或关节目标。  
- 仿真里需要：  
  - **Gazebo**：差速/阿克曼的 `gazebo_ros_diff_drive` 等，或四足用关节控制器（如 effort_controllers）。  
  - 把 cmd_vel 或关节指令接到仿真里的对应 joint。

若当前只测「规划 + 定位」，可先不接真实运动，只在 RViz 里看路径；要测跟踪再接控制器。

---

## 7. 推荐步骤（分阶段）

1. **阶段 1（已有）**  
   - 用 **planner_only_launch**（fake_localization + global_planner）在 RViz 里点 2D Pose / 2D Goal 测规划。  
   - 确认 map、body、路径、tomogram 都正常。

2. **阶段 2（仅加机器人外形）**  
   - 写机器狗 **URDF/xacro**（body、lidar_link、imu_link）。  
   - 用 `robot_state_publisher` + RViz 显示模型，TF 仍用 fake_localization。  
   - 不接仿真器，只确认模型和 TF 正确。

3. **阶段 3（带物理的仿真）**  
   - 选 Gazebo 或 Isaac，建简单世界。  
   - 在 URDF 里为 Gazebo/Isaac 加 **Lidar 与 IMU 插件**（或 Isaac 的 Livox 等），发布与 FastLIO 一致的话题类型和名字。  
   - 仿真里起 FastLIO + Localizer（或继续用 fake_localization）+ global_planner，确认点云、IMU、TF、规划都通。

4. **阶段 4（可选）**  
   - 接 path_follower + 仿真底盘/关节控制，测跟踪与避障。

---

## 8. 小结清单

- [ ] **URDF/xacro**：body、lidar_link、imu_link，尺寸与质量合理。  
- [ ] **仿真环境**：Gazebo 或 Isaac，简单世界。  
- [ ] **IMU 仿真**：发布 `sensor_msgs/Imu`，话题与 lio.yaml 一致。  
- [ ] **Lidar 仿真**：Livox 插件 或 Gazebo Lidar + PointCloud2→CustomMsg 转换。  
- [ ] **TF**：odom→body 由仿真提供，map→odom 由 Localizer 或 fake_localization 提供。  
- [ ] **控制器**（可选）：cmd_vel 或关节控制接到仿真。

按上面顺序做，可以先在 RViz 里把「规划 + 定位」测稳，再逐步加上仿真和传感器，最后再接底盘控制。

---

## 9. MuJoCo 与本栈的集成

MuJoCo 不直接支持 ROS2，需要一层「桥接节点」：跑 MuJoCo 仿真循环，把 body 位姿、关节状态、IMU、点云按你栈的接口发出去。整体数据流如下。

### 9.1 数据流（与当前栈对齐）

```
MuJoCo 仿真循环
    → 发布 odom → body 的 TF
    → 发布 /Odometry (nav_msgs/Odometry，frame_id=odom, child_frame_id=body)
    → 发布 /imu/data 或 /livox/imu (sensor_msgs/Imu，frame_id=body 或 imu_link)
    → 发布 点云：/livox/lidar (livox_ros_driver2/CustomMsg) 或 /cloud_registered (PointCloud2)
         ↓
FastLIO2 (imu_topic + lidar_topic) → /Odometry, /cloud_registered
         ↓
Localizer（或 fake_localization）→ map → odom
         ↓
PCT global_planner（查 map→body）→ /pct_path
```

要点：MuJoCo 侧只需提供 **odom→body**、**/Odometry**、**IMU**、**点云**；map 仍由 Localizer 或 fake_localization 提供，和真机一致。

### 9.2 可选实现方式

| 方式 | 作用 | 说明 |
|------|------|------|
| **mujoco_ros2** | MuJoCo ↔ ROS2 通用桥接 | 发布 JointState，订阅关节控制（Float64MultiArray）；不直接带 IMU/Lidar，需自己从 MuJoCo 读传感器再发 ROS2 话题。 |
| **MuJoCo-LiDAR** | 在 MuJoCo 里做 Lidar 射线 | 支持 Livox 等模型，GPU 加速，可出点云；需和 mujoco_ros2 或自写循环一起用，再发 CustomMsg 或 PointCloud2。 |
| **Spot-MuJoCo-ROS2 / Atlas-MuJoCo-ROS2** | 四足/人形 + MuJoCo + ROS2 | 含 IMU、odom、JointState 发布，可参考其 MuJoCoMessageHandler 把 IMU/odom 接到你栈。 |
| **自写 Python/C++ 桥接** | 完全按你栈接口来 | 用 MuJoCo Python 或 C API 步进仿真，每步发布 TF、/Odometry、Imu、点云（或转 CustomMsg）。 |

推荐：用 **mujoco_ros2** 或 **Spot-MuJoCo-ROS2** 做框架，在其上补：  
1）从 MuJoCo 读 IMU 并发布 `sensor_msgs/Imu`；  
2）用 **MuJoCo-LiDAR** 或 MuJoCo 的 ray 传感器得到点云，再发 `PointCloud2` 或写一个小节点转成 `livox_ros_driver2/CustomMsg`（这样 FastLIO 不用改）。

### 9.3 集成步骤（最小可跑）

1. **安装 MuJoCo + ROS2 桥接**
   - 安装 MuJoCo（系统或 conda，版本 ≥2.1）。
   - 克隆并编译 [mujoco_ros2](https://github.com/woolfrey/mujoco_ros2)（或 [Spot-MuJoCo-ROS2](https://github.com/MindSpaceInc/Spot-MuJoCo-ROS2)）到你的 ROS2 workspace。
   - 若有 Livox 点云需求：参考 [MuJoCo-LiDAR](https://github.com/TATP-233/MuJoCo-LiDAR) 的接口，在桥接里调用或单独节点转成 ROS2 话题。

2. **准备机器狗 MuJoCo 模型（XML）**
   - 在 MJCF 里定义 `body`（或与 body 对应的 freejoint 的 body）、`imu_link`、`lidar_link`（或 sensor site）。
   - 为 IMU：在 MuJoCo 里加 `imu` 传感器，绑在 body 上；桥接里每步读 `data.sensordata` 对应 IMU，填 `sensor_msgs/Imu` 发布。
   - 为 Lidar：用 MuJoCo-LiDAR 或 MuJoCo 的 ray sensor 得到点云，frame_id 设为 body 或 lidar_link。

3. **桥接节点要发布的内容（与你栈对齐）**
   - **TF**：`odom` → `body`（每步用当前 body 位姿发布，或从 freejoint 读出）。
   - **/Odometry**：`nav_msgs/Odometry`，`header.frame_id=odom`，`child_frame_id=body`，pose 与 TF 一致。
   - **IMU**：`sensor_msgs/Imu`，`header.frame_id=body`（或 imu_link），话题与 `lio.yaml` 里 `imu_topic` 一致（如 `/imu/data` 或 `/livox/imu`）。
   - **点云**：要么直接发 `livox_ros_driver2/CustomMsg` 到 `lio.yaml` 的 `lidar_topic`，要么发 `PointCloud2` 到 `/cloud_registered` 并加一层 PointCloud2→CustomMsg 转换（若 FastLIO 只吃 CustomMsg）。

4. **Launch 组合**
   - 启动 MuJoCo 仿真节点（加载机器狗 XML，发布上述 TF + 话题）。
   - 若只测规划：再起 **fake_localization** + **global_planner**（与 planner_only_launch 一致），不一定要起 FastLIO/Localizer。
   - 若测完整链路：起 **FastLIO**（imu_topic + lidar_topic 指向 MuJoCo 发布的话题）+ **Localizer**（或先 fake_localization）+ **global_planner**；保证 map→odom 由 Localizer/fake 提供，odom→body 由 MuJoCo 桥接提供。

5. **控制（可选）**
   - 若要让狗在 MuJoCo 里动：订阅 path_follower 的 cmd_vel 或关节目标，在桥接里转成 MuJoCo 的 actuator 控制（或参考 mujoco_ros2 的 Float64MultiArray 接口）。

### 9.4 参考仓库

- [mujoco_ros2](https://github.com/woolfrey/mujoco_ros2)：MuJoCo 与 ROS2 的通用通信。
- [MuJoCo-LiDAR](https://github.com/TATP-233/MuJoCo-LiDAR)：MuJoCo 内 Lidar 仿真（含 Livox），可对接 ROS2。
- [Spot-MuJoCo-ROS2](https://github.com/MindSpaceInc/Spot-MuJoCo-ROS2)：Spot 四足 + MuJoCo + ROS2，含 IMU/odom 发布，可参考其消息格式和 TF 结构。
- [Mujoco_rviz2_Integration](https://github.com/YunhaoTsao10/Mujoco_rviz2_Integration)：MuJoCo + RViz2 可视化，便于调试 TF 与话题。

按上述顺序：先让 MuJoCo 只发布 TF + /Odometry + IMU（不接 Lidar），用 fake_localization + global_planner 测通规划；再接 Lidar/点云 和 FastLIO/Localizer，即可把 MuJoCo 和当前仿真栈完整集成。
