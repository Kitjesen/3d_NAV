# WebRTC 实时视频流系统

> **阅读提示**: 本文档同时作为技术规范和实现对账表。  
> 每个特性均标注 ✅ **已实现** 或 ❌ **未实现 (TODO)**，方便快速识别当前差距。

---

## 实现状态总览

### 推流核心链路 — ✅ 已完成

```
摄像头 → ROS2 话题 → WebRTCBridge 订阅 → JPEG 编码 → BroadcastFrame
  → DataChannel.send() → P2P → Flutter onMessage → Image.memory 渲染
```

完整管线 10 个环节全部打通，可端到端工作。

### 逐项对账

| # | 特性 | 状态 | 位置 |
|---|------|------|------|
| **推流核心** | | | |
| 1 | 摄像头话题订阅 (compressed 优先, raw fallback) | ✅ | `webrtc_bridge.cpp:458-515` |
| 2 | Raw → JPEG 编码 (OpenCV imencode, quality=70) | ✅ | `webrtc_bridge.cpp:554-582, 685-710` |
| 3 | BroadcastFrame 广播到所有 Peer | ✅ | `webrtc_bridge.cpp:584-592` |
| 4 | DataChannel.send() 推送 JPEG 帧 | ✅ | `webrtc_bridge.cpp:191-206` |
| 5 | gRPC WebRTCSignaling 双向流信令 | ✅ | `data_service.cpp:2312-2446` |
| 6 | SDP Offer/Answer 交换 | ✅ | `data_service.cpp:2175-2254` |
| 7 | ICE Candidate 交换 | ✅ | `data_service.cpp:2286-2310` |
| 8 | Flutter DataChannel 接收 "video-jpeg" | ✅ | `webrtc_client.dart:321-344` |
| 9 | Image.memory + gaplessPlayback 渲染 | ✅ | `webrtc_video_widget.dart:155-169` |
| 10 | 连接状态 UI (loading/connected/failed/placeholder) | ✅ | `webrtc_video_widget.dart:149-187` |
| **信令与连接** | | | |
| 11 | 基础状态 (disconnected/connecting/connected/failed) | ✅ | `webrtc_client.dart:14-17` |
| 12 | RECONNECTING 状态 + 自动重连 | ❌ | — |
| 13 | Offer/Answer 超时处理 | ❌ | — |
| 14 | ICE 收集/连接超时 | ❌ | — |
| 15 | App 后台自动 HANGUP / 前台自动重连 | ❌ | — |
| **C++ 资源管理** | | | |
| 16 | Peer 析构释放 DataChannel + PeerConnection | ✅ | `webrtc_bridge.cpp:29-31, 228-250` |
| 17 | OpenCV imencode 异常捕获 + 跳帧 | ✅ | `webrtc_bridge.cpp:695-710` |
| 18 | BroadcastFrame 加锁 (peers_mutex_) | ✅ | `webrtc_bridge.cpp:585` |
| 19 | gRPC 流断开 → RemovePeer 清理 | ✅ | `data_service.cpp:2056-2072` |
| 20 | Bridge 初始化失败检测 (Initialize 返回 false) | ✅ | `data_service.cpp:2079-2084` |
| 21 | Bridge 为空时 fallback 占位 Answer | ✅ | `data_service.cpp:2194-2214` |
| 22 | 摄像头话题 publisher 检测 + 双订阅 fallback | ✅ | `webrtc_bridge.cpp:475-515` |
| 23 | BroadcastFrame 迭代中清除僵尸 Peer | ❌ | 有锁但不 erase 断开的 Peer |
| 24 | Peer 空闲超时 (30s 无数据自动销毁) | ❌ | 无 `last_activity_time_` |
| 25 | 零 Peer 退订摄像头 | ❌ | Peer 全断后订阅仍在 |
| 26 | 最大 Peer 数限制 | ❌ | 无 `max_peers_` 检查 |
| 27 | 过期帧跳过 (时间戳比较) | ❌ | 每帧都处理 |
| 28 | InitializeWebRTCBridge try/catch 保护 | ❌ | make_unique 抛异常会 crash |
| 29 | WebRTCSignaling RPC 入口 null bridge 检查 | ❌ | 不返回 UNAVAILABLE |
| **App 端可靠性** | | | |
| 30 | 指数退避自动重连 | ❌ | 只有手动"重试"按钮 |
| 31 | WebRTC 失败 → 降级到 gRPC Subscribe | ❌ | 无 fallback |
| **性能控制** | | | |
| 32 | 帧率节流 / 限流 | ❌ | 跟随摄像头话题频率 |
| 33 | 自适应码率 ABR | ❌ | 固定 quality=70 |
| 34 | CPU 过载自动降帧 | ❌ | — |

**统计**: ✅ 已实现 **22 项** / ❌ 未实现 **12 项**

---

## 1. 概述

本系统通过 WebRTC 将机器人摄像头画面实时传输到 Flutter App，目标端到端延迟 < 200ms。

核心特点：
- **传输方式**: JPEG 帧通过 WebRTC DataChannel（而非 H.264 视频轨道）
- **信令通道**: 复用 gRPC 双向流（`WebRTCSignaling`），无需额外信令服务器
- **多客户端**: 支持多个 App 同时观看（每个会话独立 PeerConnection）
- **条件编译**: 依赖 `libdatachannel`，未安装时自动禁用，不影响其他功能
- **故障隔离**: WebRTC 模块启动失败不影响 gRPC Gateway 其他服务

### 1.1 功能定义与系统边界

**本模块是**：实时操控视野模块 — 为远程遥控和巡检场景提供单向低延迟视觉流。

**本模块不是**：通用视频传输层、录制系统、双向音视频通话系统。

| 维度 | 定义 |
|------|------|
| **输入** | 仅依赖 ROS2 摄像头话题持续输出（`/camera/color/image_raw/compressed` 或 `/camera/color/image_raw`） |
| **输出** | 单向 JPEG 视频帧流（Nav Board → App），不提供 App → Nav Board 的视频/音频回传 |
| **与 SLAM 关系** | 无耦合。WebRTC 不读取/修改 SLAM 状态，不影响 LiDAR 点云处理 |
| **与摄像头配置关系** | 只读消费者。不主动修改摄像头参数（分辨率、帧率、曝光），由摄像头驱动节点控制 |
| **与导航关系** | 无耦合。视频流中断不触发导航停止；导航状态不影响视频流 |
| **数据通道用途** | 仅传输视频帧。**禁止**通过 DataChannel 传输控制指令、遥测数据或任何非视频负载 |

**显式不支持**：
- 音频传输（未来可扩展，见 §10）
- 视频录制（使用 rosbag 录制原始话题代替）
- 多摄像头切换（当前仅支持单一话题源）
- 端到端加密验证（依赖 WebRTC 内置 DTLS）

### 1.2 推流模型：为什么必须由机器人主动推流

WebRTC **不存在"拉流"语义**。在本系统中，机器人是媒体源的唯一拥有者，必须由机器人主动 push。

#### 技术原因

```
摄像头硬件 → 在机器人上
ROS2 话题  → 在机器人上
JPEG 编码  → 在机器人上
最新帧内容 → 只有机器人知道

→ 机器人是唯一拥有"实时帧"的一方
→ 必须由机器人 send()，App 只能 onMessage() 被动接收
```

WebRTC 的三种数据传输方式都是 push-driven：

| 传输方式 | 行为 | 拉取可能？ |
|---------|------|-----------|
| **DataChannel** (当前) | 对端 `send(bytes)` → 本端 `onMessage` | 不可能。没有 `pull()` API |
| **RTP MediaTrack** | 对端推 RTP packets → 本端 `onTrack` | 不可能。轨道由源端添加 |
| **SCTP** | 同 DataChannel 底层 | 不可能 |

#### 实际推流管线 ✅

```
1. App 发起 WebRTC 连接 (Offer)          ← 控制命令是"拉式"
2. 机器人接受连接 (Answer)
3. P2P 建立后，机器人自动开始推流          ← 视频数据是"推式"
4. 每收到一帧摄像头数据:
   bridge.BroadcastFrame(jpeg) → 遍历所有 Peer → DataChannel.send()
5. App 被动接收: onMessage → videoFrameStream → Image.memory 渲染
```

> **设计决策**: 当前实现是"连接即推流"——WebRTC 连接建立后自动开始推送，无需额外的 StartVideo 命令。断开连接即停止。这是最简单且延迟最低的方式。

#### 如果未来需要"按需推流"

可以增加 gRPC 控制命令（看似"拉"，底层仍是 push）：

```
App: StartVideo(session_id, camera_config)  → 机器人开始推流
App: StopVideo(session_id)                  → 机器人停止推流
```

当前未实现，因为"连接即推流"已满足需求。

---

## 2. 架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           Nav Board                                     │
│                                                                         │
│  ┌─────────────┐     ROS2 订阅      ┌──────────────────────────────┐   │
│  │ 摄像头节点   │──────────────────►│        WebRTCBridge           │   │
│  │ (Orbbec等)  │                    │                              │   │
│  │             │  /camera/color/    │  1. 接收 JPEG/Raw 帧         │   │
│  │ 发布:       │  image_raw/       │  2. Raw → JPEG 编码 (OpenCV) │   │
│  │ compressed  │  compressed       │  3. 广播到所有 Peer           │   │
│  └─────────────┘                    └──────────┬───────────────────┘   │
│                                                 │                       │
│                                      DataChannel│"video-jpeg"           │
│                                      (unreliable│unordered)             │
│  ┌──────────────────────────────┐              │                       │
│  │     DataServiceImpl          │              │                       │
│  │                              │◄─────────────┘                       │
│  │  WebRTCSignaling() RPC       │  回调: OnLocalDescription            │
│  │  (gRPC 双向流)               │  回调: OnLocalCandidate              │
│  │                              │  回调: OnStateChange                 │
│  └──────────────┬───────────────┘                                      │
│                 │                                                       │
└─────────────────┼───────────────────────────────────────────────────────┘
                  │ gRPC :50051
                  │ WebRTCSignaling (bidirectional stream)
                  │
                  │ SDP Offer/Answer + ICE Candidates
                  │
┌─────────────────┼───────────────────────────────────────────────────────┐
│                 ▼                                                       │
│  ┌──────────────────────────────┐     ┌────────────────────────────┐   │
│  │     WebRTCClient             │     │    WebRTCVideoWidget       │   │
│  │     (webrtc_client.dart)     │     │    (webrtc_video_widget)   │   │
│  │                              │     │                            │   │
│  │  1. 创建 PeerConnection      │────►│  1. 监听 videoFrameStream  │   │
│  │  2. gRPC 信令交换             │     │  2. Image.memory() 渲染    │   │
│  │  3. DataChannel 接收 JPEG    │     │  3. 连接状态展示            │   │
│  │  4. 发布 videoFrameStream    │     │                            │   │
│  └──────────────────────────────┘     └────────────────────────────┘   │
│                                                                         │
│                          Flutter App                                    │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 3. 信令流程 ✅

WebRTC 需要信令交换 SDP (Session Description Protocol) 和 ICE (Interactive Connectivity Establishment) 候选。本系统复用 gRPC 双向流作为信令通道：

```
Flutter App                        gRPC                        Nav Board
───────────────────────────────────────────────────────────────────────

1. ✅ App 创建 PeerConnection
   ├─ 配置 STUN/TURN
   └─ 创建 DataChannel

2. ✅ App 创建 SDP Offer
   │
   ├─ WebRTCSignaling ────────► DataServiceImpl
   │  type=OFFER               │
   │  sdp="v=0\r\n..."        │
   │                           ▼
   │                    HandleWebRTCOffer()
   │                    ├─ Bridge.HandleOffer()
   │                    │  ├─ CreatePeer()
   │                    │  ├─ SetRemoteDescription(offer)
   │                    │  └─ CreateAnswer()
   │                    │      └─ OnLocalDescription 回调
   │                    └─ Bridge.SubscribeCameraTopic()
   │
3. ✅ Nav Board 返回 SDP Answer
   │
   ◄─ WebRTCSignaling ───────── SendWebRTCSignalToSession()
      type=ANSWER
      sdp="v=0\r\n..."

4. ✅ ICE 候选交换 (双向, 多次)
   │
   ├─ type=ICE_CANDIDATE ────► HandleWebRTCIceCandidate()
   │  candidate="..."          Bridge.HandleIceCandidate()
   │
   ◄─ type=ICE_CANDIDATE ──── OnLocalCandidate 回调
      candidate="..."

5. ✅ ICE 完成
   ◄─ type=ICE_DONE ────────── OnIceGatheringDone 回调

6. ✅ P2P 连接建立 → 自动开始推流
   │
   ◄═══════════════════════════ DataChannel "video-jpeg"
      JPEG 帧数据 (unreliable, unordered)

7. ✅ 关闭
   ├─ type=HANGUP ───────────► RemoveWebRTCSession()
```

### 3.1 连接状态机与重连策略

> **实现状态**: 基础状态 ✅ | 重连逻辑 ❌ | 超时处理 ❌

WebRTC 在网络波动、WiFi 重关联、App 后台切换时极易断链。以下是目标状态机规范。

#### 状态定义

```
          connect()
    ┌──────────────────┐
    │                  ▼
┌───────┐       ┌───────────┐      P2P 建立       ┌───────────┐
│ IDLE  │       │CONNECTING │ ──────────────────►  │ CONNECTED │
│  ✅   │       │    ✅     │                      │    ✅     │
└───────┘       └─────┬─────┘                      └─────┬─────┘
    ▲                 │                                   │
    │           超时/失败                           ICE 断开/网络中断
    │                 │                                   │
    │                 ▼                                   ▼
    │          ┌───────────┐      重试次数耗尽     ┌──────────────┐
    │          │  FAILED   │ ◄──────────────────── │ RECONNECTING │
    │          │    ✅     │                       │     ❌       │
    │          └─────┬─────┘                       │ 指数退避重试  │
    │                │                             │ (1s/2s/4s/8s)│
    │           用户确认                            └──────────────┘
    │                │
    │                ▼
    │          ┌───────────┐
    └───────── │  CLOSED   │
               │    ❌     │
               └───────────┘
```

**当前实现**: Flutter 有 4 个状态 (`disconnected / connecting / connected / failed`)，对应 UI 正常工作。但缺少 `RECONNECTING` 和 `CLOSED`，断开后只能手动点"重试"按钮。

#### 各状态行为规范

| 状态 | 触发条件 | 行为 | UI 表现 | 实现 |
|------|---------|------|---------|------|
| **IDLE** | 初始 / 用户手动关闭 | 无 PeerConnection 存在 | 显示占位图 + "点击连接" | ✅ (disconnected) |
| **CONNECTING** | `connect()` 调用 | 创建 PC → Offer → 等待 Answer → ICE | 显示加载动画 | ✅ |
| **CONNECTED** | ICE connected + DataChannel open | 正常接收 JPEG 帧 | 显示视频画面 | ✅ |
| **RECONNECTING** | ICE disconnected / DataChannel 关闭 / 网络中断 | 关闭旧 PC → 指数退避重试 | 显示 "重连中..." + 最后一帧 | ❌ |
| **FAILED** | 连续重试 N 次失败 (默认 N=5) | 停止重试，释放所有资源 | 显示错误提示 + "重试" 按钮 | ✅ (但无自动重试前置) |
| **CLOSED** | 用户主动断开 / Hangup | 清理所有资源 | 回到 IDLE UI | ❌ (合并在 disconnected) |

#### 超时参数（规范值，当前均 ❌ 未实现）

| 参数 | 默认值 | 说明 | 实现 |
|------|--------|------|------|
| `offer_timeout` | 3000ms | SDP Offer/Answer 交换超时，超时进入 FAILED | ❌ |
| `ice_gathering_timeout` | 5000ms | ICE 收集超时，超时仍尝试已有候选 | ❌ |
| `ice_connection_timeout` | 10000ms | ICE 连接超时，超时触发 RECONNECTING | ❌ |
| `reconnect_max_retries` | 5 | 最大连续重试次数 | ❌ |
| `reconnect_base_delay` | 1000ms | 重试基础延迟，指数退避：1s → 2s → 4s → 8s → 16s | ❌ |
| `reconnect_max_delay` | 16000ms | 重试最大延迟上限 | ❌ |
| `peer_idle_timeout` | 30000ms | Peer 无数据超时，服务端自动回收（见 §5.4） | ❌ |

#### App 后台/前台切换（❌ 未实现）

| 事件 | 期望行为 | 实现 |
|------|---------|------|
| App 进入后台 | 发送 HANGUP，释放 PeerConnection（节省手机电量和机器人 CPU） | ❌ |
| App 回到前台 | 自动发起新连接（重新走 Offer/Answer 流程） | ❌ |
| WiFi 切换/断开 | 触发 RECONNECTING，gRPC 流断开后重建信令通道再重连 | ❌ |

---

## 4. 视频传输机制

### 4.1 为什么用 DataChannel + JPEG 而不是 H.264 视频轨道？

| 方案 | 优点 | 缺点 |
|------|------|------|
| **DataChannel + JPEG** (当前) | 实现简单；不依赖硬件编码器；每帧独立无累积延迟；丢帧不影响后续帧 | 带宽高（JPEG 压缩率低） |
| H.264 视频轨道 | 压缩率高；标准 WebRTC 播放 | 依赖 x264/硬件编码；关键帧丢失导致花屏；实现复杂 |

当前选择 JPEG DataChannel 是务实的：
- 机器人 WiFi 带宽通常 > 10Mbps，JPEG 640x480 约 30-50KB/帧 @ 15fps ≈ 4-6 Mbps
- 无关键帧依赖，丢帧后下一帧立即恢复
- `libdatachannel` 的 DataChannel 开箱即用

### 4.2 数据流细节 ✅

```
摄像头 → ROS2 话题
         │
         ├─ /camera/color/image_raw/compressed (sensor_msgs/CompressedImage)
         │  ✅ 已经是 JPEG，直接转发
         │
         └─ /camera/color/image_raw (sensor_msgs/Image, fallback)
            ✅ OpenCV cv::imencode(".jpg", quality=70) → JPEG bytes
         
         │
         ▼
✅ WebRTCBridge::BroadcastFrame(jpeg_bytes)
         │
         ├─ Peer 1 → DataChannel.send(jpeg_bytes)  → App 1
         ├─ Peer 2 → DataChannel.send(jpeg_bytes)  → App 2
         └─ Peer N → DataChannel.send(jpeg_bytes)  → App N
```

### 4.3 DataChannel 配置 ✅

```cpp
// C++ 侧
rtc::DataChannelInit config;
config.reliability.unordered = true;   // ✅ 不保序 → 减少延迟
config.reliability.maxRetransmits = 0; // ✅ 不重传 → 丢帧直接跳过
video_data_channel_ = peer_connection_->createDataChannel("video-jpeg", config);
```

```dart
// Flutter 侧 — 接收
void _onDataChannel(RTCDataChannel channel) {   // ✅
  if (channel.label == 'video-jpeg') {
    channel.onMessage = (data) {
      _frameController.add(data.binary);  // → videoFrameStream
    };
  }
}
```

### 4.4 性能限制与参数规范

> **实现状态**: 下方为产品规范值。当前代码中 **无任何帧率/Peer 数/带宽限制逻辑** ❌，完全跟随摄像头话题频率自由推送。

本节定义视频流的硬性约束，所有开发人员必须遵守。违反这些约束可能导致 SLAM/Nav 与 WebRTC Bridge 争抢 CPU，或网络拥塞影响导航控制链路。

#### 帧率与分辨率

| 参数 | 规范值 | 允许范围 | 当前实际 | 实现 |
|------|--------|---------|---------|------|
| **最大分辨率** | 640 x 480 | 320x240 ~ 1280x720 | 取决于摄像头 | ❌ 无限制 |
| **目标帧率** | 15 fps | 5 ~ 20 fps | 跟随摄像头话题 | ❌ 无节流 |
| **JPEG 质量** | 40 | 20 ~ 80 | **70** (硬编码) | ❌ 不可配置 |
| **单帧最大尺寸** | 65KB | — | 未检查 | ❌ 无限制 |

#### 带宽预算（参考值）

| 配置 | 单 Peer 带宽 | 2 Peer 带宽 | 3 Peer 带宽 |
|------|-------------|-------------|-------------|
| 640x480 @15fps JPEG70 | ~6 Mbps | ~12 Mbps | ~18 Mbps |
| 640x480 @15fps JPEG40 | ~4.5 Mbps | ~9 Mbps | ~13.5 Mbps |
| 320x240 @15fps JPEG40 | ~1.5 Mbps | ~3 Mbps | ~4.5 Mbps |

> **硬性上限**: 视频流总带宽不得超过 WiFi 可用带宽的 **60%**，剩余 40% 保留给 gRPC 控制链路 + SLAM 远程监控。典型 WiFi AP 模式带宽 ~20Mbps，即视频上限 ~12Mbps。

#### CPU 预算（参考值）

| 模块 | CPU 占用预期 | 上限 |
|------|-------------|------|
| JPEG 编码 (640x480, OpenCV) | ~5% 单核 | — |
| PeerConnection 管理 (单 Peer) | ~2% 单核 | — |
| WebRTCBridge 总计 (3 Peer) | ~12% 单核 | **< 20% 单核** |

> **规则**: 当 Nav Board 整体 CPU > 80% 时，WebRTCBridge 应自动降帧（降至 5fps）。**当前 ❌ 未实现**，需人工调整摄像头帧率。

#### 最大并发 Peer 数（❌ 未实现）

| 环境 | 最大 Peer | 理由 |
|------|----------|------|
| WiFi AP 模式 (192.168.4.x) | **3** | 带宽 ~20Mbps，每 Peer ~6Mbps |
| 有线 / 高速 WiFi | **5** | CPU 成为瓶颈 |

超过上限时，新的 Offer 应被拒绝并返回错误信号 `SESSION_LIMIT_REACHED`。**当前代码无此检查，可无限创建 Peer。**

---

## 5. 源码结构

### 5.1 C++ 服务端

| 文件 | 行数 | 说明 |
|------|------|------|
| `webrtc_bridge.hpp` | 186 | Bridge + Peer 类声明，`#ifdef WEBRTC_ENABLED` 包裹 |
| `webrtc_bridge.cpp` | 717 | 完整实现：PeerConnection 管理、摄像头订阅、帧广播 |
| `data_service.cpp` (WebRTC 部分) | ~400 | 信令 RPC 实现：Offer/Answer/ICE 处理 |

**关键类**:

```
WebRTCBridge (单例, 管理所有会话)
├── peers_: map<session_id, WebRTCPeer>
├── compressed_image_sub_: ROS2 订阅
├── raw_image_sub_: ROS2 订阅 (fallback)
├── HandleOffer(session_id, sdp) → Answer      ✅
├── HandleIceCandidate(session_id, candidate)   ✅
├── SubscribeCameraTopic(topic_name)            ✅
└── BroadcastFrame(jpeg_bytes)                  ✅

WebRTCPeer (单个 P2P 连接)
├── peer_connection_: rtc::PeerConnection
├── video_data_channel_: rtc::DataChannel "video-jpeg"
├── Initialize() → 配置回调          ✅
├── SetRemoteDescription(sdp)        ✅
├── CreateAnswer() → SDP             ✅
├── AddRemoteCandidate(candidate)    ✅
├── SendVideoFrame(jpeg_bytes)       ✅
└── Close() → 释放资源               ✅
```

### 5.2 Flutter 客户端

| 文件 | 行数 | 说明 |
|------|------|------|
| `webrtc_client.dart` | 380 | WebRTC 连接管理 + gRPC 信令 |
| `webrtc_video_widget.dart` | 314 | 视频渲染 Widget |

**关键类**:

```
WebRTCClient
├── _peerConnection: RTCPeerConnection
├── _signalingStream: gRPC bidirectional stream
├── videoFrameStream: Stream<Uint8List>   (JPEG 帧)        ✅
├── connectionStateStream: Stream<State>                    ✅
├── connect() → 创建 Offer → 信令交换                       ✅
├── disconnect() → Hangup → 清理                            ✅
├── _onDataChannel() → 接收 JPEG 帧                         ✅
├── _autoReconnect() → 指数退避重连                          ❌ 不存在
└── _onAppLifecycle() → 前后台切换处理                       ❌ 不存在

WebRTCVideoWidget (StatefulWidget)
├── 监听 videoFrameStream → Image.memory() 渲染             ✅
├── 监听 connectionStateStream → 状态展示                   ✅
├── gaplessPlayback: true (无闪烁)                          ✅
├── 手动重试按钮                                             ✅
└── 自动重连逻辑                                             ❌ 不存在
```

### 5.3 构建系统 ✅

```cmake
# CMakeLists.txt 关键部分

# ✅ 检测 libdatachannel
find_package(LibDataChannel QUIET)
if(NOT LibDataChannel_FOUND)
  pkg_check_modules(DATACHANNEL libdatachannel)
endif()
# 找到 → 定义 WEBRTC_ENABLED 宏

# ✅ 检测 x264 (可选, H.264 编码)
pkg_check_modules(X264 x264)
# 找到 → 定义 X264_ENABLED 宏

# ✅ 条件编译
if(DATACHANNEL_FOUND)
  list(APPEND GATEWAY_SOURCES src/webrtc_bridge.cpp)
  # 链接 libdatachannel
endif()
```

### 5.4 生命周期管理与资源约束

> **实现状态**: 基础生命周期 ✅ | 超时回收 ❌ | 零 Peer 退订 ❌ | 僵尸清除 ❌

WebRTCBridge 运行在嵌入式环境中，资源管理不当将导致内存泄漏、订阅堆积或僵尸连接。

#### WebRTCBridge 生命周期

```
gRPC Gateway 启动
  │
  ├─ webrtc_enabled == true?
  │   ├─ YES → InitializeWebRTCBridge()                ✅
  │   │         ├─ 配置 STUN/TURN                      ✅
  │   │         ├─ 注册回调                             ✅
  │   │         └─ 状态: READY (等待首个 Offer)         ✅
  │   │
  │   └─ NO → Bridge 不创建                            ✅
  │            WebRTCSignaling RPC 返回 UNAVAILABLE     ❌ (未做入口检查)
  │
  ├─ 首个 Offer 到达 → SubscribeCameraTopic()          ✅
  │   └─ 开始订阅摄像头话题 (仅一次, 多 Peer 共享)     ✅
  │
  ├─ 所有 Peer 断开 → 取消摄像头订阅                    ❌ (订阅永不取消)
  │
  └─ gRPC Gateway 关闭 → ~WebRTCBridge()
      ├─ 关闭所有 PeerConnection                       ✅
      ├─ 取消 ROS2 订阅                                ✅
      └─ 释放编码器资源                                 ✅
```

#### WebRTCPeer 生命周期

```
Offer 到达
  │
  ├─ CreatePeer(session_id)                            ✅
  │   ├─ new PeerConnection(ice_config)                ✅
  │   ├─ createDataChannel("video-jpeg", unreliable)   ✅
  │   └─ 检查 peers_.size() < max_peers_              ❌ (无限制)
  │
  ├─ SetRemoteDescription(offer) + CreateAnswer()      ✅
  │
  ├─ ICE 完成 + DataChannel open                       ✅
  │   └─ 开始接收 BroadcastFrame                       ✅
  │
  ├─ 正常关闭 (HANGUP / gRPC 流断开)
  │   ├─ peer_connection_->close()                     ✅
  │   ├─ 从 peers_ map 中移除                          ✅
  │   └─ shared_ptr 引用归零 → 析构                    ✅
  │
  └─ 异常关闭 (网络断开 / 超时)
      ├─ OnStateChange(FAILED) 回调触发                 ✅ (回调存在)
      ├─ 启动清理定时器 (5s)                            ❌ (无定时清理)
      └─ Peer 空闲 30s 自动移除                         ❌ (无 idle 检测)
```

#### 资源约束规则

| 规则 | 说明 | 实现 |
|------|------|------|
| **摄像头订阅共享** | 所有 Peer 共享同一份 JPEG 缓冲，不重复订阅 | ✅ |
| **Peer 析构释放** | 析构时显式 close DataChannel + PeerConnection | ✅ |
| **gRPC 流断开 = Peer 销毁** | 信令流 EOF/错误 → `RemoveWebRTCSession()` → `RemovePeer()` | ✅ |
| **BroadcastFrame 加锁** | `peers_mutex_` 保护并发安全 | ✅ |
| **BroadcastFrame 清除僵尸** | 迭代中移除 `!IsConnected()` 的 Peer | ❌ 只跳过不移除 |
| **Peer 空闲超时** | 30s 未发送帧 → 自动销毁 | ❌ 无 `last_activity` |
| **零 Peer 退订** | 最后一个 Peer 断开后取消摄像头订阅 | ❌ |
| **最大 Peer 数** | 超过上限拒绝新 Offer | ❌ |

#### 期望的 BroadcastFrame（对比当前实现）

```cpp
// 当前实现 (webrtc_bridge.cpp:584-592):
void WebRTCBridge::BroadcastFrame(const uint8_t *data, size_t size, uint64_t ts) {
    std::lock_guard<std::mutex> lock(peers_mutex_);   // ✅ 有锁
    for (auto &[id, peer] : peers_) {
        if (peer->IsConnected()) {
            peer->SendVideoFrame(data, size, ts);
        }
        // ❌ 断开的 Peer 不移除，下次还会检查
    }
    // ❌ 没有 peers_.empty() 时退订摄像头
}

// 期望实现:
void WebRTCBridge::BroadcastFrame(const uint8_t *data, size_t size, uint64_t ts) {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    for (auto it = peers_.begin(); it != peers_.end(); ) {
        if (it->second->IsConnected()) {
            it->second->SendVideoFrame(data, size, ts);
            ++it;
        } else {
            it = peers_.erase(it);  // 移除僵尸 Peer
        }
    }
    if (peers_.empty()) {
        UnsubscribeCameraTopic();  // 零 Peer 退订
    }
}
```

---

## 6. 配置参数

### 6.1 grpc_gateway.yaml ✅

```yaml
# WebRTC 开关
webrtc_enabled: false                    # 改为 true 启用
webrtc_offer_timeout_ms: 3000           # SDP 交换超时 (配置存在, 但代码未使用超时逻辑 ❌)

# 旧接口 (StartCamera/StopCamera, 已废弃)
webrtc_start_command: ""
webrtc_stop_command: ""
```

### 6.2 STUN/TURN 服务器 ✅

```cpp
// ✅ 默认 STUN (Google 公共服务器)
rtc::IceServer stunServer("stun:stun.l.google.com:19302");

// 可选: 配置 TURN (NAT 穿透)
// 当机器人和 App 不在同一网段时需要
```

> **注意**: 机器人 WiFi AP 模式下 (192.168.4.1)，App 直连机器人，通常不需要 STUN/TURN。

### 6.3 运行失败容忍与降级行为

WebRTC 是**可选增强功能**，其启动失败**绝不允许**影响 gRPC Gateway 的核心服务。

#### 启动阶段失败

| 失败场景 | 期望行为 | 实现 |
|---------|---------|------|
| `WEBRTC_ENABLED` 未定义 | Bridge 代码不编译；RPC 返回 `UNIMPLEMENTED` | ✅ 条件编译 |
| `webrtc_enabled: false` | Bridge 不初始化 | ✅ |
| `Initialize()` 返回 false | `webrtc_bridge_.reset()`，继续运行 | ✅ |
| `InitializeWebRTCBridge()` 抛异常 | `try/catch` 捕获，`bridge_ = nullptr` | ❌ **无 try/catch** |
| STUN 服务器不可达 | Bridge 正常创建，ICE 阶段可能失败 | ✅ 不阻塞启动 |

> **风险**: `std::make_unique<WebRTCBridge>()` 如果构造函数抛异常（如 libdatachannel 内部错误），当前代码会 **crash 整个 Gateway**。

#### 运行阶段失败

| 失败场景 | 行为 | 实现 |
|---------|------|------|
| 摄像头话题无数据 | Bridge 正常运行但不广播帧 | ✅ 订阅等待 |
| DataChannel send 失败 | 忽略（unreliable），继续下一帧 | ✅ |
| PeerConnection 崩溃 | 移除该 Peer，不影响其他 | ✅ |
| OpenCV imencode 失败 | 跳过该帧，记录 WARN | ✅ |
| 摄像头话题延迟 > 500ms | 跳过旧帧，只发送最新帧 | ❌ 每帧都发 |

#### 降级策略（❌ 未实现）

当 WebRTC 不可用时，App 应自动降级到 gRPC Subscribe 模式：

```
优先: WebRTCSignaling (P2P, 低延迟, 15fps)
  │
  └─ 不可用 → 降级: Subscribe("/camera/...", 1fps)
                      (gRPC 服务端流, 高延迟但可靠)
```

```dart
// ❌ 未实现 — 期望代码:
try {
  await webrtcClient.connect();
} catch (e) {
  log.warn("WebRTC unavailable, falling back to gRPC Subscribe");
  await _startGrpcVideoFallback();
}
```

---

## 7. 启用步骤

### 7.1 安装依赖

```bash
# 方式 A: 系统包 (Ubuntu 22.04)
sudo apt install libdatachannel-dev

# 方式 B: 源码编译
git clone https://github.com/nicknsy/libdatachannel.git
cd libdatachannel
git submodule update --init --recursive
cmake -B build -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j$(nproc)
sudo cmake --install build

# 可选: H.264 编码 (当前未使用, 未来优化)
sudo apt install libx264-dev
```

### 7.2 编译

```bash
cd /home/sunrise/data/SLAM/navigation

colcon build --packages-select remote_monitoring \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 验证: 编译日志应含 "Found libdatachannel" + "WebRTC support: ENABLED"
```

### 7.3 配置

```bash
# 修改 src/remote_monitoring/config/grpc_gateway.yaml
# webrtc_enabled: true
```

### 7.4 运行

```bash
# 1. 启动摄像头
ros2 launch orbbec_camera gemini_330_series.launch.py

# 2. 启动 gRPC Gateway (含 WebRTC Bridge)
ros2 launch remote_monitoring grpc_gateway.launch.py

# 3. 验证摄像头话题
ros2 topic hz /camera/color/image_raw/compressed
# 应看到 ~15-30 Hz

# 4. 打开 Flutter App → 控制页面 → 视频画面应自动显示
```

### 7.5 验证与验收标准

#### 基础连通性（可立即验证）

| # | 检查项 | 通过标准 | 实现 |
|---|--------|---------|------|
| 1 | 编译标志 | 日志含 `WebRTC support: ENABLED` | ✅ 可验证 |
| 2 | 库链接 | `ldd grpc_gateway \| grep datachannel` 有输出 | ✅ 可验证 |
| 3 | 配置加载 | 日志含 `WebRTC Bridge initialized` | ✅ 可验证 |
| 4 | 摄像头话题 | `ros2 topic hz` > 10Hz | ✅ 可验证 |
| 5 | gRPC 信令 | App Offer 后 3s 内收到 Answer | ✅ 可验证 |
| 6 | DataChannel | "video-jpeg" 通道 open | ✅ 可验证 |
| 7 | 首帧渲染 | 连接后 2s 内显示画面 | ✅ 可验证 |

#### 可靠性（依赖未实现功能）

| # | 检查项 | 通过标准 | 实现 |
|---|--------|---------|------|
| 8 | App 断开重连 | 杀 App 后重开，5s 恢复 | ❌ 需自动重连 |
| 9 | WiFi 断连恢复 | WiFi 关 5s 后开，15s 恢复 | ❌ 需 RECONNECTING |
| 10 | Gateway 无 crash | Peer 异常关闭不导致 crash | ✅ 可验证 |
| 11 | 5% 丢包环境 | 帧率 > 10fps | ✅ 可验证 (DataChannel unreliable) |
| 12 | 内存稳定 | 5 分钟 RSS 增长 < 10MB | ❌ 需僵尸 Peer 清理 |

---

## 8. 三种视频接口对比

| 接口 | 类型 | 状态 | 延迟 | 帧率 | 适用场景 |
|------|------|------|------|------|---------|
| **WebRTCSignaling** | gRPC 双向流 + WebRTC P2P | **推荐** ✅ | < 200ms | 15fps | 实时操控、遥控巡检 |
| **Subscribe (camera)** | gRPC 服务端流 | 可用 ✅ | 500ms~2s | 1-5fps | 后台低频监控、缩略图 |
| StartCamera/StopCamera | 旧接口, 外部脚本 | **废弃** | — | — | 不建议使用 |

---

## 9. 故障排查

### 9.1 服务端排查

#### 视频不显示

```bash
# 1. WebRTC 是否启用
grep webrtc_enabled src/remote_monitoring/config/grpc_gateway.yaml

# 2. libdatachannel 是否编译进去
ldd install/remote_monitoring/lib/remote_monitoring/grpc_gateway | grep datachannel

# 3. 摄像头话题是否有数据
ros2 topic hz /camera/color/image_raw/compressed

# 4. Gateway 日志搜索 WebRTC
ros2 topic echo /rosout --filter "WebRTC"
```

#### 延迟高

| 原因 | 诊断 | 解决 |
|------|------|------|
| WiFi 带宽不足 | `iperf3 -c 192.168.4.1` < 10Mbps | 降低分辨率 |
| 使用了 Raw 话题 (CPU 编码) | 日志含 "Subscribing to raw image" | 确认 compressed 话题 |
| JPEG 质量过高 (当前 70) | 单帧 > 65KB | 需改代码降低 quality |
| 多 Peer 带宽竞争 | N > 2 且延迟增长 | 减少连接数 |

#### ICE 连接失败

```bash
ping 192.168.4.1   # 确认网络连通
# WiFi AP 模式: App 必须连机器人热点 (192.168.4.x)
# 路由器模式: 需 STUN, 可能需 TURN
```

### 9.2 客户端 (Flutter App) 排查

| 症状 | 可能原因 | 排查 |
|------|---------|------|
| Offer 后无响应 | gRPC 连接失败 | 检查 `192.168.4.1:50051` 可达性 |
| 收到 Answer 但 ICE 失败 | NAT/防火墙 | 确认同一网段 |
| DataChannel 不 open | SDP 不匹配 | 检查 libdatachannel 版本 |
| 帧接收但黑屏 | JPEG 解码失败 | 检查首字节 `0xFF 0xD8` |
| 画面卡顿 | 手机性能不足 | 减少分辨率 |

#### 网络权限

```
Android: INTERNET + ACCESS_NETWORK_STATE
iOS: NSAllowsArbitraryLoads: YES (机器人 AP 非 HTTPS)
```

### 9.3 多会话冲突排查

| 场景 | 症状 | 解决 |
|------|------|------|
| 同手机连两次 | 旧连接停 | App 应先 disconnect |
| 超多台设备 | 带宽耗尽 | ❌ 当前无 max_peers 限制 |
| 一台断后其他卡 | 僵尸 Peer 锁竞争 | ❌ 需清理僵尸 Peer |
| Gateway 重启后 App 挂 | gRPC 流已断 | ❌ App 需自动重连 |

---

## 10. 未来优化方向

| 方向 | 优先级 | 预计收益 | 状态 |
|------|--------|---------|------|
| **C++ try/catch 保护 + RPC 入口检查** | **紧急** | 防止 Gateway crash | ❌ |
| **BroadcastFrame 僵尸清除 + 零 Peer 退订** | **高** | 防内存泄漏 | ❌ |
| **最大 Peer 数限制** | **高** | 防带宽耗尽 | ❌ |
| **Flutter 自动重连 + 指数退避** | **高** | 弱网体验 | ❌ |
| **Offer/Answer/ICE 超时** | **高** | 防卡死 | ❌ |
| **App 前后台生命周期** | 中 | 省电省 CPU | ❌ |
| **JPEG quality 可配置化** | 中 | 灵活调参 | ❌ |
| **自适应码率 (ABR)** | 中 | 弱网下体验 3x | ❌ |
| **WebRTC 失败 → gRPC Subscribe 降级** | 中 | 可靠性兜底 | ❌ |
| **Peer 空闲超时 (30s)** | 中 | 防僵尸连接 | ❌ |
| **过期帧跳过** | 低 | 降延迟 | ❌ |
| H.264 视频轨道 | 低 | 带宽降 3-5x | ❌ |
| 硬件编码 (RK3588) | 低 | CPU < 1% | ❌ |
| 音频 (Opus) | 低 | 远程协助 | ❌ |
| 多摄像头切换 | 低 | 功能扩展 | ❌ |
| 废弃 StartCamera/StopCamera | 低 | 清理技术债 | ❌ |

### 自适应码率 (ABR) 设计方向

```
带宽充足 (> 8Mbps)  → 640x480 @15fps JPEG-Q50  (高质量模式)
带宽中等 (3~8Mbps)  → 480x360 @12fps JPEG-Q35  (平衡模式)
带宽紧张 (< 3Mbps)  → 320x240 @8fps  JPEG-Q25  (省流模式)
```

---

## 11. 相关文件索引

| 文件 | 说明 |
|------|------|
| `src/remote_monitoring/include/remote_monitoring/webrtc_bridge.hpp` | Bridge + Peer 类声明 |
| `src/remote_monitoring/src/webrtc_bridge.cpp` | WebRTC 核心实现 (717 行) |
| `src/remote_monitoring/src/services/data_service.cpp` | 信令 RPC 实现 |
| `src/remote_monitoring/config/grpc_gateway.yaml` | 配置 (webrtc_enabled 等) |
| `src/remote_monitoring/CMakeLists.txt` | 条件编译 (libdatachannel 检测) |
| `client/flutter_monitor/lib/features/control/webrtc_client.dart` | Flutter WebRTC 客户端 |
| `client/flutter_monitor/lib/features/control/webrtc_video_widget.dart` | 视频渲染 Widget |
| `src/robot_proto/proto/data.proto` | WebRTCSignaling RPC + 消息定义 |

---

## 12. 参考文档

| 文档 | 关联 |
|------|------|
| [ARCHITECTURE.md](ARCHITECTURE.md) | 双板架构、gRPC Gateway 总体设计 |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | 通信故障通用排查 |
| [BUILD_GUIDE.md](BUILD_GUIDE.md) | 编译环境和依赖安装 |
| [CHANGELOG.md](CHANGELOG.md) | 版本记录 |

---

*最后更新: 2026-02-08*
