# WebRTC 相机流 — 部署与实机测量

相对 JPEG-over-WebSocket 的 ~300 ms glass-to-glass，WebRTC + H.264 把相机
端到端延迟压到 80–150 ms（LAN，软编）。本文档记录如何在 S100P 上部署、
如何量化延迟、以及 Phase B（硬编）的路线图。

## 1. 最小部署

```bash
# S100P 上
pip install 'aiortc>=1.14' 'av>=13'

# 首次跑：
python lingtu.py s100p
```

启动日志应当看到：

```
WebRTCStreamModule: ... enabled
GatewayModule: WebRTC signalling mounted at POST /api/v1/webrtc/offer
```

浏览器访问 `http://<robot-ip>:5050/`，`CameraFeed` 右下角 hint 会显示
`H.264 · 30fps · 1.20 Mbit/s · 8ms enc`。如果显示 `MJPEG`，说明 aiortc
未加载或 `/api/v1/webrtc/offer` 返回 503，按 §4 排查。

## 2. 可调参数

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `LINGTU_WEBRTC_BITRATE` | `2500000` (2.5 Mbit/s) | 视频流 RTP 上限。弱网或 Wi-Fi 2.4G 建议 800000–1500000。 |

未来的参数（Phase B 加）：`LINGTU_WEBRTC_H264_TOPIC`（指向
`hobot_codec` 的共享内存 H.264 话题，开启硬编直通）。

## 3. 实机量化延迟

### 3.1 Chrome 内置 WebRTC 诊断（首选）

打开 `chrome://webrtc-internals/`，找到活跃的 `RTCPeerConnection`，看：

- `currentRoundTripTime` — 网络 RTT，LAN 应 <10 ms
- `jitterBufferDelay / jitterBufferEmittedCount` — 浏览器侧抖动缓冲
- `framesDecoded`, `framesDropped` — 解码健康度
- `freezeCount` — 冻结次数（非零说明网络/编码打嗝）

端到端延迟 ≈ `RTT/2 + encode_avg_ms + jitterBufferDelay + decode (~5ms)`。
典型 LAN 组成：`5 + 10 + 30 + 5 = ~50 ms`（一路顺的话）。

### 3.2 肉眼 + 秒表法（最直观）

在屏幕上开一个高精度毫秒时钟（`https://time.is/ms`），把手机相机对准
时钟，机器人摄像头对准手机屏幕；让浏览器回显画面与原时钟并排，相机
快速拍下两者 → 差值就是 glass-to-glass 延迟。重复 5–10 次取均值。

### 3.3 编码/网络分解

```bash
# 在机器人 S100P 上查一下当前聚合指标
curl -s http://localhost:5050/api/v1/webrtc/stats | jq
```

关键字段：

- `encode_avg_ms` — 编码平均耗时/帧。软编 libx264 在 A78AE 上：
  - 720p30 ≈ 5–10 ms（正常）
  - 持续 >15 ms 说明 CPU 打满，降码率或切 Phase B
- `bitrate_bps` — 实际上行码率，应当略低于 `max_bitrate`
- `fps` — 实际编码帧率，应接近 30（如果 source 是 30 Hz）
- `peers[].packets_lost` — 累积丢包，LAN 应当近 0

### 3.4 源头 → 消费者时间戳（精确）

如果需要严谨数字，在 `CameraBridgeModule` 接 image callback 时把毫秒
时间戳 LSB 编入画面（例如画一个二进制条码），浏览器 canvas 逐帧读
时间戳并与 `performance.now()` 做差。此法可绕过所有人眼估计。

## 4. 出问题怎么办

| 现象 | 原因 | 解决 |
|------|------|------|
| 右下角显示 `MJPEG` 而非 `H.264` | `/offer` 返回 503 | 检查 aiortc 装了没：`python -c 'import aiortc; print(aiortc.__version__)'` |
| 视频区一直"等待画面帧…" | `color_image` 端口未接线 / CameraBridge 未启动 | `curl /api/v1/health`，看 `WebRTCStreamModule.has_frame` |
| iOS Safari 黑屏但 Chrome 正常 | SDP 里 VP8 排在 H.264 前面 | 已做 `_force_h264_first`，若仍复现请抓 `chrome://webrtc-internals` 的 SDP 对比 |
| 偶发冻结 / 长尾卡顿 | 网络抖动 / 编码打嗝 | 降 `LINGTU_WEBRTC_BITRATE`；或看 `encode_avg_ms` 是否飙高 |
| `ICE failed` 瞬间断线 | 浏览器与机器人走了非同网段 IP | 确保 host candidate 双方可达；LAN 场景禁用 VPN |

## 5. Phase B 路线（S100P Nash BPU 硬编）

当前软编在 A78AE 上占 ~30 % 单核，持续跑有热节流风险。Phase B 方案：

1. 在 S100P 上起 `hobot_codec` systemd 服务，输入 `/hbmem_img` NV12，
   输出 `/image_h264`（共享内存话题）。Horizon 官方仓：
   <https://github.com/D-Robotics/hobot_codec>
2. 给 `WebRTCStreamModule` 加"预编码直通"模式：订阅 `/image_h264`，
   把 NAL 单元直接喂给 aiortc 的 `H264PayloadContext`，跳过 libx264。
   改动范围：~80 行，协议/前端 0 修改。
3. 预期收益：`encode_avg_ms` 从 5–10 ms 降到 <1 ms，CPU 占用 30% → <5%，
   持续温度曲线明显改善。

如果 Phase B 后还不够快（需要 <50 ms），再考虑换 pion/str0m 原生 WebRTC
sidecar；纯 Python aiortc 在 asyncio + SRTP 层还有 ~10–15 ms overhead
是物理上限。

## 6. 代码结构索引

| 文件 | 作用 |
|------|------|
| `webrtc_stream_module.py` | 模块主体：In[Image] → H.264 → aiortc MediaStreamTrack |
| `webrtc_stream_module.py::_force_h264_first` | SDP 重排（iOS Safari 兼容） |
| `webrtc_stream_module.py::collect_stats` | getStats 聚合 + 码率增量计算 |
| `src/gateway/gateway_module.py::post_webrtc_offer` | SDP offer/answer 信令 |
| `src/gateway/gateway_module.py::get_webrtc_stats` | 实机监控接口 |
| `web/src/hooks/useWebRTC.ts` | 浏览器侧 PeerConnection + 1Hz stats 轮询 |
| `web/src/components/CameraFeed.tsx` | `<video>` + HUD + JPEG fallback |
