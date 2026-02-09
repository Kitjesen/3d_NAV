# OTA v3 增强 — 完整开发报告

> 日期: 2026-02-09  
> 版本: v1.1.0  
> 作者: Navigation Team

---

## 1. 背景与目标

### 1.1 问题

OTA v2 实现了基本的更新链路（下载 → 校验 → 安装 → 回滚），但在生产环境中暴露出以下不足：

| 编号 | 问题 | 影响 |
|------|------|------|
| P1 | 无全局版本快照 | `installed_manifest.json` 只记录增量安装的制品，无法回答"机器人整体是什么版本" |
| P2 | 设备端不验签 | manifest 签名仅在构建侧生成，设备端未校验，中间人可篡改 |
| P3 | 安装后无健康检查 | 更新成功即视为完成，不确认系统是否真正可用 |
| P4 | 版本比较用字符串 | `"1.9.0" > "1.10.0"` 导致版本降级误判 |
| P5 | 升级历史不持久 | 重启后丢失历史记录，无法审计 |
| P6 | 失败原因不透明 | `OtaStatus` 只有 FAILED，不区分网络/签名/磁盘/权限等具体原因 |
| P7 | App 无法校验版本一致性 | Flutter 端只能看到"已安装列表"，无法确认设备实际状态 |

### 1.2 目标

构建一个**可观测、可审计、安全可信**的 OTA 系统，覆盖：

```
版本资产 + 设备画像 + 安全凭证 + 发布策略 + 可观测性与回滚
```

### 1.3 约束

- **App-centric 控制面**: 所有 OTA 操作由 Flutter App 发起，机器人不主动拉取
- **版本一致性优先**: 任何安装/回滚操作必须原子更新所有版本状态文件
- **兼容性**: 不破坏 v2 已有的 manifest 格式和 RPC 接口

---

## 2. 架构设计

### 2.1 组件关系

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Flutter App (控制面)                          │
│                                                                     │
│  CloudOtaService ──▶ GitHub Releases (按 channel 过滤)              │
│       │                                                             │
│       ▼                                                             │
│  FirmwareOtaPage ──▶ RobotClient (gRPC)                            │
│       │                    │                                        │
│       │  新增 UI:          │  新增 RPC:                              │
│       │  - 安装版本列表     │  - GetUpgradeHistory                   │
│       │  - 升级历史面板     │  - ValidateSystemVersion               │
│       │  - Readiness 预检  │  - TLS 可选                            │
└───────│────────────────────│────────────────────────────────────────┘
        │                    │
        │              gRPC (50051)
        │              ┌─────▼─────────────────────────────────┐
        │              │      grpc_gateway (设备端)              │
        │              │                                        │
        │              │  DataServiceImpl                       │
        │              │  ├── ApplyUpdate                       │
        │              │  │   ├── CompareSemver()    [P1.4]     │
        │              │  │   ├── VerifyEd25519()    [P1.2]     │
        │              │  │   ├── PostInstallHealthCheck [P1.3] │
        │              │  │   ├── SaveSystemVersion() [P1.1]    │
        │              │  │   └── AppendHistory()    [P2.1]     │
        │              │  ├── Rollback                          │
        │              │  │   ├── SaveSystemVersion()            │
        │              │  │   └── AppendHistory()                │
        │              │  ├── GetUpgradeHistory      [P2.1]     │
        │              │  └── ValidateSystemVersion  [P2.3]     │
        │              │                                        │
        │              │  HealthMonitor (依赖注入)    [P1.3]     │
        │              └────────────────────────────────────────┘
        │
        ▼
   /opt/robot/ota/
   ├── installed_manifest.json   (增量制品列表)
   ├── system_version.json       (全局版本快照) ← 新增
   ├── upgrade_history.jsonl     (持久化历史)   ← 新增
   └── backups/                  (回滚备份)
```

### 2.2 数据流 — Install 路径

```
App: ApplyUpdate(manifest, artifacts[])
  │
  ▼
  1. CompareSemver(current, target) → 版本校验 ──────────┐
  2. VerifyEd25519Signature(manifest, sig) → 签名校验 ──│── 任一失败 → OtaFailureCode → 返回
  3. CheckDiskSpace / CheckPermissions ─────────────────┘
  │
  ▼
  4. 写事务日志 txn_{name}.json
  5. 备份旧文件到 backups/
  6. 根据 apply_action 执行:
     ├── COPY_ONLY → cp 到目标路径
     ├── RELOAD_MODEL → cp + 重载信号
     ├── RESTART_SERVICE → cp + systemctl restart
     └── REBOOT → cp + 标记重启
  │
  ▼
  7. SHA256 校验安装后文件
  8. PostInstallHealthCheck()
     ├── HOT/UNSPECIFIED → 跳过 (低风险文件级更新)
     └── WARM/COLD → 等待 HealthMonitor 确认系统 OK
         ├── OK → 继续
         └── FAULT → 自动 Rollback → 返回失败
  │
  ▼
  9. 持锁更新:
     ├── installed_artifacts_[name] = artifact
     ├── SaveInstalledManifest()       → installed_manifest.json
     └── SaveSystemVersionJson()       → system_version.json
  │
  ▼
 10. AppendUpgradeHistory() → upgrade_history.jsonl
 11. 返回 OTA_STATUS_SUCCESS
```

### 2.3 数据流 — Rollback 路径

```
App: Rollback(artifact_name)
  │
  ▼
  1. lock(ota_mutex_)
  2. 从 backups/ 恢复旧文件 → 目标路径
  3. installed_artifacts_ 回退版本
  4. SaveInstalledManifest()
  5. SaveSystemVersionJson()
  6. AppendUpgradeHistory(action="rollback")
  7. unlock
  8. 返回 OTA_STATUS_SUCCESS
```

---

## 3. 实现细节

### 3.1 P1.1 — 系统版本快照

**文件**: `system_version.json`

```json
{
  "system_version": "1.1.0",
  "components": {
    "nav_model": {"version": "2.0.0"},
    "slam_config": {"version": "1.3.0"}
  }
}
```

**实现位置**: `data_service.cpp` — `LoadSystemVersionJson()` / `SaveSystemVersionJson()`

**关键决策**: `SaveSystemVersionJson()` 不持内部锁，调用方必须保证 `ota_mutex_` 已持有。避免了 Rollback 场景下的递归死锁。

### 3.2 P1.2 — 设备端 Ed25519 验签

**实现位置**: `data_service.cpp` — `VerifyEd25519Signature()`

```cpp
bool DataServiceImpl::VerifyEd25519Signature(
    const std::string &manifest_json,
    const std::string &signature_b64) {
  // 1. 读取公钥文件 (PEM)
  // 2. Base64 解码签名
  // 3. EVP_DigestVerifyInit → EVP_DigestVerify
  // 4. 当前为 warn-only 模式，正式上线改为阻断
}
```

**配置**: `grpc_gateway.yaml` → `ota_public_key_path: "/opt/robot/ota/keys/ed25519_pub.pem"`

### 3.3 P1.3 — 安装后健康检查

**实现位置**: `data_service.cpp` — `PostInstallHealthCheck()`

**关键修复**: HOT 和 UNSPECIFIED 安全等级跳过健康检查。

原因：HOT 级别更新（模型文件、配置替换）不需要 SLAM 系统运行。当 SLAM 未启动时，`HealthMonitor` 必然报告 FAULT 状态，导致安装成功后立即触发自动回滚——这是一个严重的 false positive。

```cpp
if (safety_level == robot::v1::OTA_SAFETY_LEVEL_HOT ||
    safety_level == robot::v1::OTA_SAFETY_LEVEL_UNSPECIFIED) {
  RCLCPP_INFO(logger_, "HOT/UNSPECIFIED safety → skip health check");
  return true;  // 认为健康
}
```

### 3.4 P1.4 — 语义版本比较

**实现位置**: `data_service.cpp` — `CompareSemver()`

```
"1.9.0"  vs "1.10.0"  → -1 (正确: 1.9.0 < 1.10.0)
"2.0.0"  vs "1.99.99" → +1 (正确: 2.0.0 > 1.99.99)
"1.0.0"  vs "1.0.0"   →  0 (相等)
```

替换了原有的 `std::string::compare()`，该方法按字典序比较会导致 `"9" > "10"`。

### 3.5 P2.1 — 持久化升级历史

**格式**: JSONL (每行一个 JSON 对象)

```jsonl
{"timestamp":"2026-02-09T12:30:00Z","action":"install","name":"nav_model","from":"1.0.0","to":"2.0.0","status":"SUCCESS","failure_code":"OTA_FAILURE_NONE","duration_ms":1523}
{"timestamp":"2026-02-09T12:35:00Z","action":"rollback","name":"nav_model","from":"2.0.0","to":"1.0.0","status":"SUCCESS","failure_code":"OTA_FAILURE_NONE","duration_ms":87}
```

**RPC**: `GetUpgradeHistory(limit, offset)` — 支持分页，最新在前。

### 3.6 P2.2 — 标准化失败码

```protobuf
enum OtaFailureCode {
  OTA_FAILURE_NONE          = 0;
  OTA_FAILURE_NETWORK       = 1;   // 下载/传输失败
  OTA_FAILURE_SIGNATURE     = 2;   // 签名验证失败
  OTA_FAILURE_INTEGRITY     = 3;   // SHA256 不匹配
  OTA_FAILURE_DISK_SPACE    = 4;   // 磁盘空间不足
  OTA_FAILURE_PERMISSION    = 5;   // 文件权限不足
  OTA_FAILURE_DEPENDENCY    = 6;   // 依赖未满足
  OTA_FAILURE_HEALTH_CHECK  = 7;   // 安装后健康检查失败
  OTA_FAILURE_VERSION       = 8;   // 版本约束不满足
  OTA_FAILURE_TIMEOUT       = 9;   // 操作超时
  OTA_FAILURE_ROLLBACK      = 10;  // 回滚本身失败
  OTA_FAILURE_INTERNAL      = 99;  // 未分类内部错误
}
```

### 3.7 P2.3 — 版本一致性 RPC

```protobuf
rpc ValidateSystemVersion(ValidateSystemVersionRequest)
    returns (ValidateSystemVersionResponse);
```

App 发送期望的 manifest，设备比对 `installed_artifacts_` 中每个制品的实际版本，返回不一致列表。

### 3.8 P3.1 — Flutter UI 增强

`firmware_ota_page.dart` 新增：

- **安装版本列表** — 展示 `GetInstalledVersions` 返回的所有制品及其版本、安装时间
- **升级历史面板** — 时间线展示，包含操作类型、版本变迁、状态、失败码
- **Readiness 预检** — 安装前调用 `CheckUpdateReadiness` 并展示结果

### 3.9 P3.2 — 发布通道

`manifest.json` 新增 `"channel": "stable"` 字段。

`CloudOtaService` 过滤逻辑：
```dart
final matchingAssets = release.assets.where((a) {
  final manifest = await _fetchManifest(a);
  return manifest.channel == selectedChannel;
});
```

`generate_manifest.py` 新增 `--channel stable|beta|nightly` 参数。

### 3.10 P3.3 — gRPC TLS 可选

`grpc_gateway.cpp` 条件启用：

```cpp
if (!tls_cert_path_.empty() && !tls_key_path_.empty()) {
  grpc::SslServerCredentialsOptions ssl_opts;
  ssl_opts.pem_key_cert_pairs.push_back({key_pem, cert_pem});
  creds = grpc::SslServerCredentials(ssl_opts);
} else {
  creds = grpc::InsecureServerCredentials();
}
```

---

## 4. Bug 修复记录

### 4.1 健康检查误触发回滚

| 项目 | 详情 |
|------|------|
| **现象** | `ApplyUpdate` 对 HOT 级别的模型文件成功安装后，立即报告 `OTA_FAILURE_HEALTH_CHECK` 并自动回滚 |
| **根因** | `PostInstallHealthCheck()` 无条件调用 `HealthMonitor::GetSystemHealth()`，当 SLAM 系统未运行时返回 FAULT |
| **影响** | 所有 HOT 级别更新（模型/配置/地图）在非运行态无法安装 |
| **修复** | 对 `OTA_SAFETY_LEVEL_HOT` 和 `OTA_SAFETY_LEVEL_UNSPECIFIED` 跳过健康检查 |
| **文件** | `data_service.cpp` — `PostInstallHealthCheck()` |

### 4.2 Rollback RPC 死锁

| 项目 | 详情 |
|------|------|
| **现象** | `Rollback` RPC 调用后无响应，grpcurl 超时；后续所有 RPC 也被阻塞 |
| **根因** | 调用链: `Rollback()` [持有 `ota_mutex_`] → `SaveSystemVersionJson()` [尝试获取 `ota_mutex_`] → 死锁 |
| **影响** | Rollback 功能完全不可用，且会导致整个 DataService 挂起 |
| **修复** | (1) 移除 `SaveSystemVersionJson()` 内部的 `std::lock_guard`，改为调用方保证互斥 (2) 将 `ApplyUpdate` 中的 `SaveInstalledManifest()` / `SaveSystemVersionJson()` 调用移入 `ota_mutex_` 锁作用域内 |
| **文件** | `data_service.cpp` — `SaveSystemVersionJson()` + `ApplyUpdate()` |

---

## 5. 端到端测试

### 5.1 测试环境

| 项目 | 值 |
|------|-----|
| 硬件 | Sunrise X5 (aarch64) |
| OS | Linux 6.1.112-rt43 |
| ROS2 | Humble |
| gRPC 测试工具 | grpcurl |
| 测试制品 | `/opt/robot/models/nav_model.bin` |

### 5.2 测试准备

```bash
# 创建测试模型文件 (v1.0.0)
echo "THIS IS OLD MODEL v1.0.0" > /opt/robot/models/nav_model.bin

# 清空旧状态
rm -f /opt/robot/ota/installed_manifest.json
rm -f /opt/robot/ota/upgrade_history.jsonl
rm -f /opt/robot/ota/system_version.json

# 启动 grpc_gateway
ros2 run remote_monitoring grpc_gateway
```

### 5.3 测试步骤与结果

#### Step 1: 预检查

```bash
grpcurl -plaintext -d '{
  "manifest": {
    "artifacts": [{
      "name": "nav_model",
      "version": "2.0.0",
      "category": "OTA_CATEGORY_MODEL",
      "file_size_bytes": 1024,
      "target_path": "/opt/robot/models/nav_model.bin",
      "safety_level": "OTA_SAFETY_LEVEL_HOT",
      "apply_action": "OTA_APPLY_ACTION_COPY_ONLY"
    }]
  }
}' localhost:50051 robot.v1.DataService/CheckUpdateReadiness
```

**结果**: `"ready": true`

#### Step 2: 安装 (v1.0.0 → v2.0.0)

```bash
grpcurl -plaintext -d '{
  "manifest": { ... },
  "file_data": { "nav_model": "<base64-of-new-model>" }
}' localhost:50051 robot.v1.DataService/ApplyUpdate
```

**结果**: `"status": "OTA_STATUS_SUCCESS"`, `"message": "All 1 artifacts installed successfully"`

**验证**:
```bash
$ cat /opt/robot/models/nav_model.bin
THIS IS NEW MODEL v2.0.0
```

#### Step 3: 查看安装版本

```bash
grpcurl -plaintext localhost:50051 robot.v1.DataService/GetInstalledVersions
```

**结果**: 包含 `nav_model` v2.0.0，`system_version_json` 正确填充。

#### Step 4: 查看升级历史

```bash
grpcurl -plaintext -d '{"limit": 10}' localhost:50051 robot.v1.DataService/GetUpgradeHistory
```

**结果**: 包含 1 条 `install` 记录。

#### Step 5: 回滚

```bash
grpcurl -plaintext -d '{"artifact_name": "nav_model"}' \
  localhost:50051 robot.v1.DataService/Rollback
```

**结果**: `"status": "OTA_STATUS_SUCCESS"`

**验证**:
```bash
$ cat /opt/robot/models/nav_model.bin
THIS IS OLD MODEL v1.0.0
```

#### Step 6: 回滚后状态

- `upgrade_history.jsonl` 包含 2 条记录 (install + rollback)
- `system_version.json` 版本回退
- `installed_manifest.json` 版本回退

### 5.4 测试结论

| 测试项 | 状态 |
|--------|------|
| 预检查 (CheckUpdateReadiness) | PASS |
| 安装 (ApplyUpdate) | PASS |
| 文件替换验证 | PASS |
| 版本状态更新 | PASS |
| 健康检查跳过 (HOT) | PASS |
| 回滚 (Rollback) | PASS |
| 文件恢复验证 | PASS |
| 升级历史持久化 | PASS |
| 系统版本快照 | PASS |
| 无死锁 | PASS |

**结论: OTA v3 完整闭环通过。Install → 文件替换 → Rollback → 文件恢复，所有状态文件正确更新。**

---

## 6. 变更文件清单

| 文件 | 行数变化 | 说明 |
|------|---------|------|
| `src/robot_proto/proto/data.proto` | submodule | 新增 OtaFailureCode, GetUpgradeHistory, ValidateSystemVersion |
| `src/remote_monitoring/src/services/data_service.cpp` | +583 | 核心逻辑: 版本快照、验签、健康检查、历史、死锁修复 |
| `src/remote_monitoring/include/.../data_service.hpp` | +51 | 方法声明 + 成员变量 |
| `src/remote_monitoring/src/grpc_gateway.cpp` | +59 | HealthMonitor 注入、TLS |
| `src/remote_monitoring/include/.../grpc_gateway.hpp` | +4 | TLS 成员 |
| `src/remote_monitoring/config/grpc_gateway.yaml` | +12 | v3 配置项 |
| `client/flutter_monitor/.../robot_client.dart` | +51 | RPC 实现 + TLS |
| `client/flutter_monitor/.../robot_client_base.dart` | +13 | 抽象方法 |
| `client/flutter_monitor/.../firmware_ota_page.dart` | +433 | 安装版本 + 升级历史 UI |
| `client/flutter_monitor/.../cloud_ota_service.dart` | +55 | 通道过滤 |
| `scripts/ota/generate_manifest.py` | +38 | --channel, --system-manifest |
| `scripts/ota/manifest_template.json` | +20 | channel 字段 |
| `scripts/ota/build_nav_package.sh` | new | 导航包构建脚本 |
| `docs/OTA_GUIDE.md` | +470 | §16 v3 增强 + §17 Roadmap |
| `docs/CHANGELOG.md` | +60 | v1.1.0 条目 |
| `.gitignore` | +7 | 排除 OTA 运行时文件 |
| **合计** | **~1,725+** | |

---

## 7. 遗留问题与后续计划

### 7.1 短期 (P0/P1)

| 优先级 | 任务 | 说明 |
|--------|------|------|
| P0 | ModeManager 强制检查 | COLD 更新必须要求 mode == IDLE 或 ESTOP |
| P1 | Ed25519 验签改为强制 | 当前 warn-only，正式环境应阻断未签名 manifest |
| P1 | Flutter 通道选择 UI | App Settings 增加"更新通道"下拉框 |

### 7.2 中期

| 任务 | 说明 |
|------|------|
| 升级历史 UI 详情页 | 点击历史条目展开失败原因、耗时、健康检查结果 |
| TLS 证书自动交换 | App 首次连接时弹出指纹确认对话框 |
| Flutter App 端到端连接测试 | 验证 gRPC 连接、UI 展示、操作闭环 |

### 7.3 长期

- 差分更新 (bsdiff/courgette)
- A/B 分区方案
- Fleet OTA 管理 (多机统一升级)

---

## 8. 附录

### A. 关键配置 (`grpc_gateway.yaml`)

```yaml
# OTA v3 新增配置
ota_public_key_path: "/opt/robot/ota/keys/ed25519_pub.pem"
ota_history_path: "/opt/robot/ota/upgrade_history.jsonl"
system_version_path: "/opt/robot/ota/system_version.json"

# TLS (可选)
tls_cert_path: ""   # 留空则使用 InsecureCredentials
tls_key_path: ""
```

### B. Protobuf 新增定义

```protobuf
// 失败码
enum OtaFailureCode { ... }

// 升级历史
rpc GetUpgradeHistory(GetUpgradeHistoryRequest) returns (GetUpgradeHistoryResponse);

message UpgradeHistoryEntry {
  string timestamp = 1;
  string action = 2;        // "install" | "rollback"
  string artifact_name = 3;
  string from_version = 4;
  string to_version = 5;
  OtaStatus status = 6;
  OtaFailureCode failure_code = 7;
  int64 duration_ms = 8;
}

// 版本一致性校验
rpc ValidateSystemVersion(ValidateSystemVersionRequest) returns (ValidateSystemVersionResponse);

message VersionMismatch {
  string artifact_name = 1;
  string expected_version = 2;
  string actual_version = 3;
}
```

### C. 测试命令速查

```bash
# 预检查
grpcurl -plaintext -d '{"manifest":{...}}' localhost:50051 robot.v1.DataService/CheckUpdateReadiness

# 安装
grpcurl -plaintext -d '{"manifest":{...},"file_data":{...}}' localhost:50051 robot.v1.DataService/ApplyUpdate

# 查看已安装版本
grpcurl -plaintext localhost:50051 robot.v1.DataService/GetInstalledVersions

# 查看升级历史
grpcurl -plaintext -d '{"limit":10}' localhost:50051 robot.v1.DataService/GetUpgradeHistory

# 回滚
grpcurl -plaintext -d '{"artifact_name":"nav_model"}' localhost:50051 robot.v1.DataService/Rollback

# 版本一致性校验
grpcurl -plaintext -d '{"expected_versions":{...}}' localhost:50051 robot.v1.DataService/ValidateSystemVersion
```

---

*报告完成于 2026-02-09*
