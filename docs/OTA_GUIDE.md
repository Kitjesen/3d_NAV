# OTA (Over-The-Air) 更新系统 — 产品级规范

## 1. 设计原则

本系统的 OTA 必须满足三条底线，违反任意一条则不允许发布：

| 底线 | 含义 | 实现机制 |
|------|------|---------|
| **可验证的真实性** | 机器人确认制品来自官方发布，而非伪造 | Ed25519 manifest 签名 |
| **原子安装 + 崩溃恢复** | 断电/WiFi 中断/磁盘满不会导致机器人不可用 | 事务日志 + 自动回滚 |
| **安全状态约束** | 更新关键制品时机器人不能处于运动状态 | safety_level 分级 + 模式检查 |

---

## 2. 系统架构

```
┌──────────────────┐     HTTPS      ┌──────────────────┐      gRPC       ┌──────────────────┐
│  GitHub Releases │◄────────────────│  Flutter App     │────────────────►│  Nav Board       │
│                  │                 │                  │                 │  DataService     │
│  manifest.json   │   直接下载      │  1. 解析 manifest │                 │                  │
│  (Ed25519 签名)  │◄─ ─ ─ ─ ─ ─ ─ ─│  2. 签名验证     │                 │  1. 签名验证     │
│  *.onnx / .deb   │                 │  3. 版本对比     │                 │  2. 依赖检查     │
│  *.pcd / .yaml   │                 │  4. 预检查       │                 │  3. 安全模式     │
│                  │                 │  5. 进度展示     │                 │  4. 事务日志     │
└──────────────────┘                 └──────────────────┘                 │  5. 备份→安装    │
                                                                          │  6. 验证→清理    │
                                                                          └──────────────────┘
```

### OTA 完整生命周期

```
1. 检查更新 (Check)
   ├── 客户端从 GitHub Releases 拉取 manifest.json
   ├── Ed25519 签名验证 (拒绝未签名/伪造 manifest)
   └── 对比本地已安装版本 (GetInstalledVersions)

2. 预检查 (Pre-flight) — CheckUpdateReadiness
   ├── 磁盘空间 (需 2x: staging + 目标)
   ├── 电量 (min_battery_percent)
   ├── 硬件兼容性 (hw_compat)
   ├── 网络连通性
   ├── 依赖兼容性 (dependencies: A requires B >= v1.2)
   ├── 安全模式检查 (COLD 级别 → 机器人必须坐下+禁用电机)
   └── 事务日志残留 (检测上次中断的安装)

3. 安全模式进入 (根据 safety_level)
   ├── HOT:  无需停机, 直接安装
   ├── WARM: 暂停导航, 安装后恢复
   └── COLD: sit → disable motors → 进入维护态 → 安装 → 重启

4. 下载到机器人
   ├── 方式A: DownloadFromUrl → 机器人直连 GitHub (推荐)
   └── 方式B: UploadFile → 手机中转 (支持断点续传)

5. 原子安装 (ApplyUpdate)
   ├── SHA256 校验 (完整性)
   ├── 依赖版本检查
   ├── 写入事务日志 (status=installing)
   ├── 备份旧版本
   ├── 执行安装 (按 apply_action 分发到对应模块)
   ├── 成功: 清理事务日志 + 更新 installed_manifest
   └── 失败: 标记事务日志 + 自动回滚备份

6. 崩溃恢复 (下次启动时)
   └── 检测 txn_*.json → status=installing → 自动回滚

7. 验证 (Validate)
   └── GetInstalledVersions 确认版本更新

8. 回滚 (Rollback)
   └── 手动触发: 从备份恢复上一版本
```

---

## 3. 制品安全等级与系统边界

### 3.1 安全等级 (safety_level)

每个制品必须声明安全等级，决定安装前机器人必须达到的状态：

| 等级 | 要求 | 制品类型 | 安装时机器人状态 |
|------|------|---------|----------------|
| **HOT** | 无需停机 | 地图 (PCD), 配置 (YAML) | 可运行中安装 |
| **WARM** | 暂停导航 | 模型 (ONNX) | ModeManager 切到 IDLE, 安装后恢复 |
| **COLD** | sit + disable + 维护态 | 固件 (DEB), MCU (HEX) | 必须坐下, 禁用电机, 等待重启 |

**COLD 更新的完整流程**:

```
客户端发起 COLD 更新
    │
    ▼
CheckUpdateReadiness → safety_mode check → 提示用户
    │
    ▼
用户确认 → App 发送 SetMode(IDLE) → SitDown → Disable
    │
    ▼
ApplyUpdate → 写事务日志 → 备份 → dpkg -i / flash_mcu
    │
    ├── 成功 → 清理日志 → 提示重启
    └── 失败 → 自动回滚 → 标记日志 → 提示用户
```

### 3.2 系统边界 (owner_module)

不同类型的制品由不同模块负责安装和生命周期管理：

| owner_module | 负责制品 | 安装方式 | 生命周期管理 |
|-------------|---------|---------|------------|
| **brain** | ONNX 模型 | 复制 + 通知 Dog Board 热加载 | Dog Board CMS 管理模型版本 |
| **navigation** | PCD 地图 | 复制 + 通知 Localizer 重新加载 | Localizer 管理 map cache |
| **config_service** | YAML 配置 | 复制 + 发 ROS2 参数事件 | 各节点监听参数变化 |
| **system** | DEB 固件 | dpkg -i + 重启 | systemd 管理服务生命周期 |
| **mcu** | HEX/BIN | 刷写脚本 + 重启 | 硬件自检 |

**为什么需要边界**: 当前所有安装动作都由 DataService 执行，短期可行。但未来模块拆分后（例如 Brain 独立进程、ConfigService 动态刷新），OTA 安装行为必须委托给对应模块，DataService 只负责协调。`owner_module` 字段为此预留扩展点。

---

## 4. Manifest 签名与真实性验证

### 4.1 签名方案: Ed25519

```
发布侧 (CI/CD):                      机器人侧:
┌────────────────────┐                ┌────────────────────┐
│ 1. 生成 manifest   │                │ 1. 下载 manifest   │
│ 2. 去除 signature  │                │ 2. 提取 signature  │
│ 3. canonical JSON  │                │ 3. canonical JSON  │
│ 4. Ed25519 签名    │                │ 4. Ed25519 验签    │
│ 5. 写入 signature  │                │ 5. 通过 → 继续     │
│ 6. 上传 Release    │                │    失败 → 拒绝     │
└────────────────────┘                └────────────────────┘
```

### 4.2 密钥管理

```bash
# 生成密钥对 (首次, 在开发机上)
python3 scripts/ota/generate_manifest.py --generate-keys --key-dir ./keys/

# 输出:
#   keys/ota_private.pem  ← 保密! 仅 CI/CD 使用
#   keys/ota_public.pem   ← 部署到每台机器人

# 部署公钥到机器人
scp keys/ota_public.pem robot:/opt/robot/ota/ota_public.pem
```

### 4.3 CI 签名流程

```yaml
# .github/workflows/release.yml
- name: Generate & sign manifest
  run: |
    python3 scripts/ota/generate_manifest.py \
      --version ${{ github.ref_name }} \
      --artifacts-dir ./dist/ \
      --signing-key ${{ secrets.OTA_SIGNING_KEY_PATH }} \
      --key-id ota-signing-key-01 \
      --output ./dist/manifest.json
```

### 4.4 当前状态

> **注意**: 签名验证在客户端侧实现（Flutter App 内置公钥验证 manifest）。
> 服务端 (`CheckUpdateReadiness`) 预留了 `manifest_signature` 字段，
> 未来可在机器人侧也做验签（双重验证）。
> 当前阶段，未签名的 manifest 仍然可以使用，但会在日志中打印警告。

---

## 5. 原子安装与崩溃恢复

### 5.1 事务日志

每次 `ApplyUpdate` 开始前，写入事务日志:

```
/opt/robot/ota/backup/txn_{artifact_name}.json
```

内容:
```json
{
  "artifact": "policy_walk",
  "version": "2.3.0",
  "status": "installing",
  "staged_path": "/tmp/ota_staging/policy_walk_v2.3.onnx",
  "target_path": "/opt/robot/models/policy_walk.onnx",
  "started_at": "2026-02-08T10:30:00Z"
}
```

### 5.2 安装状态机

```
写入 txn (status=installing)
    │
    ├── 安装成功 → 删除 txn 文件 → 更新 installed_manifest
    │
    ├── 安装失败 → 更新 txn (status=failed) → 如有备份则回滚
    │
    └── 异常中断 (断电/崩溃)
         → 下次启动: 检测 txn status=installing
         → 备份存在: 自动回滚 → 记录事件
         → 无备份: 标记为 failed → 告警
```

### 5.3 目录结构

```
/opt/robot/
├── ota/
│   ├── installed_manifest.json    # 已安装制品记录 (原子写: 先写 .tmp 再 rename)
│   ├── ota_public.pem             # Ed25519 公钥 (验签用)
│   └── backup/
│       ├── policy_walk_20260207_120000.onnx   # 旧版本备份
│       ├── terrain_params_20260207_120000.yaml
│       └── txn_policy_walk.json               # 事务日志 (安装完成后删除)
├── models/
│   ├── policy_walk.onnx           # 当前步态模型
│   └── perception.onnx
├── maps/
│   ├── campus_map.pcd
│   └── indoor_map.pcd
├── config/
│   └── terrain_params.yaml
└── firmware/
    └── nav_board.deb
```

---

## 6. 版本兼容与依赖管理

### 6.1 manifest 中的 dependencies 字段

每个制品可以声明对其他制品的版本依赖：

```json
{
  "name": "policy_walk",
  "category": "model",
  "version": "2.3.0",
  "dependencies": [
    {
      "artifact_name": "nav_firmware",
      "min_version": "1.2.0"
    }
  ]
}
```

含义: `policy_walk v2.3.0` 要求 `nav_firmware` 已安装且版本 >= `1.2.0`。

### 6.2 依赖检查时机

| 阶段 | 检查 | 行为 |
|------|------|------|
| `CheckUpdateReadiness` | 预扫描所有制品的依赖 | 不满足 → `ready=false`, 告诉用户需要先更新哪些 |
| `ApplyUpdate` | 安装前二次确认 | 不满足 → 拒绝安装 (除非 `force=true`) |

### 6.3 常见依赖场景

| 场景 | 依赖表达 |
|------|---------|
| 模型 v2.3 需要新版推理框架 | `policy_walk.dependencies = [{nav_firmware, >=1.2.0}]` |
| 地图需要特定 SLAM 参数 | `campus_map.dependencies = [{terrain_config, >=1.1.0}]` |
| 固件需要特定内核版本 | `nav_firmware` 的 `min_system_version = "1.0.0"` (manifest 级别) |

---

## 7. 云端 manifest.json 格式 (schema v2)

### 完整示例

```json
{
  "schema_version": "2",
  "release_version": "v2.3.0",
  "release_date": "2026-02-08T12:00:00Z",
  "min_system_version": "1.0.0",
  "signature": "a1b2c3d4...128hex...",
  "public_key_id": "ota-signing-key-01",
  "artifacts": [
    {
      "name": "policy_walk",
      "category": "model",
      "version": "2.3.0",
      "filename": "policy_walk_v2.3.onnx",
      "sha256": "a1b2c3d4e5f6...64hex...",
      "target_path": "/opt/robot/models/policy_walk.onnx",
      "target_board": "dog",
      "hw_compat": ["dog_v2", "dog_v3"],
      "apply_action": "reload_model",
      "requires_reboot": false,
      "min_battery_percent": 20,
      "changelog": "优化步态稳定性，减少摔倒率",
      "rollback_safe": true,
      "safety_level": "warm",
      "owner_module": "brain",
      "dependencies": [
        { "artifact_name": "nav_firmware", "min_version": "1.2.0" }
      ]
    },
    {
      "name": "nav_firmware",
      "category": "firmware",
      "version": "1.2.0",
      "filename": "nav_board_v1.2.0.deb",
      "sha256": "f6e5d4c3b2a1...64hex...",
      "target_path": "/opt/robot/firmware/nav_board.deb",
      "target_board": "nav",
      "hw_compat": ["*"],
      "apply_action": "install_deb",
      "requires_reboot": true,
      "min_battery_percent": 50,
      "changelog": "修复 SLAM 内存泄漏",
      "rollback_safe": false,
      "safety_level": "cold",
      "owner_module": "system",
      "dependencies": []
    },
    {
      "name": "campus_map",
      "category": "map",
      "version": "1.0.0",
      "filename": "campus_map.pcd",
      "sha256": "1234abcd...64hex...",
      "target_path": "/opt/robot/maps/campus_map.pcd",
      "target_board": "nav",
      "hw_compat": ["*"],
      "apply_action": "copy_only",
      "requires_reboot": false,
      "min_battery_percent": 0,
      "changelog": "校园环境预建地图",
      "rollback_safe": true,
      "safety_level": "hot",
      "owner_module": "navigation",
      "dependencies": []
    }
  ]
}
```

### 字段说明 (v2 新增字段标记 NEW)

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `schema_version` | string | 是 | `"2"` (新版) |
| `release_version` | string | 是 | Release 版本号 (semver) |
| `release_date` | string | 是 | ISO 8601 日期 |
| `min_system_version` | string | 否 | 机器人最低系统版本要求 |
| `signature` | string | **NEW** | Ed25519 签名 (hex), 对去除此字段后的 canonical JSON 签名 |
| `public_key_id` | string | **NEW** | 公钥标识, 用于密钥轮换 |
| `artifacts[].name` | string | 是 | 制品唯一名称 |
| `artifacts[].category` | string | 是 | `model` / `firmware` / `map` / `config` |
| `artifacts[].version` | string | 是 | semver 版本号 |
| `artifacts[].filename` | string | 是 | Release Asset 文件名 |
| `artifacts[].sha256` | string | 是 | 文件 SHA256 |
| `artifacts[].target_path` | string | 是 | 安装路径 |
| `artifacts[].target_board` | string | 否 | `nav` / `dog`, 默认 `nav` |
| `artifacts[].hw_compat` | string[] | 否 | `["*"]` = 全部 |
| `artifacts[].apply_action` | string | 否 | 安装动作, 见下表 |
| `artifacts[].requires_reboot` | bool | 否 | 安装后是否需重启 |
| `artifacts[].min_battery_percent` | int | 否 | 最低电量 |
| `artifacts[].changelog` | string | 否 | 变更说明 |
| `artifacts[].rollback_safe` | bool | 否 | 是否支持回滚 |
| `artifacts[].safety_level` | string | **NEW** | `hot` / `warm` / `cold` |
| `artifacts[].owner_module` | string | **NEW** | 负责模块: `brain` / `navigation` / `config_service` / `system` / `mcu` |
| `artifacts[].dependencies` | array | **NEW** | 依赖列表 `[{artifact_name, min_version, max_version}]` |

### apply_action 枚举

| 值 | safety_level | 说明 |
|----|-------------|------|
| `copy_only` | hot | 仅复制到 target_path |
| `reload_model` | warm | 复制后通知 Dog Board 热加载 |
| `install_deb` | cold | dpkg -i 安装 |
| `flash_mcu` | cold | 刷写脚本烧录 MCU |
| `install_script` | cold | 执行自定义安装脚本 |

---

## 8. gRPC 接口

### CheckUpdateReadiness (安装前预检查)

```protobuf
rpc CheckUpdateReadiness(CheckUpdateReadinessRequest) returns (CheckUpdateReadinessResponse);
```

**v2 新增检查项**:

| check_name | 说明 | 阻塞 |
|-----------|------|------|
| `disk_space` | 磁盘空间 ≥ 2x 总制品大小 | 是 |
| `battery` | 电量 ≥ max(min_battery_percent) | 是 |
| `hw_compat` | 硬件 ID 在 hw_compat 列表中 | 是 |
| `network` | github.com 可达 | 否 (可手机中转) |
| `safety_mode` | COLD 级别需机器人坐下 | 是 (COLD) |
| `dependency` | 前置制品版本满足 | 是 |
| `stale_transaction` | 上次中断的安装残留 | 否 (警告) |

### ApplyUpdate (原子安装)

```protobuf
rpc ApplyUpdate(ApplyUpdateRequest) returns (ApplyUpdateResponse);
```

**v2 增强流程**:
1. 路径安全检查 (禁止 `..`)
2. SHA256 校验
3. 硬件兼容性
4. **安全等级检查** (COLD → 要求 IDLE 模式)
5. **依赖版本检查**
6. **写入事务日志** (`txn_{name}.json`)
7. 备份旧版本
8. 执行安装 (按 apply_action 分发)
9. 成功 → 清理日志 + 更新 manifest / 失败 → 标记日志

### DownloadFromUrl (机器人直接下载)

```protobuf
rpc DownloadFromUrl(DownloadFromUrlRequest) returns (stream OtaProgress);
```

机器人通过 curl 直接从 GitHub 下载，返回实时进度流。
200MB ONNX 不再经手机中转，速度 2-3x。

### UploadFile (断点续传)

```protobuf
rpc UploadFile(stream UploadFileChunk) returns (UploadFileResponse);
```

`resume_from_offset` 支持 WiFi 中断后从断点继续。

### GetInstalledVersions / Rollback

```protobuf
rpc GetInstalledVersions(GetInstalledVersionsRequest) returns (GetInstalledVersionsResponse);
rpc Rollback(RollbackRequest) returns (RollbackResponse);
```

---

## 9. 用户体验规范

### 9.1 更新提示

| 场景 | App 行为 |
|------|---------|
| 新版本可用 | Settings 页显示红点 + "有 N 个更新可用" |
| COLD 更新 | **明确警告**: "此更新需要机器人停机，安装期间无法使用" |
| 依赖不满足 | "需要先更新 X 到 v1.2+，是否一起更新？" |
| 电量不足 | "电量 15%，需要 50% 以上，请先充电" |

### 9.2 更新窗口

- **自动检查**: App 启动时 + 每小时静默检查 GitHub Release
- **手动触发**: Settings → OTA → "检查更新"
- **自动安装**: 仅 HOT 级别 + 用户已开启自动更新 + 机器人 IDLE 状态
- **WARM/COLD**: 必须用户手动确认

### 9.3 断电/中断恢复

| 中断时机 | 恢复行为 |
|---------|---------|
| 下载中断 | 下次自动从断点继续 (resume_from_offset) |
| 安装中断 (HOT) | 下次启动检测 txn → 自动回滚 |
| 安装中断 (COLD) | 下次启动检测 txn → 自动回滚 → App 提示 |
| 安装成功但验证失败 | 自动回滚 + App 告警 |

### 9.4 极端场景兜底

| 场景 | 自动行为 |
|------|---------|
| 更新模型后机器人无法站立 | App 检测到 FAULT → 提示一键回滚 |
| 更新固件后服务不启动 | systemd watchdog → 自动回滚 deb |
| 回滚也失败 | App 提示: "请联系技术支持" + 自动上传事务日志 |

---

## 10. GitHub Release 发布流程

### 手动发布

```bash
# 1. 生成密钥 (首次)
python3 scripts/ota/generate_manifest.py --generate-keys --key-dir ./keys/

# 2. 计算 SHA256
sha256sum dist/policy_walk_v2.3.onnx

# 3. 生成签名 manifest
python3 scripts/ota/generate_manifest.py \
    --version v2.3.0 \
    --artifacts-dir ./dist/ \
    --signing-key ./keys/ota_private.pem \
    --output ./dist/manifest.json

# 4. 上传到 GitHub Release
gh release create v2.3.0 dist/* --title "v2.3.0" --notes "步态优化"
```

### CI/CD 自动发布

```yaml
name: OTA Release
on:
  push:
    tags: ['v*']

jobs:
  release:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Build artifacts
        run: make build-artifacts

      - name: Install signing deps
        run: pip install cryptography

      - name: Generate & sign manifest
        run: |
          echo "${{ secrets.OTA_PRIVATE_KEY }}" > /tmp/ota_key.pem
          python3 scripts/ota/generate_manifest.py \
            --version ${{ github.ref_name }} \
            --artifacts-dir ./dist/ \
            --signing-key /tmp/ota_key.pem \
            --key-id ota-signing-key-01 \
            --output ./dist/manifest.json
          rm /tmp/ota_key.pem

      - name: Create GitHub Release
        uses: softprops/action-gh-release@v2
        with:
          files: dist/*
```

---

## 11. 安全考虑

| 威胁 | 防御 | 当前状态 |
|------|------|---------|
| manifest 伪造 | Ed25519 签名验证 | 已实现 (generate_manifest.py) |
| 制品篡改 | SHA256 校验 (每个制品) | 已实现 |
| 路径遍历攻击 | 禁止 `..` 组件 | 已实现 |
| 错误硬件刷写 | hw_compat 白名单 | 已实现 |
| 刷写中断 | 事务日志 + 自动回滚 | 已实现 |
| 运动中刷写 | safety_level 分级 + 模式检查 | 已实现 (COLD 警告, TODO: 强制) |
| 电量不足 | min_battery_percent | 已实现 |
| 版本不兼容 | dependencies 依赖检查 | 已实现 |
| 中间人攻击 | gRPC channel TLS | TODO: 启用 TLS |
| 密钥泄露 | public_key_id 支持密钥轮换 | 已预留 |

---

## 12. Proto 新增定义 (v2)

```protobuf
// 安全等级
enum OtaSafetyLevel {
  OTA_SAFETY_LEVEL_UNSPECIFIED = 0;
  OTA_SAFETY_LEVEL_HOT = 1;     // 热更新: 地图、配置
  OTA_SAFETY_LEVEL_WARM = 2;    // 温更新: 模型
  OTA_SAFETY_LEVEL_COLD = 3;    // 冷更新: 固件、MCU
}

// 制品间依赖
message ArtifactDependency {
  string artifact_name = 1;
  string min_version = 2;
  string max_version = 3;       // 可选
}

// OtaArtifact 新增字段:
//   safety_level = 15;
//   dependencies = 16;
//   owner_module = 17;

// 事务日志 (崩溃恢复)
message OtaTransactionLog {
  string transaction_id = 1;
  string artifact_name = 2;
  string artifact_version = 3;
  OtaUpdateStatus status = 4;
  string staged_path = 5;
  string target_path = 6;
  string backup_path = 7;
  string started_at = 8;
  string completed_at = 9;
  string error_message = 10;
}

// CheckUpdateReadinessRequest 新增:
//   string manifest_signature = 3;  // 验签用
```

---

## 13. 客户端调用示例 (Flutter/Dart)

### 方式 A: 机器人直接下载 (推荐)

```dart
// 1. 检查更新 + 签名验证
final release = await cloudOtaService.fetchLatestRelease();
if (release?.hasManifest != true) return;
if (!_verifySignature(release!.manifest!)) {
  showError('manifest 签名验证失败, 可能被篡改');
  return;
}

// 2. 预检查 (含依赖 + 安全等级)
final artifacts = release.manifest!.artifacts.map(_toOtaArtifact).toList();
final readiness = await robotClient.checkUpdateReadiness(
    artifacts: artifacts,
    manifestSignature: release.manifest!.signature);

if (!readiness.ready) {
  for (final check in readiness.checks.where((c) => !c.passed)) {
    print('预检查失败: ${check.checkName} - ${check.message}');
  }
  return;
}

// 3. COLD 更新提示
final hasCold = artifacts.any((a) => a.safetyLevel == OtaSafetyLevel.COLD);
if (hasCold) {
  final confirmed = await showDialog('此更新需要机器人停机, 是否继续?');
  if (!confirmed) return;
  await robotClient.setMode(RobotMode.IDLE);
  await robotClient.sitDown();
  await robotClient.disable();
}

// 4. 下载 + 安装
for (final artifact in release.manifest!.artifacts) {
  final asset = release.assets.firstWhere((a) => a.name == artifact.filename);
  final stagingPath = '/tmp/ota_staging/${artifact.filename}';

  await for (final progress in robotClient.downloadFromUrl(
    url: asset.downloadUrl,
    stagingPath: stagingPath,
    expectedSha256: artifact.sha256,
    expectedSize: asset.size,
  )) {
    updateProgressUI(progress);
  }

  final result = await robotClient.applyUpdate(
      artifact: _toOtaArtifact(artifact), stagedPath: stagingPath);
  print('${artifact.name}: ${result.success ? "成功" : result.message}');
}
```

### 方式 B: 手机中转 (无外网时)

```dart
final stagingPath = '/tmp/ota_staging/${artifact.filename}';
final uploadResp = await robotClient.uploadFile(
    localBytes: bytes,
    remotePath: stagingPath,
    filename: artifact.filename,
    sha256: artifact.sha256,
    resumeFromOffset: lastReceivedOffset,  // 断点续传
    onProgress: (p) => updateUI(p));

assert(uploadResp.sha256 == artifact.sha256);
```

---

## 14. 从 v1 迁移到 v2

| 变更 | 影响 | 迁移方式 |
|------|------|---------|
| schema_version: "1" → "2" | manifest 格式 | 新字段均为可选, v1 manifest 仍可解析 |
| 新增 signature | 安全性 | 未签名的 manifest 继续工作, 仅打警告 |
| 新增 safety_level | 安装流程 | 默认 HOT, 不影响现有行为 |
| 新增 dependencies | 兼容性 | 默认空, 不影响现有安装 |
| 新增 owner_module | 未来扩展 | 当前仅记录, 不影响安装逻辑 |
| 事务日志 | 可靠性 | 新增功能, 自动启用 |

**向后兼容**: v2 服务端完全兼容 v1 manifest, 新字段缺失时使用安全默认值。

---

*最后更新: 2026-02-08*
