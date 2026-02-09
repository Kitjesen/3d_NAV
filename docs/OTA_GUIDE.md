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

> **v3 更新**: 签名验证已在**双端**实现：
> - **Flutter App 端**: 内置公钥验证下载的 manifest
> - **设备端** (v3 P1.2): `CheckUpdateReadiness` 使用 OpenSSL `EVP_DigestVerify` 验签
>   公钥路径: `/opt/robot/ota/ota_public.pem`
>
> **向后兼容**: 未配置公钥或未签名的 manifest 仍可通过，但会在日志中打印 WARN。
> 正式环境建议切为强制阻塞（见 §17.1 待开展工作）。

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
| manifest 伪造 | Ed25519 签名验证 | ✅ App + 设备端双重验签 (v3 P1.2) |
| 制品篡改 | SHA256 校验 (每个制品) | ✅ 已实现 |
| 路径遍历攻击 | 禁止 `..` 组件 | ✅ 已实现 |
| 错误硬件刷写 | hw_compat 白名单 | ✅ 已实现 |
| 刷写中断 | 事务日志 + 自动回滚 | ✅ 已实现 |
| 安装后故障 | HealthMonitor 自动健康检查 + 回滚 | ✅ v3 P1.3 新增 |
| 运动中刷写 | safety_level 分级 + 模式检查 | ⚠️ WARN 日志 (TODO: COLD 强制阻塞) |
| 电量不足 | min_battery_percent | ✅ 已实现 |
| 版本不兼容 | dependencies 依赖检查 | ✅ semver 整数比较 (v3 P1.4 修复) |
| 版本漂移 | system_version.json + ValidateSystemVersion | ✅ v3 P1.1 / P2.3 新增 |
| 事后审计 | upgrade_history.jsonl 持久日志 | ✅ v3 P2.1 新增 |
| 失败分类 | OtaFailureCode 标准化 | ✅ v3 P2.2 新增 |
| 中间人攻击 | gRPC TLS (可选, 自签证书) | ✅ v3 P3.3 可选启用 |
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

## 15. 导航功能包 OTA (ROS2 Package Push)

除了模型/地图/配置/固件，OTA 系统还支持推送**完整的 ROS2 导航功能包**更新。

### 15.1 本地维护清单

开发端 (本地) 需要维护以下内容：

| 文件/目录 | 作用 | 维护方式 |
|-----------|------|---------|
| `VERSION` | 统一版本号 (semver) | 每次发布前手动更新 |
| `scripts/ota/build_nav_package.sh` | 构建 + 打包脚本 | 随工程一起维护 |
| `scripts/ota/install_nav.sh` | 机器人端安装脚本 (随 tarball 打包) | 随工程一起维护 |
| `scripts/ota/push_to_robot.sh` | 直连推送脚本 (开发/调试用) | 随工程一起维护 |
| `scripts/ota/manifest_template.json` | manifest 模板 (含 navigation 制品) | 新增制品类型时更新 |
| `scripts/ota/generate_manifest.py` | manifest 生成 + Ed25519 签名 | 已稳定 |
| `keys/ota_private.pem` | Ed25519 私钥 (**不入 git**) | 首次生成后妥善保管 |
| `.github/workflows/release-navigation.yml` | CI/CD 自动构建发布 | 随工程一起维护 |

### 15.2 机器人端目录结构

```
/opt/robot/
├── ota/
│   ├── installed_manifest.json    # 已安装制品记录
│   ├── ota_public.pem             # Ed25519 公钥
│   ├── ota_events.log             # OTA 事件日志
│   └── backup/
│       └── txn_navigation.json    # 事务日志 (安装完成后删除)
│
├── navigation/
│   ├── start_nav.sh               # systemd 启动入口
│   ├── current -> 1.0.0           # 当前版本 (软链接)
│   ├── previous -> 0.9.0          # 上一版本 (回滚用)
│   ├── 1.0.0/                     # 版本化安装
│   │   ├── install/               # colcon install 产物
│   │   │   ├── fastlio2/
│   │   │   ├── local_planner/
│   │   │   ├── terrain_analysis/
│   │   │   ├── ...
│   │   │   └── setup.bash
│   │   ├── launch/
│   │   │   ├── navigation_bringup.launch.py
│   │   │   └── navigation_run.launch.py
│   │   ├── fastdds_no_shm.xml
│   │   └── metadata.json          # 构建元数据
│   └── 0.9.0/                     # 旧版本 (保留最近3个)
│       └── ...
│
├── models/                         # ONNX 模型 (独立更新)
├── maps/                           # PCD 地图 (独立更新)
└── config/                         # YAML 配置 (独立更新)
```

### 15.3 首次部署

```bash
# 1. 在机器人上运行首次配置 (创建目录 + systemd 服务)
scp scripts/ota/setup_robot.sh scripts/ota/navigation.service \
    scripts/ota/start_nav.sh sunrise@robot:/tmp/
ssh sunrise@robot 'sudo bash /tmp/setup_robot.sh'

# 2. 生成签名密钥 (首次)
python3 scripts/ota/generate_manifest.py --generate-keys --key-dir ./keys/

# 3. 部署公钥到机器人
scp keys/ota_public.pem sunrise@robot:/opt/robot/ota/

# 4. 构建并推送首版
./scripts/ota/push_to_robot.sh sunrise@<robot_ip> --build --restart
```

### 15.4 日常更新工作流

#### 方式 A: 直连推送 (开发/调试，推荐)

```bash
# 修改代码后...

# 全量推送 (编译所有包 → 打包 → scp → 安装)
./scripts/ota/push_to_robot.sh sunrise@192.168.1.100 --build --restart

# 增量推送 (仅改了 local_planner)
./scripts/ota/push_to_robot.sh sunrise@192.168.1.100 \
    --build --packages-select local_planner --restart

# 极速同步 (已编译好，仅 rsync install/ 目录)
./scripts/ota/push_to_robot.sh sunrise@192.168.1.100 --sync-only --restart
```

#### 方式 B: GitHub Release (正式发布)

```bash
# 1. 更新版本号
echo "1.1.0" > VERSION

# 2. 提交并打 tag
git add . && git commit -m "release: v1.1.0"
git tag v1.1.0
git push origin main --tags

# CI/CD 自动: 构建 → 打包 → 签名 → 发布到 GitHub Release
# 机器人端: MapPilot App → 设置 → 云端更新 → 检查更新 → 安装
```

#### 方式 C: 手动 Release

```bash
# 1. 本地构建打包
./scripts/ota/build_nav_package.sh

# 2. 生成签名 manifest
python3 scripts/ota/generate_manifest.py \
    --version v1.1.0 \
    --artifacts-dir ./dist/ \
    --template scripts/ota/manifest_template.json \
    --signing-key ./keys/ota_private.pem \
    --output ./dist/manifest.json

# 3. 上传到 GitHub Release
gh release create v1.1.0 dist/* --title "v1.1.0" --notes "功能更新"
```

### 15.5 版本管理规范

```
VERSION 文件格式: major.minor.patch (semver)

major: 架构变更 / 不兼容更新 (如坐标系重构)
minor: 功能新增 (如新增避障算法)
patch: Bug 修复 / 参数调优

示例:
  1.0.0  — 首个稳定版本
  1.0.1  — 修复 terrain_analysis NaN 问题
  1.1.0  — 新增近场急停功能
  2.0.0  — SLAM 引擎从 Fast-LIO2 切换到 FAST-LIVO
```

### 15.6 回滚

```bash
# 方式 1: 在机器人端回滚
ssh sunrise@robot 'sudo /opt/robot/navigation/current/install_nav.sh --rollback'

# 方式 2: 通过 App 回滚
# MapPilot App → 设置 → OTA → 导航系统 → 回滚

# 方式 3: 从 OTA gRPC 接口
# Rollback(artifact_name="navigation")
```

### 15.7 安全级别

导航功能包更新属于 **COLD** 级别：
- 安装前必须停止导航服务 (自动处理)
- systemd 服务自动停止 → 安装 → 重启
- 事务日志保护: 安装中断 → 下次启动自动回滚

---

## 16. v3 增强: 版本一致性 + 可观测性 + 安全加固

> 本节记录 2026-02 实施的 Production OTA 升级，覆盖三个阶段共 10 项改进。

### 16.1 改动总览

| Phase | 改进项 | 涉及文件 | 状态 |
|-------|--------|---------|------|
| **P1.1** | 系统版本 (system_version.json) | `data.proto`, `data_service.cpp`, `generate_manifest.py`, `build_nav_package.sh` | **已完成** |
| **P1.2** | 设备端 Ed25519 验签 | `data_service.cpp` (OpenSSL `EVP_DigestVerify`) | **已完成** |
| **P1.3** | 安装后自动健康检查 + 自动回滚 | `data_service.cpp/hpp`, `grpc_gateway.cpp` | **已完成** |
| **P1.4** | semver 整数比较 + robot_id 初始化 | `data_service.cpp`, `grpc_gateway.cpp` | **已完成** |
| **P2.1** | 持久升级历史 (JSONL) + `GetUpgradeHistory` RPC | `data.proto`, `data_service.cpp` | **已完成** |
| **P2.2** | 标准化失败码 `OtaFailureCode` enum | `data.proto`, `data_service.cpp` 全链路 | **已完成** |
| **P2.3** | `ValidateSystemVersion` RPC | `data.proto`, `data_service.cpp` | **已完成** |
| **P3.1** | Flutter UI 补全 (版本列表/回滚/历史) | `firmware_ota_page.dart`, `robot_client.dart`, `robot_client_base.dart` | **已完成** |
| **P3.2** | 发布通道 dev/canary/stable | `manifest_template.json`, `generate_manifest.py`, `cloud_ota_service.dart` | **已完成** |
| **P3.3** | gRPC TLS (可选) | `grpc_gateway.cpp/hpp`, `grpc_gateway.yaml`, `robot_client.dart` | **已完成** |

### 16.2 Phase 1 — 版本一致性 + 设备端加固

#### P1.1 系统版本概念

**问题**: 之前只有零散的 `installed_manifest.json` 记录单个制品版本，无法回答 "这台机器人整体是什么版本？"

**方案**: 引入 `/opt/robot/ota/system_version.json`，每次 `ApplyUpdate` 成功后自动重建：

```json
{
  "system_version": "1.0.0",
  "components": {
    "navigation": {"version": "1.0.0"},
    "policy_walk": {"version": "2.3.0"},
    "campus_map": {"version": "1.0.0"}
  }
}
```

**改动**:
- `data.proto`: `GetInstalledVersionsResponse` 新增 `string system_version_json = 7`
- `data_service.cpp`: 新增 `LoadSystemVersionJson()` / `SaveSystemVersionJson()`
- `generate_manifest.py`: 新增 `--system-manifest` 参数，自动生成 `system_manifest.json`
- `build_nav_package.sh`: 打包时纳入 `system_manifest.json`
- `grpc_gateway.yaml`: 新增 `system_version_path` 配置项

#### P1.2 设备端 Ed25519 验签

**问题**: 签名仅在 Flutter App 端验证，绕过 App 直接调 gRPC 即可推伪造包。

**方案**: `data_service.cpp` 新增 `VerifyEd25519Signature()`:
- 使用 OpenSSL `EVP_DigestVerify` API
- 公钥路径: `/opt/robot/ota/ota_public.pem` (与 OTA_GUIDE 一致)
- `CheckUpdateReadiness` 新增 `signature` 检查项
- **向后兼容**: 未配置公钥时 → 跳过验签 (WARN 日志); 未来可切为强制阻塞

#### P1.3 安装后自动健康检查

**问题**: 装坏了不知道，只能人工发现。

**方案**: `ApplyUpdate` 成功后调用 `PostInstallHealthCheck()`:

| 安全等级 | 等待时间 | 检查方式 |
|---------|---------|---------|
| HOT | 2 秒 | 查 HealthMonitor 对应子系统 |
| WARM | 5 秒 | 查 HealthMonitor 整体 `GetOverallLevel()` |
| COLD | 跳过 | 由 systemd `ExecStartPre` 负责 (重启后检查) |

- 健康检查返回 `FAULT` / `CRITICAL` → **自动回滚** + 事务日志标记 `status=rolled_back_health_fail`
- `GrpcGateway` 构造时通过 `data_service_->SetHealthMonitor(health_monitor_)` 注入

#### P1.4 semver 比较 + robot_id

**问题 1**: `"1.10.0" < "1.2.0"` 字符串比较错误。
**修复**: 新增 `CompareSemver(a, b)` → 解析 `major.minor.patch` 为整数比较，支持 `v` 前缀。所有依赖检查均已替换。

**问题 2**: `SystemServiceImpl` 用默认 `robot_001`，未从 YAML 配置读取。
**修复**: `grpc_gateway.cpp` 从参数读取 `robot_id` / `firmware_version` 并传入构造函数。使用 `has_parameter()` 避免重复声明。

### 16.3 Phase 2 — 可观测性 + 标准化

#### P2.1 持久升级历史

**问题**: `txn_*.json` 是瞬态的 (安装成功即删除)，事后无法审计。

**方案**: 新增 `/opt/robot/ota/upgrade_history.jsonl` (append-only):

```jsonl
{"ts":"2026-02-09T10:30:00Z","action":"install","artifact":"navigation","from":"1.0.0","to":"1.1.0","status":"success","failure_code":0,"failure_reason":"","duration_ms":12345,"health_check":"passed"}
{"ts":"2026-02-09T11:00:00Z","action":"rollback","artifact":"navigation","from":"1.1.0","to":"1.0.0","status":"success","failure_code":0,"failure_reason":"","duration_ms":0,"health_check":"skipped"}
```

**记录时机**: `ApplyUpdate` 成功 / 失败 / 健康回滚 三条路径各记录一条；`Rollback` 成功也记录。

**查询 RPC**:
```protobuf
rpc GetUpgradeHistory(GetUpgradeHistoryRequest) returns (GetUpgradeHistoryResponse);
```
支持 `artifact_filter` 和 `limit`，返回 newest-first。

#### P2.2 标准化失败码

**问题**: `error_message` 是自由文本，无法统计分类。

**方案**: 新增 proto enum + 全链路接入:

```protobuf
enum OtaFailureCode {
  OTA_FAILURE_NONE = 0;
  OTA_FAILURE_NETWORK = 1;          // 下载失败
  OTA_FAILURE_SHA256_MISMATCH = 2;  // 校验失败
  OTA_FAILURE_SIGNATURE_INVALID = 3;// 签名无效
  OTA_FAILURE_DISK_FULL = 4;        // 磁盘不足
  OTA_FAILURE_DEPENDENCY = 5;       // 依赖不满足
  OTA_FAILURE_INSTALL_SCRIPT = 6;   // 安装脚本失败
  OTA_FAILURE_HEALTH_CHECK = 7;     // 安装后健康检查失败
  OTA_FAILURE_BATTERY_LOW = 8;      // 电量不足
  OTA_FAILURE_HW_INCOMPAT = 9;     // 硬件不兼容
  OTA_FAILURE_SAFETY_MODE = 10;     // 安全模式不允许
  OTA_FAILURE_PERMISSION = 11;      // 权限不足
  OTA_FAILURE_ROLLBACK_FAILED = 12; // 回滚也失败
}
```

`ApplyUpdateResponse` 新增 `OtaFailureCode failure_code = 7`。所有失败路径 (SHA256 / HW / 依赖 / 安装脚本 / 健康检查) 均已赋值。

#### P2.3 版本一致性校验 RPC

**用途**: 客户端传入期望的系统版本和组件版本集，机器人端逐一对比：

```protobuf
rpc ValidateSystemVersion(ValidateSystemVersionRequest) returns (ValidateSystemVersionResponse);
```

返回 `consistent` (bool) + 每个组件的 `match` / `mismatch` / `missing` / `extra` 状态。

### 16.4 Phase 3 — 用户体验 + 发布通道 + 安全传输

#### P3.1 Flutter UI 补全

**问题**: `firmware_ota_page.dart` 只用了 `applyFirmware()`，其余 5 个 OTA RPC 未接入 UI。

**补全内容**:

| 功能 | 使用 RPC | UI 位置 |
|------|---------|---------|
| 已安装版本列表 | `getInstalledVersions()` | 新增 "已安装版本" 区域 |
| 一键回滚 | `rollback()` | 每个制品旁的 "回滚" 按钮 |
| 升级历史 | `getUpgradeHistory()` | 新增 "升级历史" 区域 (newest-first) |
| 安装前预检 | `checkUpdateReadiness()` | 点击 "应用固件" 时自动执行 |
| 系统版本显示 | `getInstalledVersions().systemVersion` | 设备状态区域 |

#### P3.2 发布通道 (dev / canary / stable)

**实现**:
- `manifest_template.json`: 顶层新增 `"channel": "stable"`
- `generate_manifest.py`: 新增 `--channel` 参数 (dev / canary / stable)
- `cloud_ota_service.dart`:
  - `CloudOtaManifest` 新增 `channel` 字段
  - `_channel` 持久化到 `SharedPreferences`
  - `fetchLatestRelease()` 按 channel 过滤:
    - `stable` → GitHub API `/latest` (仅非 prerelease)
    - `dev` / `canary` → 扫描全部 releases，按 manifest `channel` 或 `prerelease` 标记匹配

**使用方式**:
```bash
# CI 发布 canary 版
python3 scripts/ota/generate_manifest.py --version v1.1.0-beta1 --channel canary ...

# CI 发布 stable 版
python3 scripts/ota/generate_manifest.py --version v1.1.0 --channel stable ...
```

App 端在设置中选择更新通道 (stable / dev)，检查更新时自动过滤。

#### P3.3 gRPC TLS

**实现**:
- `grpc_gateway.yaml`: 新增 `tls_cert_path` / `tls_key_path` (默认空 = 不启用)
- `grpc_gateway.cpp`: 当两个路径均配置且可读时，使用 `grpc::SslServerCredentials`
- `robot_client.dart`: 构造函数新增 `useTls` / `trustedCertificateBytes` 参数

**证书方案**: 每台机器人自签证书，App 端 **trust-on-first-use** (类似 SSH):
```bash
# 在机器人上生成自签证书
openssl req -x509 -newkey ed25519 -keyout /opt/robot/certs/server.key \
  -out /opt/robot/certs/server.crt -days 3650 -nodes \
  -subj "/CN=$(hostname)"

# 配置 grpc_gateway.yaml
tls_cert_path: "/opt/robot/certs/server.crt"
tls_key_path: "/opt/robot/certs/server.key"
```

**向后兼容**: 不配置证书路径时保持明文 gRPC，无需任何改动。

### 16.5 v3 新增目录结构

```
/opt/robot/ota/
├── installed_manifest.json      # [已有] 已安装制品记录
├── ota_public.pem               # [已有] Ed25519 公钥
├── system_version.json          # [P1.1 新增] 整机组件版本集
├── upgrade_history.jsonl        # [P2.1 新增] 持久升级历史 (append-only)
└── backup/
    └── txn_*.json               # [已有] 事务日志
```

### 16.6 v3 新增 Proto 定义

```protobuf
// ---- 枚举 ----
enum OtaFailureCode { ... }                         // P2.2: 13 个标准失败码

// ---- 新增 RPC ----
rpc GetUpgradeHistory(...)   returns (...);          // P2.1: 升级历史查询
rpc ValidateSystemVersion(...) returns (...);        // P2.3: 版本一致性校验

// ---- 新增/扩展消息 ----
ApplyUpdateResponse        { + failure_code = 7 }   // P2.2
GetInstalledVersionsResponse { + system_version_json = 7 } // P1.1
UpgradeHistoryEntry        { ... }                   // P2.1
ComponentVersion           { ... }                   // P2.3
VersionMismatch            { ... }                   // P2.3
```

### 16.7 安全考虑 (v3 更新)

| 威胁 | 防御 | 状态 |
|------|------|------|
| manifest 伪造 | Ed25519 签名验证 | **App + 设备端双重验证** |
| 制品篡改 | SHA256 校验 | 已实现 |
| 安装后故障 | HealthMonitor 自动健康检查 + 自动回滚 | **已实现** |
| 版本混乱 | `system_version.json` + `ValidateSystemVersion` RPC | **已实现** |
| 事后审计 | `upgrade_history.jsonl` 持久日志 | **已实现** |
| 失败统计 | `OtaFailureCode` 标准化 | **已实现** |
| 中间人攻击 | gRPC TLS (自签证书, trust-on-first-use) | **已实现 (可选启用)** |
| semver 误判 | 整数解析比较 (不再用字符串) | **已修复** |

---

## 17. 待开展工作 (Roadmap)

### 17.1 近期 (Next Sprint)

| 项目 | 说明 | 优先级 |
|------|------|--------|
| ModeManager 强制检查 | COLD 更新时强制要求 `mode == IDLE \|\| mode == ESTOP`，当前仅 WARN 日志 | P0 |
| Ed25519 验签切为强制 | 当前未签名 manifest 仍可通过 (向后兼容)；正式环境应阻塞 | P1 |
| Flutter 设置页增加通道选择 | App Settings → "更新通道" 下拉 (stable / dev)，当前仅代码层支持 | P1 |
| 升级历史 UI 详情页 | 点击历史条目展开失败原因、耗时、健康检查结果 | P2 |
| TLS 证书自动交换 | App 首次连接时弹出指纹确认对话框 (trust-on-first-use 完整实现) | P2 |

### 17.2 中期 (v4 规划)

| 项目 | 说明 |
|------|------|
| 增量/Delta 更新 | 仅传输变更部分，减少 navigation tarball 大小 (bsdiff / rsync) |
| A/B 分区 | rootfs/MCU 双分区无缝切换，彻底消除更新风险 |
| 自动冒烟测试 | 安装后自动运行一组 ROS2 smoke test (Topic 频率 / TF 有效 / 规划器响应) |
| 多设备批量管理 | 当机器人数量 >5 时，App 单机模式不够用；需要轻量级 fleet dashboard |
| OTA 指标上报 | 更新成功率、平均耗时、失败分布等指标推送到 InfluxDB/Grafana |

### 17.3 长期 (架构演进)

| 项目 | 说明 |
|------|------|
| 独立 OTA Agent 进程 | 将 OTA 逻辑从 gRPC Gateway 拆分为独立 daemon，支持自升级 |
| 服务端 Update Orchestrator | 当车队规模 >20 时，引入服务端控制面 (灰度发布 / 自动回滚熔断) |
| 安全启动链 | Secure Boot → 签名内核 → 签名 rootfs → 签名应用层 |
| 配置热更新 (HOT reload) | `config_service` 接收参数变更 → ROS2 dynamic_reconfigure → 零停机生效 |
| owner_module 委托安装 | OTA DataService 不再直接执行安装，而是通过 ROS2 Action 委托给对应模块 |

---

*最后更新: 2026-02-09*
