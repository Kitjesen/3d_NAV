# OTA (Over-The-Air) 更新系统指南

## 1. 概述

本系统支持通过 gRPC 远程更新机器人上的模型、固件、地图和配置文件。

### 架构

```
┌─────────────────────┐          ┌─────────────────────┐
│   GitHub Releases   │          │  Flutter 客户端      │
│  ┌───────────────┐  │  HTTP    │  ┌───────────────┐  │
│  │ manifest.json │◄─┼──────────┼──│ CloudOtaService│  │
│  │ *.onnx        │  │          │  │ (解析manifest) │  │
│  │ *.deb         │  │          │  └───────┬───────┘  │
│  │ *.pcd         │  │          │          │ gRPC     │
│  └───────────────┘  │          │          ▼          │
└─────────────────────┘          │  ┌───────────────┐  │
                                 │  │ RobotClient   │  │
                                 │  │ UploadFile()  │  │
                                 │  │ ApplyUpdate() │  │
                                 │  └───────────────┘  │
                                 └──────────┬──────────┘
                                            │ gRPC
                                            ▼
                                 ┌──────────────────────┐
                                 │  Nav Board (Gateway)  │
                                 │  ┌────────────────┐  │
                                 │  │ DataService     │  │
                                 │  │ ─ ApplyUpdate() │  │
                                 │  │ ─ SHA256 校验   │  │
                                 │  │ ─ 备份+回滚     │  │
                                 │  │ ─ manifest管理  │  │
                                 │  └────────────────┘  │
                                 └──────────────────────┘
```

### OTA 生命周期

```
1. 检查更新 (Check)
   └── 客户端从 GitHub Releases 拉取 manifest.json
   └── 对比本地已安装版本 (GetInstalledVersions)

2. 预检查 (Pre-flight)
   └── CheckUpdateReadiness: 磁盘空间、电量、硬件兼容性、网络

3. 下载到机器人 (两种方式)
   ├── 方式A: DownloadFromUrl → 机器人直接从 GitHub 下载（推荐，免手机中转）
   └── 方式B: 手机下载 → UploadFile（支持断点续传）推送到机器人

4. 校验+安装 (Verify & Install)
   └── ApplyUpdate: SHA256 → 硬件兼容性 → 备份旧版本 → 安装

5. 验证 (Validate)
   └── 客户端通过 GetInstalledVersions 确认版本已更新

6. 回滚 (Rollback)
   └── 如有问题，调用 Rollback 恢复上一版本
```

---

## 2. 云端 manifest.json 格式

每个 GitHub Release 中应包含一个 `manifest.json` 文件，描述该版本包含的所有制品。

### 完整示例

```json
{
  "schema_version": "1",
  "release_version": "v2.3.0",
  "release_date": "2026-02-07T12:00:00Z",
  "min_system_version": "1.0.0",
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
      "rollback_safe": true
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
      "rollback_safe": false
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
      "rollback_safe": true
    },
    {
      "name": "terrain_config",
      "category": "config",
      "version": "1.1.0",
      "filename": "terrain_params.yaml",
      "sha256": "abcd1234...64hex...",
      "target_path": "/opt/robot/config/terrain_params.yaml",
      "target_board": "nav",
      "hw_compat": ["*"],
      "apply_action": "copy_only",
      "requires_reboot": false,
      "min_battery_percent": 0,
      "changelog": "调整障碍物高度阈值",
      "rollback_safe": true
    }
  ]
}
```

### 字段说明

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `schema_version` | string | 是 | 固定为 "1" |
| `release_version` | string | 是 | Release 版本号 (semver) |
| `release_date` | string | 是 | ISO 8601 日期 |
| `min_system_version` | string | 否 | 机器人最低系统版本要求 |
| `artifacts[].name` | string | 是 | 制品唯一名称 (英文, 无空格) |
| `artifacts[].category` | string | 是 | `model` / `firmware` / `map` / `config` |
| `artifacts[].version` | string | 是 | semver 版本号 |
| `artifacts[].filename` | string | 是 | Release Asset 中对应的文件名 |
| `artifacts[].sha256` | string | 是 | 文件的 SHA256 校验值 |
| `artifacts[].target_path` | string | 是 | 安装到机器人上的目标路径 |
| `artifacts[].target_board` | string | 否 | `nav` (导航板) 或 `dog` (狗控板), 默认 `nav` |
| `artifacts[].hw_compat` | string[] | 否 | 兼容硬件列表, `["*"]` = 全部, 默认 `["*"]` |
| `artifacts[].apply_action` | string | 否 | 安装动作, 见下表, 默认 `copy_only` |
| `artifacts[].requires_reboot` | bool | 否 | 安装后是否需要重启, 默认 `false` |
| `artifacts[].min_battery_percent` | int | 否 | 最低电量要求, 0=不限, 默认 `0` |
| `artifacts[].changelog` | string | 否 | 变更说明 |
| `artifacts[].rollback_safe` | bool | 否 | 是否支持回滚, 默认 `true` |

### apply_action 枚举

| 值 | 说明 | 适用场景 |
|----|------|---------|
| `copy_only` | 仅复制到 target_path | 地图、配置、小模型 |
| `reload_model` | 复制后通知 Dog Board 热加载 | ONNX 模型 |
| `install_deb` | dpkg -i 安装 | DEB 系统包 |
| `flash_mcu` | 调用刷写脚本烧录 MCU | STM32 固件 |
| `install_script` | 执行自定义安装脚本 | 复杂部署 |

---

## 3. GitHub Release 发布流程

### 手动发布

1. 生成 manifest.json（参考上方格式）
2. 计算每个文件的 SHA256：
   ```bash
   sha256sum policy_walk_v2.3.onnx
   # 输出: a1b2c3d4... policy_walk_v2.3.onnx
   ```
3. 在 GitHub 创建 Release，上传所有文件 + manifest.json

### CI/CD 自动发布 (GitHub Actions)

```yaml
# .github/workflows/release.yml
name: OTA Release

on:
  push:
    tags:
      - 'v*'

jobs:
  release:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Build artifacts
        run: |
          # 编译模型、打包 deb 等
          make build-artifacts

      - name: Generate manifest
        run: |
          python3 scripts/generate_manifest.py \
            --version ${{ github.ref_name }} \
            --artifacts-dir ./dist/ \
            --output ./dist/manifest.json

      - name: Create GitHub Release
        uses: softprops/action-gh-release@v2
        with:
          files: |
            dist/*.onnx
            dist/*.deb
            dist/*.pcd
            dist/*.yaml
            dist/manifest.json
          body_path: CHANGELOG.md
```

---

## 4. gRPC 接口

### DownloadFromUrl (机器人直接下载，免手机中转)

```protobuf
rpc DownloadFromUrl(DownloadFromUrlRequest) returns (stream OtaProgress);
```

机器人通过 curl 直接从 GitHub Release URL 下载文件，返回实时进度流。
**关键优势**: 200MB ONNX 模型不再经过手机中转，速度提升 2-3x。

### UploadFile (支持断点续传)

```protobuf
rpc UploadFile(stream UploadFileChunk) returns (UploadFileResponse);
```

UploadFileMetadata 新增 `resume_from_offset` 字段。WiFi 中断后客户端可从断点继续。
UploadFileResponse 返回服务端计算的 SHA256，可用于校验。

### CheckUpdateReadiness (安装前预检查)

```protobuf
rpc CheckUpdateReadiness(CheckUpdateReadinessRequest) returns (CheckUpdateReadinessResponse);
```

检查项: 磁盘空间、电量、硬件兼容性、网络连通性。

### ApplyUpdate

```protobuf
rpc ApplyUpdate(ApplyUpdateRequest) returns (ApplyUpdateResponse);
```

流程：SHA256 校验 → 硬件兼容性检查 → 备份旧版本 → 安装 → 更新本地 manifest

### GetInstalledVersions

```protobuf
rpc GetInstalledVersions(GetInstalledVersionsRequest) returns (GetInstalledVersionsResponse);
```

返回已安装制品列表 + 可回滚版本列表 + 机器人硬件 ID

### Rollback

```protobuf
rpc Rollback(RollbackRequest) returns (RollbackResponse);
```

从备份中恢复上一版本

---

## 5. 机器人端目录结构

```
/opt/robot/
├── ota/
│   ├── installed_manifest.json    # 已安装制品记录
│   └── backup/                    # 旧版本备份
│       ├── policy_walk_20260207_120000.onnx
│       └── terrain_params_20260207_120000.yaml
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

## 6. 客户端 OTA 调用示例 (Flutter/Dart)

### 方式 A: 机器人直接下载（推荐，适合大文件）

```dart
// 1. 检查更新
final release = await cloudOtaService.fetchLatestRelease();
if (release?.hasManifest != true) return;

// 2. 预检查
final otaArtifacts = release!.manifest!.artifacts.map(_toOtaArtifact).toList();
final readiness = await robotClient.checkUpdateReadiness(artifacts: otaArtifacts);
if (!readiness.ready) {
  for (final check in readiness.checks.where((c) => !c.passed)) {
    print('预检查失败: ${check.checkName} - ${check.message}');
  }
  return;
}

// 3. 查询已安装版本
final installed = await robotClient.getInstalledVersions();

// 4. 对比版本, 找出需要更新的制品
for (final artifact in release.manifest!.artifacts) {
  final current = installed.installed
      .where((i) => i.name == artifact.name)
      .firstOrNull;
  if (current != null && current.version == artifact.version) continue;

  // 5. 机器人直接从 GitHub 下载 (免手机中转!)
  final asset = release.assets.firstWhere((a) => a.name == artifact.filename);
  final stagingPath = '/tmp/ota_staging/${artifact.filename}';

  await for (final progress in robotClient.downloadFromUrl(
    url: asset.downloadUrl,
    stagingPath: stagingPath,
    expectedSha256: artifact.sha256,
    expectedSize: asset.size,
  )) {
    print('下载: ${progress.progressPercent.toStringAsFixed(1)}% - ${progress.message}');
    if (progress.status == OtaUpdateStatus.OTA_UPDATE_STATUS_FAILED) {
      throw Exception('下载失败: ${progress.message}');
    }
  }

  // 6. ApplyUpdate
  final result = await robotClient.applyUpdate(
      artifact: _toOtaArtifact(artifact), stagedPath: stagingPath);
  print('${artifact.name}: ${result.success ? "成功" : result.message}');
}
```

### 方式 B: 手机中转上传（适合无外网的机器人）

```dart
// 手机先下载到内存
final bytes = await cloudOtaService.downloadAsset(asset,
    onProgress: (p) => print('下载到手机: ${(p * 100).toInt()}%'));

// 上传到机器人 (支持断点续传)
final stagingPath = '/tmp/ota_staging/${artifact.filename}';
final uploadResp = await robotClient.uploadFile(
    localBytes: bytes,
    remotePath: stagingPath,
    filename: artifact.filename,
    category: artifact.category,
    sha256: artifact.sha256,
    // WiFi 中断后可以从上次位置继续:
    // resumeFromOffset: lastReceivedOffset,
    onProgress: (p) => print('上传到机器人: ${(p * 100).toInt()}%'));

// 服务端返回 SHA256, 可以二次验证
assert(uploadResp.sha256 == artifact.sha256);
```

---

## 7. 安全考虑

- **SHA256 校验**: 每个制品必须有 SHA256，服务端安装前强制校验
- **路径遍历防护**: 服务端禁止 `..` 路径组件
- **硬件兼容性**: manifest 中 `hw_compat` 防止错误刷写
- **电量检查**: `min_battery_percent` 防止刷写中断
- **回滚支持**: `rollback_safe=true` 的制品安装前自动备份
- **无密钥上传**: gRPC channel 后续应启用 TLS

---

*最后更新: 2026-02-07*
