import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:file_picker/file_picker.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/features/settings/cloud_ota_service.dart';

class FirmwareOtaPage extends StatefulWidget {
  const FirmwareOtaPage({super.key});

  @override
  State<FirmwareOtaPage> createState() => _FirmwareOtaPageState();
}

class _FirmwareOtaPageState extends State<FirmwareOtaPage> {
  // ---- 本地文件上传 ----
  bool _isUploading = false;
  double _uploadProgress = 0.0;
  String? _statusMessage;
  String? _selectedFileName;
  String? _lastUploadedRemotePath;

  // ---- 云端更新检查 ----
  final CloudOtaService _cloudService = CloudOtaService();
  bool _isCheckingCloud = false;
  CloudRelease? _latestRelease;
  String? _cloudError;
  bool _isDownloadingAsset = false;
  double _downloadProgress = 0.0;
  String? _downloadingAssetName;

  // ---- 应用固件 ----
  bool _isApplying = false;

  @override
  void initState() {
    super.initState();
    _cloudService.loadConfig();
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final provider = context.watch<RobotConnectionProvider>();
    final isConnected = provider.isConnected;
    final slowState = provider.latestSlowState;
    final currentVersion = slowState?.currentMode ?? '未知';
    final battery = slowState?.resources.batteryPercent ?? 0.0;

    return Scaffold(
      appBar: AppBar(
        title: const Text('固件升级'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 18),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: ListView(
        physics: const BouncingScrollPhysics(),
        padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
        children: [
          // ===== 设备状态 =====
          _buildDeviceStatusSection(isDark, isConnected, currentVersion, battery),
          const SizedBox(height: 24),

          // ===== 状态提示 =====
          if (!isConnected) _buildNotice('请先连接机器人', NoticeType.warning),
          if (isConnected && battery < 30)
            _buildNotice('电量低于 30%，建议充电后再升级', NoticeType.error),
          if (_statusMessage != null)
            _buildNotice(
              _statusMessage!,
              _statusMessage!.contains('成功')
                  ? NoticeType.success
                  : _statusMessage!.contains('失败')
                      ? NoticeType.error
                      : NoticeType.info,
            ),

          // ===== 上传进度 =====
          if (_isUploading) ...[
            _buildProgressSection('正在上传固件...', _uploadProgress,
                subtitle: _selectedFileName),
            const SizedBox(height: 24),
          ],

          // ===== 本地固件 =====
          _buildSectionTitle('本地固件'),
          const SizedBox(height: 8),
          _buildLocalFirmwareSection(isDark, isConnected, battery),
          const SizedBox(height: 28),

          // ===== 云端更新 =====
          _buildSectionTitle('云端更新'),
          const SizedBox(height: 8),
          _buildCloudSection(isDark, isConnected, battery),
          const SizedBox(height: 28),

          // ===== 升级须知 =====
          _buildSectionTitle('升级须知'),
          const SizedBox(height: 8),
          _buildInfoSection(isDark),

          const SizedBox(height: 40),
        ],
      ),
    );
  }

  // ================================================================
  // 设备状态
  // ================================================================

  Widget _buildDeviceStatusSection(
      bool isDark, bool isConnected, String currentVersion, double battery) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        border: Border.all(color: isDark ? AppColors.borderDark : AppColors.borderLight),
      ),
      child: Row(
        children: [
          // 固件版本
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  '当前固件',
                  style: TextStyle(
                    fontSize: 12,
                    color: context.subtitleColor,
                  ),
                ),
                const SizedBox(height: 4),
                Text(
                  currentVersion,
                  style: TextStyle(
                    fontSize: 17,
                    fontWeight: FontWeight.w600,
                    color: context.titleColor,
                    letterSpacing: -0.3,
                  ),
                ),
              ],
            ),
          ),
          // 状态指示器
          Row(
            mainAxisSize: MainAxisSize.min,
            children: [
              _StatusDot(
                label: isConnected ? '已连接' : '未连接',
                color: isConnected ? AppColors.success : const Color(0xFF86868B),
              ),
              const SizedBox(width: 16),
              _StatusDot(
                label: '${battery.toStringAsFixed(0)}%',
                color: battery > 30 ? AppColors.success : AppColors.error,
              ),
            ],
          ),
        ],
      ),
    );
  }

  // ================================================================
  // 本地固件
  // ================================================================

  Widget _buildLocalFirmwareSection(bool isDark, bool isConnected, double battery) {
    return Container(
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        border: Border.all(color: isDark ? AppColors.borderDark : AppColors.borderLight),
      ),
      child: Column(
        children: [
          // 上传按钮
          InkWell(
            borderRadius: BorderRadius.circular(AppRadius.card),
            onTap: isConnected && !_isUploading && battery >= 30
                ? () => _selectAndUploadFirmware(context)
                : null,
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
              child: Row(
                children: [
                  Expanded(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        Text(
                          '选择固件文件并上传',
                          style: TextStyle(
                            fontSize: 14,
                            fontWeight: FontWeight.w500,
                            color: isConnected && battery >= 30
                                ? context.titleColor
                                : context.subtitleColor,
                          ),
                        ),
                        const SizedBox(height: 2),
                        Text(
                          '支持 .bin / .fw / .zip / .deb 格式',
                          style: TextStyle(
                            fontSize: 12,
                            color: context.subtitleColor,
                          ),
                        ),
                      ],
                    ),
                  ),
                  Icon(
                    Icons.upload_outlined,
                    size: 18,
                    color: isConnected && battery >= 30
                        ? AppColors.primary
                        : context.subtitleColor,
                  ),
                ],
              ),
            ),
          ),

          // 应用固件按钮（上传成功后出现）
          if (_lastUploadedRemotePath != null && !_isUploading) ...[
            Divider(
              height: 1,
              color: isDark ? AppColors.borderDark : AppColors.borderLight,
            ),
            InkWell(
              borderRadius: const BorderRadius.only(
                bottomLeft: Radius.circular(12),
                bottomRight: Radius.circular(12),
              ),
              onTap: isConnected && !_isApplying ? () => _applyFirmware() : null,
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
                child: Row(
                  children: [
                    Expanded(
                      child: Text(
                        _isApplying ? '正在触发刷写...' : '应用固件到机器人',
                        style: TextStyle(
                          fontSize: 14,
                          fontWeight: FontWeight.w500,
                          color: AppColors.warning,
                        ),
                      ),
                    ),
                    if (_isApplying)
                      const SizedBox(
                        width: 16,
                        height: 16,
                        child: CircularProgressIndicator(
                          strokeWidth: 2,
                          color: AppColors.warning,
                        ),
                      )
                    else
                      Icon(
                        Icons.play_arrow_outlined,
                        size: 18,
                        color: AppColors.warning,
                      ),
                  ],
                ),
              ),
            ),
          ],
        ],
      ),
    );
  }

  // ================================================================
  // 云端更新
  // ================================================================

  Widget _buildCloudSection(bool isDark, bool isConnected, double battery) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        // 仓库来源 + 检查按钮
        Container(
          decoration: BoxDecoration(
            color: isDark ? AppColors.darkCard : Colors.white,
            borderRadius: BorderRadius.circular(AppRadius.card),
            border: Border.all(
                color: isDark ? AppColors.borderDark : AppColors.borderLight),
          ),
          child: Column(
            children: [
              // 仓库来源
              Padding(
                padding: const EdgeInsets.fromLTRB(16, 14, 16, 0),
                child: Row(
                  children: [
                    Expanded(
                      child: Column(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Text(
                            '更新源',
                            style: TextStyle(
                              fontSize: 12,
                              color: context.subtitleColor,
                            ),
                          ),
                          const SizedBox(height: 2),
                          Text(
                            _cloudService.repoDisplay,
                            style: TextStyle(
                              fontSize: 14,
                              fontWeight: FontWeight.w500,
                              color: context.titleColor,
                            ),
                          ),
                        ],
                      ),
                    ),
                    Text(
                      'GitHub Releases',
                      style: TextStyle(
                        fontSize: 11,
                        color: context.subtitleColor,
                      ),
                    ),
                  ],
                ),
              ),

              Padding(
                padding: const EdgeInsets.fromLTRB(16, 12, 16, 14),
                child: SizedBox(
                  width: double.infinity,
                  height: 40,
                  child: TextButton(
                    onPressed: _isCheckingCloud ? null : _checkCloudUpdate,
                    style: TextButton.styleFrom(
                      backgroundColor: isDark
                          ? Colors.white.withOpacity(0.05)
                          : Colors.black.withOpacity(0.03),
                      foregroundColor: context.titleColor,
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(8),
                      ),
                    ),
                    child: _isCheckingCloud
                        ? SizedBox(
                            width: 16,
                            height: 16,
                            child: CircularProgressIndicator(
                              strokeWidth: 2,
                              color: context.subtitleColor,
                            ),
                          )
                        : const Text(
                            '检查最新版本',
                            style: TextStyle(fontSize: 13, fontWeight: FontWeight.w500),
                          ),
                  ),
                ),
              ),
            ],
          ),
        ),

        if (_cloudError != null) ...[
          const SizedBox(height: 12),
          _buildNotice(_cloudError!, NoticeType.error),
        ],

        // 最新版本信息
        if (_latestRelease != null) ...[
          const SizedBox(height: 16),
          _buildReleaseCard(isDark, isConnected, battery),
        ],
      ],
    );
  }

  Widget _buildReleaseCard(bool isDark, bool isConnected, double battery) {
    final release = _latestRelease!;
    final firmwareAssets =
        release.assets.where((a) => a.category == 'firmware').toList();

    return Container(
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        border: Border.all(
            color: isDark ? AppColors.borderDark : AppColors.borderLight),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Header: 版本号 + 日期
          Padding(
            padding: const EdgeInsets.fromLTRB(16, 14, 16, 0),
            child: Row(
              children: [
                // 版本 tag
                Container(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 8, vertical: 3),
                  decoration: BoxDecoration(
                    color: AppColors.primary.withOpacity(isDark ? 0.15 : 0.08),
                    borderRadius: BorderRadius.circular(6),
                  ),
                  child: Text(
                    release.tagName,
                    style: const TextStyle(
                      fontSize: 12,
                      fontWeight: FontWeight.w600,
                      color: AppColors.primary,
                    ),
                  ),
                ),
                if (release.prerelease) ...[
                  const SizedBox(width: 6),
                  Container(
                    padding:
                        const EdgeInsets.symmetric(horizontal: 6, vertical: 2),
                    decoration: BoxDecoration(
                      color: AppColors.warning.withOpacity(0.1),
                      borderRadius: BorderRadius.circular(6),
                    ),
                    child: Text(
                      'Pre',
                      style: TextStyle(
                        fontSize: 10,
                        fontWeight: FontWeight.w600,
                        color: AppColors.warning,
                      ),
                    ),
                  ),
                ],
                const Spacer(),
                Text(
                  release.publishedDate,
                  style: TextStyle(fontSize: 11, color: context.subtitleColor),
                ),
              ],
            ),
          ),

          // Release 名称
          if (release.name.isNotEmpty)
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 10, 16, 0),
              child: Text(
                release.name,
                style: TextStyle(
                  fontSize: 14,
                  fontWeight: FontWeight.w600,
                  color: context.titleColor,
                ),
              ),
            ),

          // Release 描述
          if (release.body.isNotEmpty)
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 6, 16, 0),
              child: Text(
                release.body.length > 200
                    ? '${release.body.substring(0, 200)}...'
                    : release.body,
                style: TextStyle(
                  fontSize: 12,
                  color: context.subtitleColor,
                  height: 1.5,
                ),
              ),
            ),

          const SizedBox(height: 12),

          // 固件资产列表
          if (firmwareAssets.isEmpty)
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 0, 16, 14),
              child: Text(
                '此版本无固件文件',
                style: TextStyle(fontSize: 12, color: context.subtitleColor),
              ),
            )
          else ...[
            Divider(
              height: 1,
              color: isDark ? AppColors.borderDark : AppColors.borderLight,
            ),
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 10, 16, 4),
              child: Text(
                '固件文件（${firmwareAssets.length}）',
                style: TextStyle(
                  fontSize: 11,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                  letterSpacing: 0.3,
                ),
              ),
            ),
            ...firmwareAssets.map((asset) => _buildAssetRow(
                  asset,
                  isDark,
                  canUpload: isConnected && battery >= 30,
                )),
            const SizedBox(height: 8),
          ],

          // 其他可部署资产
          if (release.deployableAssets.length > firmwareAssets.length) ...[
            Divider(
              height: 1,
              color: isDark ? AppColors.borderDark : AppColors.borderLight,
            ),
            Padding(
              padding: const EdgeInsets.fromLTRB(16, 10, 16, 4),
              child: Text(
                '其他资源',
                style: TextStyle(
                  fontSize: 11,
                  fontWeight: FontWeight.w600,
                  color: context.subtitleColor,
                  letterSpacing: 0.3,
                ),
              ),
            ),
            ...release.deployableAssets
                .where((a) => a.category != 'firmware')
                .map((asset) => _buildAssetRow(
                      asset,
                      isDark,
                      canUpload: isConnected,
                    )),
            const SizedBox(height: 8),
          ],
        ],
      ),
    );
  }

  Widget _buildAssetRow(CloudAsset asset, bool isDark,
      {required bool canUpload}) {
    final isThisDownloading =
        _isDownloadingAsset && _downloadingAssetName == asset.name;

    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 6),
      child: Column(
        children: [
          Row(
            children: [
              // 分类小圆点
              Container(
                width: 6,
                height: 6,
                decoration: BoxDecoration(
                  color: _categoryColor(asset.category),
                  shape: BoxShape.circle,
                ),
              ),
              const SizedBox(width: 10),
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    Text(
                      asset.name,
                      style: TextStyle(
                        fontSize: 13,
                        fontWeight: FontWeight.w500,
                        color: context.titleColor,
                      ),
                      overflow: TextOverflow.ellipsis,
                    ),
                    Text(
                      asset.formattedSize,
                      style: TextStyle(
                        fontSize: 11,
                        color: context.subtitleColor,
                      ),
                    ),
                  ],
                ),
              ),
              const SizedBox(width: 8),
              // 部署按钮
              SizedBox(
                height: 28,
                child: TextButton(
                  onPressed: canUpload && !_isDownloadingAsset
                      ? () => _downloadAndUploadAsset(asset)
                      : null,
                  style: TextButton.styleFrom(
                    padding: const EdgeInsets.symmetric(horizontal: 12),
                    backgroundColor: isDark
                        ? Colors.white.withOpacity(0.06)
                        : Colors.black.withOpacity(0.04),
                    foregroundColor: AppColors.primary,
                    disabledForegroundColor: context.subtitleColor,
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(6),
                    ),
                    textStyle: const TextStyle(
                        fontSize: 12, fontWeight: FontWeight.w500),
                  ),
                  child: isThisDownloading
                      ? SizedBox(
                          width: 12,
                          height: 12,
                          child: CircularProgressIndicator(
                            strokeWidth: 1.5,
                            color: AppColors.primary,
                          ),
                        )
                      : const Text('部署'),
                ),
              ),
            ],
          ),
          if (isThisDownloading) ...[
            const SizedBox(height: 6),
            ClipRRect(
              borderRadius: BorderRadius.circular(2),
              child: LinearProgressIndicator(
                value: _downloadProgress,
                minHeight: 2,
                backgroundColor: isDark
                    ? Colors.white.withOpacity(0.06)
                    : Colors.black.withOpacity(0.04),
              ),
            ),
          ],
        ],
      ),
    );
  }

  // ================================================================
  // 上传进度
  // ================================================================

  Widget _buildProgressSection(String title, double progress,
      {String? subtitle}) {
    final isDark = context.isDark;
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        border: Border.all(
            color: isDark ? AppColors.borderDark : AppColors.borderLight),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Expanded(
                child: Text(
                  title,
                  style: TextStyle(
                    fontSize: 14,
                    fontWeight: FontWeight.w500,
                    color: context.titleColor,
                  ),
                ),
              ),
              Text(
                '${(progress * 100).toStringAsFixed(0)}%',
                style: const TextStyle(
                  fontSize: 13,
                  fontWeight: FontWeight.w600,
                  color: AppColors.primary,
                ),
              ),
            ],
          ),
          if (subtitle != null) ...[
            const SizedBox(height: 2),
            Text(
              subtitle,
              style: TextStyle(fontSize: 12, color: context.subtitleColor),
            ),
          ],
          const SizedBox(height: 10),
          ClipRRect(
            borderRadius: BorderRadius.circular(3),
            child: LinearProgressIndicator(
              value: progress,
              minHeight: 4,
              backgroundColor: isDark
                  ? Colors.white.withOpacity(0.06)
                  : Colors.black.withOpacity(0.04),
            ),
          ),
        ],
      ),
    );
  }

  // ================================================================
  // 升级须知
  // ================================================================

  Widget _buildInfoSection(bool isDark) {
    final items = [
      '确保机器人电量 >= 30%',
      '升级过程中请勿断开连接',
      '确保机器人处于静止状态',
      '支持 .bin / .fw / .zip / .deb 格式',
      '上传完成后需点击"应用固件"触发刷写',
    ];

    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(AppRadius.card),
        border: Border.all(
            color: isDark ? AppColors.borderDark : AppColors.borderLight),
      ),
      child: Column(
        children: items.asMap().entries.map((entry) {
          final isLast = entry.key == items.length - 1;
          return Padding(
            padding: EdgeInsets.only(bottom: isLast ? 0 : 10),
            child: Row(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Padding(
                  padding: const EdgeInsets.only(top: 6),
                  child: Container(
                    width: 4,
                    height: 4,
                    decoration: BoxDecoration(
                      color: context.subtitleColor,
                      shape: BoxShape.circle,
                    ),
                  ),
                ),
                const SizedBox(width: 10),
                Expanded(
                  child: Text(
                    entry.value,
                    style: TextStyle(
                      fontSize: 13,
                      color: context.subtitleColor,
                      height: 1.3,
                    ),
                  ),
                ),
              ],
            ),
          );
        }).toList(),
      ),
    );
  }

  // ================================================================
  // 通用组件
  // ================================================================

  Widget _buildSectionTitle(String title) {
    return Padding(
      padding: const EdgeInsets.only(left: 2),
      child: Text(
        title,
        style: TextStyle(
          fontSize: 13,
          fontWeight: FontWeight.w600,
          color: context.subtitleColor,
        ),
      ),
    );
  }

  Widget _buildNotice(String text, NoticeType type) {
    final Color accentColor;
    switch (type) {
      case NoticeType.error:
        accentColor = AppColors.error;
      case NoticeType.warning:
        accentColor = AppColors.warning;
      case NoticeType.success:
        accentColor = AppColors.success;
      case NoticeType.info:
        accentColor = AppColors.primary;
    }

    final isDark = context.isDark;
    return Padding(
      padding: const EdgeInsets.only(bottom: 12),
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
        decoration: BoxDecoration(
          color: isDark ? AppColors.darkCard : Colors.white,
          borderRadius: BorderRadius.circular(AppRadius.card),
          border: Border.all(
              color: isDark ? AppColors.borderDark : AppColors.borderLight),
        ),
        child: Row(
          children: [
            Container(
              width: 3,
              height: 28,
              decoration: BoxDecoration(
                color: accentColor,
                borderRadius: BorderRadius.circular(2),
              ),
            ),
            const SizedBox(width: 12),
            Expanded(
              child: Text(
                text,
                style: TextStyle(
                  fontSize: 13,
                  fontWeight: FontWeight.w500,
                  color: context.titleColor,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  Color _categoryColor(String category) {
    switch (category) {
      case 'firmware':
        return AppColors.warning;
      case 'model':
        return AppColors.secondary;
      case 'map':
        return AppColors.success;
      case 'config':
        return AppColors.info;
      default:
        return AppColors.primary;
    }
  }

  // ================================================================
  // 业务逻辑
  // ================================================================

  /// 从本地选择文件并上传到机器人
  Future<void> _selectAndUploadFirmware(BuildContext context) async {
    HapticFeedback.mediumImpact();

    final result = await FilePicker.platform.pickFiles(
      type: FileType.any,
      withData: true,
    );

    if (result == null || result.files.isEmpty) return;

    final file = result.files.first;
    if (file.bytes == null && file.path == null) {
      _showError('无法读取文件');
      return;
    }

    final bytes = file.bytes ?? await File(file.path!).readAsBytes();
    final filename = file.name;

    setState(() {
      _isUploading = true;
      _uploadProgress = 0.0;
      _statusMessage = null;
      _selectedFileName = filename;
      _lastUploadedRemotePath = null;
    });

    try {
      final client = context.read<RobotConnectionProvider>().client;
      if (client == null) throw Exception('未连接');

      final remotePath = '/firmware/$filename';
      final response = await client.uploadFile(
        localBytes: bytes,
        remotePath: remotePath,
        filename: filename,
        category: 'firmware',
        overwrite: true,
        onProgress: (progress) {
          if (mounted) setState(() => _uploadProgress = progress);
        },
      );

      setState(() {
        _isUploading = false;
        if (response.success) {
          _statusMessage = '固件上传成功，可点击下方按钮应用到机器人';
          _lastUploadedRemotePath = remotePath;
        } else {
          _statusMessage = '上传失败: ${response.message}';
        }
      });
    } catch (e) {
      setState(() {
        _isUploading = false;
        _statusMessage = '升级失败: $e';
      });
    }
  }

  /// 触发机器人端固件应用（刷写脚本）
  Future<void> _applyFirmware() async {
    if (_lastUploadedRemotePath == null) return;
    HapticFeedback.mediumImpact();

    setState(() => _isApplying = true);

    try {
      final client = context.read<RobotConnectionProvider>().client;
      if (client == null) throw Exception('未连接');

      final response =
          await client.applyFirmware(firmwarePath: _lastUploadedRemotePath!);

      setState(() {
        _isApplying = false;
        _statusMessage = response.success
            ? '固件应用指令已发送，机器人可能即将重启'
            : '应用失败: ${response.message}';
        if (response.success) _lastUploadedRemotePath = null;
      });
    } catch (e) {
      setState(() {
        _isApplying = false;
        _statusMessage = '应用固件失败: $e';
      });
    }
  }

  /// 从 GitHub 检查最新版本
  Future<void> _checkCloudUpdate() async {
    setState(() {
      _isCheckingCloud = true;
      _cloudError = null;
      _latestRelease = null;
    });

    try {
      final release = await _cloudService.fetchLatestRelease();
      setState(() {
        _isCheckingCloud = false;
        _latestRelease = release;
        if (release == null) {
          _cloudError = '未找到任何 Release';
        }
      });
    } catch (e) {
      setState(() {
        _isCheckingCloud = false;
        _cloudError = '检查更新失败: $e';
      });
    }
  }

  /// 从云端下载资产并自动上传到机器人
  Future<void> _downloadAndUploadAsset(CloudAsset asset) async {
    HapticFeedback.mediumImpact();

    setState(() {
      _isDownloadingAsset = true;
      _downloadProgress = 0.0;
      _downloadingAssetName = asset.name;
      _statusMessage = null;
      _lastUploadedRemotePath = null;
    });

    try {
      // 1. 下载到内存
      final bytes = await _cloudService.downloadAsset(
        asset,
        onProgress: (p) {
          if (mounted) setState(() => _downloadProgress = p * 0.5); // 0~50%
        },
      );

      // 2. 上传到机器人
      final client = context.read<RobotConnectionProvider>().client;
      if (client == null) throw Exception('未连接');

      final remotePath = '/${asset.category}/${asset.name}';
      final response = await client.uploadFile(
        localBytes: bytes,
        remotePath: remotePath,
        filename: asset.name,
        category: asset.category,
        overwrite: true,
        onProgress: (p) {
          if (mounted) {
            setState(() => _downloadProgress = 0.5 + p * 0.5); // 50~100%
          }
        },
      );

      setState(() {
        _isDownloadingAsset = false;
        _downloadingAssetName = null;
        if (response.success) {
          _statusMessage = '${asset.name} 已成功部署到机器人';
          if (asset.category == 'firmware') {
            _lastUploadedRemotePath = remotePath;
          }
        } else {
          _statusMessage = '部署失败: ${response.message}';
        }
      });
    } catch (e) {
      setState(() {
        _isDownloadingAsset = false;
        _downloadingAssetName = null;
        _statusMessage = '部署失败: $e';
      });
    }
  }

  void _showError(String msg) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(msg),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
      ),
    );
  }
}

// ================================================================
// 辅助枚举和组件
// ================================================================

enum NoticeType { error, warning, success, info }

class _StatusDot extends StatelessWidget {
  final String label;
  final Color color;

  const _StatusDot({required this.label, required this.color});

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        Container(
          width: 6,
          height: 6,
          decoration: BoxDecoration(
            color: color,
            shape: BoxShape.circle,
          ),
        ),
        const SizedBox(width: 5),
        Text(
          label,
          style: TextStyle(
            fontSize: 12,
            fontWeight: FontWeight.w500,
            color: context.subtitleColor,
          ),
        ),
      ],
    );
  }
}
