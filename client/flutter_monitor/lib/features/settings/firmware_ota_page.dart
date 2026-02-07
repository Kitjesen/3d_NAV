import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:file_picker/file_picker.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';

class FirmwareOtaPage extends StatefulWidget {
  const FirmwareOtaPage({super.key});

  @override
  State<FirmwareOtaPage> createState() => _FirmwareOtaPageState();
}

class _FirmwareOtaPageState extends State<FirmwareOtaPage> {
  bool _isUploading = false;
  double _uploadProgress = 0.0;
  String? _statusMessage;
  String? _selectedFileName;

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final provider = context.watch<RobotConnectionProvider>();
    final isConnected = provider.isConnected;
    final slowState = provider.latestSlowState;

    // Try to get firmware version from slow state
    final currentVersion = slowState?.currentMode ?? '未知';
    final battery = slowState?.resources.batteryPercent ?? 0.0;

    return Scaffold(
      appBar: AppBar(
        title: const Text('固件升级'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: ListView(
        physics: const BouncingScrollPhysics(),
        padding: const EdgeInsets.all(20),
        children: [
          // ===== 当前固件信息卡片 =====
          Container(
            padding: const EdgeInsets.all(20),
            decoration: BoxDecoration(
              color: isDark ? AppColors.darkCard : Colors.white,
              borderRadius: BorderRadius.circular(22),
              boxShadow: [
                BoxShadow(
                  color: context.cardShadowColor,
                  blurRadius: 16,
                  offset: const Offset(0, 6),
                ),
              ],
            ),
            child: Column(
              children: [
                Container(
                  width: 64,
                  height: 64,
                  decoration: BoxDecoration(
                    color: AppColors.primary.withOpacity(isDark ? 0.15 : 0.08),
                    shape: BoxShape.circle,
                  ),
                  child: const Icon(Icons.memory,
                      size: 32, color: AppColors.primary),
                ),
                const SizedBox(height: 16),
                Text(
                  '当前固件',
                  style: TextStyle(
                    fontSize: 13,
                    color: context.subtitleColor,
                  ),
                ),
                const SizedBox(height: 4),
                Text(
                  currentVersion,
                  style: TextStyle(
                    fontSize: 20,
                    fontWeight: FontWeight.w700,
                    color: isDark ? Colors.white : Colors.black87,
                  ),
                ),
                const SizedBox(height: 16),
                // Status indicators
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    _buildStatusChip(
                      icon: Icons.power,
                      label: isConnected ? '已连接' : '未连接',
                      color: isConnected ? AppColors.success : AppColors.error,
                    ),
                    const SizedBox(width: 10),
                    _buildStatusChip(
                      icon: Icons.battery_charging_full,
                      label: '${battery.toStringAsFixed(0)}%',
                      color: battery > 30 ? AppColors.success : AppColors.error,
                    ),
                  ],
                ),
              ],
            ),
          ),
          const SizedBox(height: 24),

          // ===== 升级前检查 =====
          if (!isConnected)
            _buildWarningBanner('请先连接机器人', AppColors.warning),
          if (isConnected && battery < 30)
            _buildWarningBanner('电量低于 30%，建议充电后再升级', AppColors.error),
          if (_statusMessage != null)
            Padding(
              padding: const EdgeInsets.only(bottom: 16),
              child: _buildWarningBanner(
                  _statusMessage!,
                  _statusMessage!.contains('成功')
                      ? AppColors.success
                      : AppColors.primary),
            ),

          // ===== 升级进度 =====
          if (_isUploading) ...[
            Container(
              padding: const EdgeInsets.all(20),
              decoration: BoxDecoration(
                color: isDark ? AppColors.darkCard : Colors.white,
                borderRadius: BorderRadius.circular(22),
                boxShadow: [
                  BoxShadow(
                    color: context.cardShadowColor,
                    blurRadius: 16,
                    offset: const Offset(0, 6),
                  ),
                ],
              ),
              child: Column(
                children: [
                  Text(
                    '正在上传固件...',
                    style: TextStyle(
                      fontSize: 15,
                      fontWeight: FontWeight.w600,
                      color: isDark ? Colors.white : Colors.black87,
                    ),
                  ),
                  if (_selectedFileName != null) ...[
                    const SizedBox(height: 4),
                    Text(
                      _selectedFileName!,
                      style: TextStyle(
                        fontSize: 12,
                        color: context.subtitleColor,
                      ),
                    ),
                  ],
                  const SizedBox(height: 16),
                  ClipRRect(
                    borderRadius: BorderRadius.circular(8),
                    child: LinearProgressIndicator(
                      value: _uploadProgress,
                      minHeight: 8,
                      backgroundColor: isDark
                          ? Colors.white.withOpacity(0.06)
                          : Colors.black.withOpacity(0.05),
                    ),
                  ),
                  const SizedBox(height: 8),
                  Text(
                    '${(_uploadProgress * 100).toStringAsFixed(0)}%',
                    style: TextStyle(
                      fontSize: 14,
                      fontWeight: FontWeight.w600,
                      color: AppColors.primary,
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 24),
          ],

          // ===== 操作按钮 =====
          SizedBox(
            width: double.infinity,
            height: 52,
            child: ElevatedButton.icon(
              onPressed:
                  isConnected && !_isUploading && battery >= 30
                      ? () => _selectAndUploadFirmware(context)
                      : null,
              icon: const Icon(Icons.upload_file),
              label: const Text('选择固件文件并升级'),
              style: ElevatedButton.styleFrom(
                backgroundColor: AppColors.primary,
                foregroundColor: Colors.white,
                disabledBackgroundColor:
                    AppColors.primary.withOpacity(isDark ? 0.15 : 0.1),
                disabledForegroundColor: context.subtitleColor,
                shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(16)),
                elevation: 0,
              ),
            ),
          ),

          // ===== 说明 =====
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 20),
            child: Text(
              '升级须知：\n'
              '• 升级前确保机器人电量 ≥ 30%\n'
              '• 升级过程中请勿断开连接\n'
              '• 确保机器人处于静止状态\n'
              '• 支持 .bin / .fw / .zip 格式固件包\n'
              '• 升级完成后机器人将自动重启',
              style: TextStyle(
                fontSize: 12,
                color: context.subtitleColor,
                height: 1.6,
              ),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildStatusChip(
      {required IconData icon, required String label, required Color color}) {
    final isDark = context.isDark;
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      decoration: BoxDecoration(
        color: color.withOpacity(isDark ? 0.12 : 0.08),
        borderRadius: BorderRadius.circular(20),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 14, color: color),
          const SizedBox(width: 5),
          Text(
            label,
            style: TextStyle(
              fontSize: 12,
              fontWeight: FontWeight.w600,
              color: color,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildWarningBanner(String text, Color color) {
    return Container(
      margin: const EdgeInsets.only(bottom: 16),
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
      decoration: BoxDecoration(
        color: color.withOpacity(0.1),
        borderRadius: BorderRadius.circular(14),
      ),
      child: Row(
        children: [
          Icon(Icons.info_outline, size: 18, color: color),
          const SizedBox(width: 10),
          Expanded(
            child: Text(
              text,
              style: TextStyle(fontSize: 13, fontWeight: FontWeight.w600, color: color),
            ),
          ),
        ],
      ),
    );
  }

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
    });

    try {
      final client = context.read<RobotConnectionProvider>().client;
      if (client == null) throw Exception('未连接');

      final response = await client.uploadFile(
        localBytes: bytes,
        remotePath: '/firmware/$filename',
        filename: filename,
        category: 'firmware',
        overwrite: true,
        onProgress: (progress) {
          if (mounted) {
            setState(() => _uploadProgress = progress);
          }
        },
      );

      setState(() {
        _isUploading = false;
        _statusMessage = response.success
            ? '固件上传成功！机器人将自动应用升级。'
            : '上传失败: ${response.message}';
      });
    } catch (e) {
      setState(() {
        _isUploading = false;
        _statusMessage = '升级失败: $e';
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
