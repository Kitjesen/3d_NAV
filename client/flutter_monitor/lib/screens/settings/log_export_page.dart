import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:intl/intl.dart';
import 'package:provider/provider.dart';
import 'package:path_provider/path_provider.dart';
import '../../theme/app_theme.dart';
import '../../services/robot_connection_provider.dart';

class LogExportPage extends StatefulWidget {
  const LogExportPage({super.key});

  @override
  State<LogExportPage> createState() => _LogExportPageState();
}

class _LogExportPageState extends State<LogExportPage> {
  DateTime _startDate = DateTime.now().subtract(const Duration(days: 1));
  DateTime _endDate = DateTime.now();
  bool _isExporting = false;
  double _exportProgress = 0.0;
  String? _exportedFilePath;
  String? _errorMessage;

  final _dateFormat = DateFormat('yyyy-MM-dd HH:mm');

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final provider = context.watch<RobotConnectionProvider>();
    final isConnected = provider.isConnected;

    return Scaffold(
      appBar: AppBar(
        title: const Text('导出日志'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: ListView(
        physics: const BouncingScrollPhysics(),
        padding: const EdgeInsets.all(20),
        children: [
          // ===== 时间范围选择 =====
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
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  '时间范围',
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.w700,
                    color: isDark ? Colors.white : Colors.black87,
                  ),
                ),
                const SizedBox(height: 16),
                _buildDateRow(
                  label: '开始时间',
                  date: _startDate,
                  onTap: () => _pickDate(isStart: true),
                ),
                Divider(height: 24, color: context.dividerColor),
                _buildDateRow(
                  label: '结束时间',
                  date: _endDate,
                  onTap: () => _pickDate(isStart: false),
                ),
              ],
            ),
          ),
          const SizedBox(height: 16),

          // ===== 快捷选择 =====
          Row(
            children: [
              _buildQuickChip('最近1小时', const Duration(hours: 1)),
              const SizedBox(width: 8),
              _buildQuickChip('最近24小时', const Duration(days: 1)),
              const SizedBox(width: 8),
              _buildQuickChip('最近7天', const Duration(days: 7)),
            ],
          ),
          const SizedBox(height: 24),

          // ===== 导出内容说明 =====
          Container(
            padding: const EdgeInsets.all(16),
            decoration: BoxDecoration(
              color: AppColors.info.withOpacity(isDark ? 0.08 : 0.05),
              borderRadius: BorderRadius.circular(16),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  '导出内容包含',
                  style: TextStyle(
                    fontSize: 13,
                    fontWeight: FontWeight.w600,
                    color: isDark ? Colors.white70 : Colors.black87,
                  ),
                ),
                const SizedBox(height: 8),
                _buildContentItem('控制指令日志 (Teleop / Mode)'),
                _buildContentItem('状态数据 (Battery / CPU / Pose)'),
                _buildContentItem('事件日志 (Events & Alerts)'),
                _buildContentItem('通信日志 (Connection / Reconnect)'),
              ],
            ),
          ),
          const SizedBox(height: 24),

          // ===== 进度 =====
          if (_isExporting) ...[
            Container(
              padding: const EdgeInsets.all(20),
              decoration: BoxDecoration(
                color: isDark ? AppColors.darkCard : Colors.white,
                borderRadius: BorderRadius.circular(22),
                boxShadow: [
                  BoxShadow(
                    color: context.cardShadowColor,
                    blurRadius: 14,
                    offset: const Offset(0, 5),
                  ),
                ],
              ),
              child: Column(
                children: [
                  const Text('正在导出...', style: TextStyle(fontWeight: FontWeight.w600)),
                  const SizedBox(height: 12),
                  ClipRRect(
                    borderRadius: BorderRadius.circular(8),
                    child: LinearProgressIndicator(
                      value: _exportProgress,
                      minHeight: 6,
                    ),
                  ),
                  const SizedBox(height: 8),
                  Text(
                    '${(_exportProgress * 100).toStringAsFixed(0)}%',
                    style: const TextStyle(
                        fontWeight: FontWeight.w600, color: AppColors.primary),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 24),
          ],

          // ===== 导出成功 =====
          if (_exportedFilePath != null) ...[
            Container(
              padding: const EdgeInsets.all(16),
              decoration: BoxDecoration(
                color: AppColors.success.withOpacity(0.1),
                borderRadius: BorderRadius.circular(16),
              ),
              child: Row(
                children: [
                  const Icon(Icons.check_circle,
                      color: AppColors.success, size: 20),
                  const SizedBox(width: 10),
                  Expanded(
                    child: Text(
                      '日志已导出到:\n$_exportedFilePath',
                      style: TextStyle(
                        fontSize: 12,
                        color: isDark ? Colors.white70 : Colors.black87,
                      ),
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(height: 16),
          ],

          if (_errorMessage != null)
            Container(
              padding: const EdgeInsets.all(16),
              margin: const EdgeInsets.only(bottom: 16),
              decoration: BoxDecoration(
                color: AppColors.error.withOpacity(0.1),
                borderRadius: BorderRadius.circular(16),
              ),
              child: Text(
                _errorMessage!,
                style: const TextStyle(fontSize: 13, color: AppColors.error),
              ),
            ),

          // ===== 导出按钮 =====
          SizedBox(
            height: 52,
            child: ElevatedButton.icon(
              onPressed: isConnected && !_isExporting
                  ? () => _exportLogs(context)
                  : null,
              icon: const Icon(Icons.download_rounded),
              label: const Text('开始导出'),
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

          if (!isConnected)
            Padding(
              padding: const EdgeInsets.only(top: 12),
              child: Text(
                '请先连接机器人',
                textAlign: TextAlign.center,
                style: TextStyle(fontSize: 13, color: context.subtitleColor),
              ),
            ),
        ],
      ),
    );
  }

  Widget _buildDateRow({
    required String label,
    required DateTime date,
    required VoidCallback onTap,
  }) {
    return GestureDetector(
      onTap: onTap,
      child: Row(
        children: [
          Text(label, style: TextStyle(fontSize: 14, color: context.subtitleColor)),
          const Spacer(),
          Text(
            _dateFormat.format(date),
            style: TextStyle(
              fontSize: 14,
              fontWeight: FontWeight.w600,
              color: context.isDark ? Colors.white : Colors.black87,
            ),
          ),
          const SizedBox(width: 6),
          Icon(Icons.chevron_right, size: 18, color: context.subtitleColor),
        ],
      ),
    );
  }

  Widget _buildQuickChip(String label, Duration duration) {
    final isDark = context.isDark;
    return Expanded(
      child: GestureDetector(
        onTap: () {
          HapticFeedback.selectionClick();
          setState(() {
            _endDate = DateTime.now();
            _startDate = _endDate.subtract(duration);
          });
        },
        child: Container(
          padding: const EdgeInsets.symmetric(vertical: 10),
          decoration: BoxDecoration(
            color: isDark ? AppColors.darkCard : Colors.white,
            borderRadius: BorderRadius.circular(12),
            boxShadow: [
              BoxShadow(
                color: context.cardShadowColor,
                blurRadius: 8,
                offset: const Offset(0, 3),
              ),
            ],
          ),
          child: Text(
            label,
            textAlign: TextAlign.center,
            style: TextStyle(
              fontSize: 12,
              fontWeight: FontWeight.w600,
              color: isDark ? Colors.white70 : Colors.black87,
            ),
          ),
        ),
      ),
    );
  }

  Widget _buildContentItem(String text) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 3),
      child: Row(
        children: [
          const Icon(Icons.check, size: 14, color: AppColors.info),
          const SizedBox(width: 8),
          Text(
            text,
            style: TextStyle(fontSize: 12, color: context.subtitleColor),
          ),
        ],
      ),
    );
  }

  Future<void> _pickDate({required bool isStart}) async {
    final initial = isStart ? _startDate : _endDate;
    final date = await showDatePicker(
      context: context,
      initialDate: initial,
      firstDate: DateTime.now().subtract(const Duration(days: 365)),
      lastDate: DateTime.now(),
    );
    if (date == null || !mounted) return;

    final time = await showTimePicker(
      context: context,
      initialTime: TimeOfDay.fromDateTime(initial),
    );
    if (time == null || !mounted) return;

    final combined = DateTime(date.year, date.month, date.day, time.hour, time.minute);
    setState(() {
      if (isStart) {
        _startDate = combined;
      } else {
        _endDate = combined;
      }
    });
  }

  Future<void> _exportLogs(BuildContext context) async {
    HapticFeedback.mediumImpact();

    setState(() {
      _isExporting = true;
      _exportProgress = 0.0;
      _exportedFilePath = null;
      _errorMessage = null;
    });

    try {
      // Simulate collecting logs from provider's cached data
      final provider = context.read<RobotConnectionProvider>();

      // Step 1: Collect events
      setState(() => _exportProgress = 0.2);
      await Future.delayed(const Duration(milliseconds: 500));

      // Step 2: Build log content
      setState(() => _exportProgress = 0.5);
      final buffer = StringBuffer();
      buffer.writeln('# Robot Log Export');
      buffer.writeln('# Range: ${_dateFormat.format(_startDate)} ~ ${_dateFormat.format(_endDate)}');
      buffer.writeln('# Generated: ${_dateFormat.format(DateTime.now())}');
      buffer.writeln('');

      // Connection info
      buffer.writeln('## Connection');
      buffer.writeln('Status: ${provider.status}');
      buffer.writeln('');

      // Latest state snapshot
      final fast = provider.latestFastState;
      final slow = provider.latestSlowState;
      if (fast != null) {
        buffer.writeln('## Latest Fast State');
        buffer.writeln('Pose: (${fast.pose.position.x.toStringAsFixed(3)}, ${fast.pose.position.y.toStringAsFixed(3)})');
        buffer.writeln('Linear: ${fast.velocity.linear.x.toStringAsFixed(3)} m/s');
        buffer.writeln('Angular: ${fast.velocity.angular.z.toStringAsFixed(3)} rad/s');
        buffer.writeln('');
      }
      if (slow != null) {
        buffer.writeln('## Latest Slow State');
        buffer.writeln('Battery: ${slow.resources.batteryPercent.toStringAsFixed(1)}%');
        buffer.writeln('CPU: ${slow.resources.cpuPercent.toStringAsFixed(1)}%');
        buffer.writeln('Temp: ${slow.resources.cpuTemp.toStringAsFixed(1)}°C');
        buffer.writeln('Mode: ${slow.currentMode}');
        buffer.writeln('');
      }

      setState(() => _exportProgress = 0.8);
      await Future.delayed(const Duration(milliseconds: 300));

      // Step 3: Save file
      final dir = await getApplicationDocumentsDirectory();
      final timestamp = DateFormat('yyyyMMdd_HHmmss').format(DateTime.now());
      final filePath = '${dir.path}/robot_log_$timestamp.txt';
      final file = File(filePath);
      await file.writeAsString(buffer.toString());

      setState(() {
        _isExporting = false;
        _exportProgress = 1.0;
        _exportedFilePath = filePath;
      });
    } catch (e) {
      setState(() {
        _isExporting = false;
        _errorMessage = '导出失败: $e';
      });
    }
  }
}
