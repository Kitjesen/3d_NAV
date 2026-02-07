import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';

class SupportPage extends StatelessWidget {
  const SupportPage({super.key});

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    return Scaffold(
      appBar: AppBar(
        title: const Text('反馈与支持'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: ListView(
        physics: const BouncingScrollPhysics(),
        padding: const EdgeInsets.all(20),
        children: [
          // ===== 联系方式 =====
          _SupportCard(
            icon: Icons.email_outlined,
            iconColor: AppColors.primary,
            title: '技术支持邮箱',
            subtitle: 'support@dasuan-robot.com',
            action: '复制邮箱',
            onAction: () => _copyToClipboard(context, 'support@dasuan-robot.com', '邮箱已复制'),
          ),
          const SizedBox(height: 12),
          _SupportCard(
            icon: Icons.language,
            iconColor: AppColors.info,
            title: '文档中心',
            subtitle: 'docs.dasuan-robot.com',
            action: '复制链接',
            onAction: () => _copyToClipboard(context, 'https://docs.dasuan-robot.com', '链接已复制'),
          ),
          const SizedBox(height: 12),
          _SupportCard(
            icon: Icons.forum_outlined,
            iconColor: AppColors.secondary,
            title: '社区论坛',
            subtitle: '与其他用户交流',
            action: '复制链接',
            onAction: () => _copyToClipboard(context, 'https://community.dasuan-robot.com', '链接已复制'),
          ),
          const SizedBox(height: 12),
          _SupportCard(
            icon: Icons.question_answer_outlined,
            iconColor: AppColors.warning,
            title: '常见问题 FAQ',
            subtitle: '查看常见问题及解答',
            action: '复制链接',
            onAction: () => _copyToClipboard(context, 'https://docs.dasuan-robot.com/faq', '链接已复制'),
          ),
          const SizedBox(height: 24),

          // ===== 一键提交设备信息 =====
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
                  '一键复制设备信息',
                  style: TextStyle(
                    fontSize: 16,
                    fontWeight: FontWeight.w700,
                    color: isDark ? Colors.white : Colors.black87,
                  ),
                ),
                const SizedBox(height: 8),
                Text(
                  '反馈问题时附带设备信息，可以帮助我们更快定位问题。',
                  style: TextStyle(
                    fontSize: 13,
                    color: context.subtitleColor,
                    height: 1.4,
                  ),
                ),
                const SizedBox(height: 16),
                SizedBox(
                  width: double.infinity,
                  height: 48,
                  child: ElevatedButton.icon(
                    onPressed: () => _copyDeviceInfo(context),
                    icon: const Icon(Icons.content_copy, size: 18),
                    label: const Text('复制设备信息'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: AppColors.primary,
                      foregroundColor: Colors.white,
                      shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(14)),
                      elevation: 0,
                    ),
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  void _copyToClipboard(BuildContext context, String text, String message) {
    Clipboard.setData(ClipboardData(text: text));
    HapticFeedback.lightImpact();
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
        duration: const Duration(seconds: 1),
      ),
    );
  }

  void _copyDeviceInfo(BuildContext context) {
    final provider = context.read<RobotConnectionProvider>();
    final slow = provider.latestSlowState;

    final info = StringBuffer();
    info.writeln('=== 大算机器人 设备信息 ===');
    info.writeln('APP 版本: v1.0.0 (Build 1)');
    info.writeln('协议: gRPC + WebRTC');
    info.writeln('连接状态: ${provider.isConnected ? "已连接" : "未连接"}');
    if (slow != null) {
      info.writeln('电池: ${slow.resources.batteryPercent.toStringAsFixed(0)}%');
      info.writeln('CPU: ${slow.resources.cpuPercent.toStringAsFixed(0)}%');
      info.writeln('温度: ${slow.resources.cpuTemp.toStringAsFixed(1)}°C');
      info.writeln('模式: ${slow.currentMode}');
    }
    info.writeln('时间: ${DateTime.now().toIso8601String()}');

    _copyToClipboard(context, info.toString(), '设备信息已复制到剪贴板');
  }
}

class _SupportCard extends StatelessWidget {
  final IconData icon;
  final Color iconColor;
  final String title;
  final String subtitle;
  final String action;
  final VoidCallback onAction;

  const _SupportCard({
    required this.icon,
    required this.iconColor,
    required this.title,
    required this.subtitle,
    required this.action,
    required this.onAction,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(20),
        boxShadow: [
          BoxShadow(
            color: context.cardShadowColor,
            blurRadius: 12,
            offset: const Offset(0, 4),
          ),
        ],
      ),
      child: Row(
        children: [
          Container(
            width: 42,
            height: 42,
            decoration: BoxDecoration(
              color: iconColor.withOpacity(isDark ? 0.15 : 0.08),
              borderRadius: BorderRadius.circular(12),
            ),
            child: Icon(icon, color: iconColor, size: 22),
          ),
          const SizedBox(width: 14),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  title,
                  style: TextStyle(
                    fontSize: 15,
                    fontWeight: FontWeight.w600,
                    color: isDark ? Colors.white : Colors.black87,
                  ),
                ),
                const SizedBox(height: 2),
                Text(
                  subtitle,
                  style: TextStyle(fontSize: 12, color: context.subtitleColor),
                ),
              ],
            ),
          ),
          TextButton(
            onPressed: onAction,
            style: TextButton.styleFrom(
              foregroundColor: AppColors.primary,
              visualDensity: VisualDensity.compact,
            ),
            child: Text(action, style: const TextStyle(fontSize: 12)),
          ),
        ],
      ),
    );
  }
}
