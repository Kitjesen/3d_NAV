import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_monitor/app/theme.dart';

class VersionDetailPage extends StatelessWidget {
  const VersionDetailPage({super.key});

  static const _appVersion = 'v1.0.0';
  static const _buildNumber = '1';
  static const _protoVersion = 'gRPC v1.0';
  static const _framework = 'Flutter 3.x';

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    return Scaffold(
      appBar: AppBar(
        title: const Text('版本信息'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: ListView(
        physics: const BouncingScrollPhysics(),
        padding: const EdgeInsets.all(20),
        children: [
          // ===== Logo & Version =====
          Container(
            padding: const EdgeInsets.all(28),
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
                  width: 80,
                  height: 80,
                  decoration: BoxDecoration(
                    gradient: const LinearGradient(
                      begin: Alignment.topLeft,
                      end: Alignment.bottomRight,
                      colors: [AppColors.primary, AppColors.secondary],
                    ),
                    borderRadius: BorderRadius.circular(22),
                    boxShadow: [
                      BoxShadow(
                        color: AppColors.primary.withOpacity(0.3),
                        blurRadius: 20,
                        offset: const Offset(0, 8),
                      ),
                    ],
                  ),
                  child: const Icon(Icons.smart_toy_outlined,
                      size: 40, color: Colors.white),
                ),
                const SizedBox(height: 20),
                Text(
                  '大算机器人',
                  style: TextStyle(
                    fontSize: 22,
                    fontWeight: FontWeight.w800,
                    color: isDark ? Colors.white : Colors.black87,
                  ),
                ),
                const SizedBox(height: 4),
                Text(
                  'Robot Monitor & Control',
                  style: TextStyle(
                    fontSize: 13,
                    color: context.subtitleColor,
                    letterSpacing: 0.5,
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(height: 20),

          // ===== Detail Rows =====
          Container(
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
                _buildRow(context, 'App 版本', _appVersion),
                Divider(height: 0.5, indent: 16, color: context.dividerColor),
                _buildRow(context, 'Build 号', _buildNumber),
                Divider(height: 0.5, indent: 16, color: context.dividerColor),
                _buildRow(context, '协议版本', _protoVersion),
                Divider(height: 0.5, indent: 16, color: context.dividerColor),
                _buildRow(context, '框架', _framework),
                Divider(height: 0.5, indent: 16, color: context.dividerColor),
                _buildRow(context, '平台', 'ROS 2 + gRPC + WebRTC'),
              ],
            ),
          ),
          const SizedBox(height: 24),

          // ===== Copy All Button =====
          SizedBox(
            height: 48,
            child: ElevatedButton.icon(
              onPressed: () => _copyAll(context),
              icon: const Icon(Icons.content_copy, size: 18),
              label: const Text('一键复制版本信息'),
              style: ElevatedButton.styleFrom(
                backgroundColor: AppColors.primary,
                foregroundColor: Colors.white,
                shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(14)),
                elevation: 0,
              ),
            ),
          ),

          // ===== Footer =====
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 24),
            child: Text(
              '© 2025 大算科技\ngRPC · WebRTC · ROS 2 · BLE',
              textAlign: TextAlign.center,
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

  Widget _buildRow(BuildContext context, String label, String value) {
    final isDark = context.isDark;
    return Padding(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
      child: Row(
        children: [
          Text(
            label,
            style: TextStyle(fontSize: 14, color: context.subtitleColor),
          ),
          const Spacer(),
          Text(
            value,
            style: TextStyle(
              fontSize: 14,
              fontWeight: FontWeight.w600,
              color: isDark ? Colors.white : Colors.black87,
            ),
          ),
        ],
      ),
    );
  }

  void _copyAll(BuildContext context) {
    final info = 'App: $_appVersion (Build $_buildNumber)\n'
        'Proto: $_protoVersion\n'
        'Framework: $_framework\n'
        'Platform: ROS 2 + gRPC + WebRTC';

    Clipboard.setData(ClipboardData(text: info));
    HapticFeedback.lightImpact();
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: const Text('版本信息已复制到剪贴板'),
        behavior: SnackBarBehavior.floating,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
        duration: const Duration(seconds: 1),
      ),
    );
  }
}
