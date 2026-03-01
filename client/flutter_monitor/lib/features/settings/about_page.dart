import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/features/connection/splash_screen.dart' show kAppVersion;
import 'package:provider/provider.dart';
import 'package:flutter_monitor/core/locale/locale_provider.dart';

class AboutPage extends StatelessWidget {
  const AboutPage({super.key});

  @override
  Widget build(BuildContext context) {
    final locale = context.watch<LocaleProvider>();
    final isDark = context.isDark;

    return Scaffold(
      appBar: AppBar(
        title: Text(locale.tr('关于', 'About')),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
      ),
      body: ListView(
        padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
        children: [
          const SizedBox(height: 24),
          // Brand logo + name
          Center(
            child: Container(
              width: 72,
              height: 72,
              decoration: BoxDecoration(
                gradient: AppColors.brandGradient,
                borderRadius: BorderRadius.circular(20),
              ),
              child: const Center(
                child: Text(
                  'DS',
                  style: TextStyle(
                    fontSize: 28,
                    fontWeight: FontWeight.w900,
                    color: Colors.white,
                    letterSpacing: 1,
                    height: 1,
                  ),
                ),
              ),
            ),
          ),
          const SizedBox(height: 16),
          Center(
            child: Text(
              locale.tr('灵途 MapPilot', 'MapPilot'),
              style: TextStyle(
                fontSize: 22,
                fontWeight: FontWeight.w700,
                color: context.titleColor,
              ),
            ),
          ),
          const SizedBox(height: 4),
          Center(
            child: Text(
              locale.tr('自主导航系统', 'Autonomous Navigation System'),
              style: TextStyle(
                fontSize: 14,
                color: context.subtitleColor,
              ),
            ),
          ),
          const SizedBox(height: 32),

          // Info cards
          Container(
            decoration: context.softCardDecoration,
            padding: const EdgeInsets.all(16),
            child: Column(
              children: [
                _InfoRow(
                  label: locale.tr('版本', 'Version'),
                  value: 'v$kAppVersion',
                ),
                _divider(context),
                _InfoRow(
                  label: locale.tr('构建号', 'Build Number'),
                  value: const String.fromEnvironment('BUILD_NUMBER').isNotEmpty
                      ? const String.fromEnvironment('BUILD_NUMBER')
                      : locale.tr('开发版', 'Dev'),
                ),
                _divider(context),
                _InfoRow(
                  label: locale.tr('gRPC 端口', 'gRPC Port'),
                  value: '50051',
                ),
                _divider(context),
                _InfoRow(
                  label: locale.tr('平台', 'Platform'),
                  value: Theme.of(context).platform.name,
                ),
              ],
            ),
          ),
          const SizedBox(height: 16),

          // GitHub link
          Container(
            decoration: context.softCardDecoration,
            child: Material(
              color: Colors.transparent,
              borderRadius: BorderRadius.circular(AppRadius.card),
              child: InkWell(
                borderRadius: BorderRadius.circular(AppRadius.card),
                onTap: () {
                  HapticFeedback.selectionClick();
                  Clipboard.setData(const ClipboardData(
                    text: 'https://github.com/Kitjesen/MapPilot',
                  ));
                  ScaffoldMessenger.of(context).showSnackBar(
                    SnackBar(
                      content: Text(locale.tr(
                        'GitHub 地址已复制到剪贴板',
                        'GitHub URL copied to clipboard',
                      )),
                      behavior: SnackBarBehavior.floating,
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(10),
                      ),
                      duration: const Duration(seconds: 2),
                    ),
                  );
                },
                child: Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
                  child: Row(
                    children: [
                      Icon(Icons.code_outlined,
                          size: 18, color: context.subtitleColor),
                      const SizedBox(width: 12),
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text(
                              'GitHub',
                              style: TextStyle(
                                fontSize: 14,
                                fontWeight: FontWeight.w500,
                                color: context.titleColor,
                              ),
                            ),
                            const SizedBox(height: 2),
                            Text(
                              'github.com/Kitjesen/MapPilot',
                              style: TextStyle(
                                fontSize: 12,
                                color: context.subtitleColor,
                              ),
                            ),
                          ],
                        ),
                      ),
                      Icon(Icons.copy_outlined,
                          size: 16, color: context.subtitleColor),
                    ],
                  ),
                ),
              ),
            ),
          ),
          const SizedBox(height: 16),

          // Open-source licenses
          Container(
            decoration: context.softCardDecoration,
            child: Material(
              color: Colors.transparent,
              borderRadius: BorderRadius.circular(AppRadius.card),
              child: InkWell(
                borderRadius: BorderRadius.circular(AppRadius.card),
                onTap: () {
                  showLicensePage(
                    context: context,
                    applicationName: locale.tr('灵途 MapPilot', 'MapPilot'),
                    applicationVersion: 'v$kAppVersion',
                  );
                },
                child: Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
                  child: Row(
                    children: [
                      Icon(Icons.article_outlined,
                          size: 18, color: context.subtitleColor),
                      const SizedBox(width: 12),
                      Expanded(
                        child: Text(
                          locale.tr('开源许可证', 'Open Source Licenses'),
                          style: TextStyle(
                            fontSize: 14,
                            fontWeight: FontWeight.w500,
                            color: context.titleColor,
                          ),
                        ),
                      ),
                      Icon(Icons.chevron_right,
                          size: 18, color: context.subtitleColor),
                    ],
                  ),
                ),
              ),
            ),
          ),
          const SizedBox(height: 40),
          Center(
            child: Text(
              locale.tr('大算机器人', 'Dasuan Robotics'),
              style: TextStyle(
                fontSize: 12,
                color: context.subtitleColor,
              ),
            ),
          ),
          const SizedBox(height: 8),
        ],
      ),
    );
  }

  Widget _divider(BuildContext context) => Divider(
        height: 16,
        thickness: 0.5,
        color: context.dividerColor,
      );
}

class _InfoRow extends StatelessWidget {
  final String label;
  final String value;

  const _InfoRow({required this.label, required this.value});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(
            label,
            style: TextStyle(
              fontSize: 14,
              color: context.subtitleColor,
            ),
          ),
          Text(
            value,
            style: TextStyle(
              fontSize: 14,
              fontWeight: FontWeight.w500,
              color: context.titleColor,
            ),
          ),
        ],
      ),
    );
  }
}
