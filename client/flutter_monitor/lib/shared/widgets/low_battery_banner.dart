import 'dart:async';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:robot_proto/robot_proto.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';

/// Persistent low-battery banner displayed at the top of MainShellScreen.
///
/// - battery < 5%:  red banner  "battery critically low — stop tasks immediately"
/// - battery < 15%: orange banner "battery low — return to base soon"
/// - battery >= 15%: hidden
class LowBatteryBanner extends StatefulWidget {
  const LowBatteryBanner({super.key});

  @override
  State<LowBatteryBanner> createState() => _LowBatteryBannerState();
}

class _LowBatteryBannerState extends State<LowBatteryBanner> {
  double? _batteryPercent;
  StreamSubscription<SlowState>? _sub;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      final provider = context.read<RobotConnectionProvider>();
      _batteryPercent = provider.latestSlowState?.resources.batteryPercent;
      _sub = provider.slowStateStream.listen((ss) {
        final bat = ss.resources.batteryPercent;
        if (bat != _batteryPercent && mounted) {
          setState(() => _batteryPercent = bat);
        }
      });
    });
  }

  @override
  void dispose() {
    _sub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final isConnected = context.select<RobotConnectionProvider, bool>(
      (p) => p.isConnected,
    );
    if (!isConnected) return const SizedBox.shrink();

    final battery = _batteryPercent;
    if (battery == null || battery >= 15) return const SizedBox.shrink();

    final isCritical = battery < 5;
    final color = isCritical ? AppColors.error : AppColors.warning;
    final icon = isCritical ? Icons.battery_alert : Icons.battery_2_bar;
    final text = isCritical
        ? '\u{1F534} 电量极低 (${battery.toStringAsFixed(0)}%) \u2014 立即停止任务'
        : '\u26A0 电量低 (${battery.toStringAsFixed(0)}%) \u2014 建议尽快返航';
    final isDark = context.isDark;

    return TweenAnimationBuilder<double>(
      tween: Tween(begin: -1.0, end: 0.0),
      duration: const Duration(milliseconds: 350),
      curve: Curves.easeOutCubic,
      builder: (context, value, child) => Transform.translate(
        offset: Offset(0, value * 48),
        child: child,
      ),
      child: Container(
        width: double.infinity,
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
        decoration: BoxDecoration(
          color: isDark
              ? color.withValues(alpha: 0.2)
              : color.withValues(alpha: 0.1),
          border: Border(
            bottom: BorderSide(
              color: color.withValues(alpha: 0.3),
              width: 1,
            ),
          ),
        ),
        child: Row(
          children: [
            Icon(icon, size: 16, color: color),
            const SizedBox(width: 8),
            Expanded(
              child: Text(
                text,
                style: TextStyle(
                  fontSize: 12,
                  fontWeight: FontWeight.w600,
                  color: color,
                ),
                maxLines: 1,
                overflow: TextOverflow.ellipsis,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
