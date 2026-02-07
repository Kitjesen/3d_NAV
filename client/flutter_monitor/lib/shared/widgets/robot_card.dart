import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';

/// A hero card representing a saved or connected robot on the dashboard
class RobotCard extends StatelessWidget {
  final String name;
  final String address;
  final ConnectionStatus connectionStatus;
  final double? batteryPercent;
  final double? cpuPercent;
  final String connectionType; // 'wifi' or 'ble'
  final VoidCallback? onTap;
  final VoidCallback? onLongPress;
  final String? heroTag;

  const RobotCard({
    super.key,
    required this.name,
    required this.address,
    this.connectionStatus = ConnectionStatus.disconnected,
    this.batteryPercent,
    this.cpuPercent,
    this.connectionType = 'wifi',
    this.onTap,
    this.onLongPress,
    this.heroTag,
  });

  Color _statusColor() {
    switch (connectionStatus) {
      case ConnectionStatus.connected:
        return AppColors.success;
      case ConnectionStatus.connecting:
      case ConnectionStatus.reconnecting:
        return AppColors.warning;
      case ConnectionStatus.error:
        return AppColors.error;
      case ConnectionStatus.disconnected:
        return Colors.grey;
    }
  }

  String _statusText() {
    switch (connectionStatus) {
      case ConnectionStatus.connected:
        return '已连接';
      case ConnectionStatus.connecting:
        return '连接中...';
      case ConnectionStatus.reconnecting:
        return '重连中...';
      case ConnectionStatus.error:
        return '连接错误';
      case ConnectionStatus.disconnected:
        return '未连接';
    }
  }

  IconData _connectionIcon() {
    return connectionType == 'ble' ? Icons.bluetooth : Icons.wifi;
  }

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final isConnected = connectionStatus == ConnectionStatus.connected;
    final statusColor = _statusColor();

    Widget card = GestureDetector(
      onTap: () {
        HapticFeedback.lightImpact();
        onTap?.call();
      },
      onLongPress: onLongPress,
      child: Container(
        decoration: BoxDecoration(
          borderRadius: BorderRadius.circular(20),
          boxShadow: [
            BoxShadow(
              color: isConnected
                  ? AppColors.primary.withOpacity(isDark ? 0.2 : 0.12)
                  : context.cardShadowColor,
              blurRadius: 24,
              offset: const Offset(0, 8),
            ),
          ],
        ),
        child: ClipRRect(
          borderRadius: BorderRadius.circular(20),
          child: BackdropFilter(
            filter: ImageFilter.blur(sigmaX: 20, sigmaY: 20),
            child: Container(
              padding: const EdgeInsets.all(20),
              decoration: BoxDecoration(
                color: context.glassColor,
                borderRadius: BorderRadius.circular(20),
                border: Border.all(
                  color: isConnected
                      ? AppColors.primary.withOpacity(0.3)
                      : context.glassBorder,
                ),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  // Header row: icon + name + status
                  Row(
                    children: [
                      // Robot avatar
                      Container(
                        width: 48,
                        height: 48,
                        decoration: BoxDecoration(
                          gradient: LinearGradient(
                            begin: Alignment.topLeft,
                            end: Alignment.bottomRight,
                            colors: isConnected
                                ? [AppColors.primary, AppColors.secondary]
                                : [Colors.grey.shade400, Colors.grey.shade500],
                          ),
                          borderRadius: BorderRadius.circular(14),
                        ),
                        child: const Icon(
                          Icons.smart_toy_outlined,
                          color: Colors.white,
                          size: 24,
                        ),
                      ),
                      const SizedBox(width: 14),
                      // Name + address
                      Expanded(
                        child: Column(
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text(
                              name,
                              style: TextStyle(
                                fontSize: 17,
                                fontWeight: FontWeight.w700,
                                color: isDark ? Colors.white : Colors.black87,
                                letterSpacing: -0.3,
                              ),
                            ),
                            const SizedBox(height: 2),
                            Row(
                              children: [
                                Icon(
                                  _connectionIcon(),
                                  size: 13,
                                  color: context.subtitleColor,
                                ),
                                const SizedBox(width: 4),
                                Text(
                                  address,
                                  style: TextStyle(
                                    fontSize: 13,
                                    color: context.subtitleColor,
                                  ),
                                ),
                              ],
                            ),
                          ],
                        ),
                      ),
                      // Status badge
                      Container(
                        padding: const EdgeInsets.symmetric(
                            horizontal: 10, vertical: 5),
                        decoration: BoxDecoration(
                          color: statusColor.withOpacity(0.12),
                          borderRadius: BorderRadius.circular(20),
                        ),
                        child: Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Container(
                              width: 7,
                              height: 7,
                              decoration: BoxDecoration(
                                color: statusColor,
                                shape: BoxShape.circle,
                              ),
                            ),
                            const SizedBox(width: 5),
                            Text(
                              _statusText(),
                              style: TextStyle(
                                fontSize: 12,
                                fontWeight: FontWeight.w600,
                                color: statusColor,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ],
                  ),

                  // Quick metrics (only when connected)
                  if (isConnected &&
                      (batteryPercent != null || cpuPercent != null)) ...[
                    const SizedBox(height: 16),
                    Row(
                      children: [
                        if (batteryPercent != null)
                          _MetricChip(
                            icon: Icons.battery_charging_full,
                            label: '${batteryPercent!.toStringAsFixed(0)}%',
                            color: _batteryColor(batteryPercent!),
                          ),
                        if (batteryPercent != null && cpuPercent != null)
                          const SizedBox(width: 12),
                        if (cpuPercent != null)
                          _MetricChip(
                            icon: Icons.memory,
                            label: '${cpuPercent!.toStringAsFixed(0)}%',
                            color: _cpuColor(cpuPercent!),
                          ),
                        const Spacer(),
                        Icon(
                          Icons.chevron_right,
                          size: 20,
                          color: context.subtitleColor,
                        ),
                      ],
                    ),
                  ],
                ],
              ),
            ),
          ),
        ),
      ),
    );

    if (heroTag != null) {
      card = Hero(tag: heroTag!, child: card);
    }
    return card;
  }

  Color _batteryColor(double pct) {
    if (pct > 60) return AppColors.success;
    if (pct > 20) return AppColors.warning;
    return AppColors.error;
  }

  Color _cpuColor(double pct) {
    if (pct < 50) return AppColors.success;
    if (pct < 80) return AppColors.warning;
    return AppColors.error;
  }
}

class _MetricChip extends StatelessWidget {
  final IconData icon;
  final String label;
  final Color color;

  const _MetricChip({
    required this.icon,
    required this.label,
    required this.color,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
      decoration: BoxDecoration(
        color: color.withOpacity(0.1),
        borderRadius: BorderRadius.circular(10),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(icon, size: 14, color: color),
          const SizedBox(width: 4),
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
}
