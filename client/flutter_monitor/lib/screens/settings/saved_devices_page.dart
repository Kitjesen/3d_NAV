import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import '../../theme/app_theme.dart';
import '../../services/settings_preferences.dart';
import '../../services/robot_connection_provider.dart';
import '../../services/robot_client.dart';

class SavedDevicesPage extends StatelessWidget {
  const SavedDevicesPage({super.key});

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final settingsPrefs = context.watch<SettingsPreferences>();
    final devices = settingsPrefs.savedDevices;

    return Scaffold(
      appBar: AppBar(
        title: const Text('已保存的设备'),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back_ios_new, size: 20),
          onPressed: () => Navigator.pop(context),
        ),
        actions: [
          if (devices.isNotEmpty)
            IconButton(
              icon: const Icon(Icons.delete_sweep_outlined),
              tooltip: '清空全部',
              onPressed: () => _confirmClearAll(context, settingsPrefs),
            ),
        ],
      ),
      body: devices.isEmpty
          ? Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Icon(Icons.devices_other,
                      size: 56, color: context.subtitleColor),
                  const SizedBox(height: 16),
                  Text(
                    '暂无已保存的设备',
                    style: TextStyle(
                      fontSize: 16,
                      color: context.subtitleColor,
                    ),
                  ),
                  const SizedBox(height: 8),
                  Text(
                    '连接成功后设备将自动保存在此',
                    style: TextStyle(
                      fontSize: 13,
                      color: context.subtitleColor,
                    ),
                  ),
                ],
              ),
            )
          : ListView.builder(
              padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
              itemCount: devices.length,
              itemBuilder: (context, index) {
                final device = devices[index];
                return _DeviceCard(
                  device: device,
                  onConnect: () => _connectToDevice(context, device),
                  onDelete: () => _confirmDelete(context, settingsPrefs, device),
                  onSetDefault: () => settingsPrefs.setDefaultDevice(device),
                );
              },
            ),
    );
  }

  Future<void> _connectToDevice(BuildContext context, SavedDevice device) async {
    HapticFeedback.mediumImpact();
    final provider = context.read<RobotConnectionProvider>();
    final settingsPrefs = context.read<SettingsPreferences>();

    // Show loading
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (_) => const Center(child: CircularProgressIndicator()),
    );

    try {
      final client = RobotClient(host: device.host, port: device.port);
      final success = await provider.connect(client);

      if (context.mounted) Navigator.pop(context); // dismiss loading

      if (success) {
        // Update last connected time
        await settingsPrefs.addOrUpdateDevice(SavedDevice(
          name: device.name,
          host: device.host,
          port: device.port,
          lastConnected: DateTime.now(),
          isDefault: device.isDefault,
        ));
        if (context.mounted) {
          Navigator.pop(context); // return to settings
          Navigator.of(context).pushNamed('/robot-detail');
        }
      } else {
        if (context.mounted) {
          ScaffoldMessenger.of(context).showSnackBar(
            SnackBar(
              content: Text('连接失败: ${device.host}:${device.port}'),
              behavior: SnackBarBehavior.floating,
              shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(10)),
            ),
          );
        }
      }
    } catch (e) {
      if (context.mounted) {
        Navigator.pop(context); // dismiss loading
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text('连接错误: $e'),
            behavior: SnackBarBehavior.floating,
            shape:
                RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
          ),
        );
      }
    }
  }

  void _confirmDelete(
      BuildContext context, SettingsPreferences prefs, SavedDevice device) {
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('删除设备'),
        content: Text('确定要删除 "${device.name}" (${device.host}) 吗？'),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(ctx), child: const Text('取消')),
          TextButton(
            onPressed: () {
              prefs.removeDevice(device);
              Navigator.pop(ctx);
            },
            style: TextButton.styleFrom(foregroundColor: AppColors.error),
            child: const Text('删除'),
          ),
        ],
      ),
    );
  }

  void _confirmClearAll(BuildContext context, SettingsPreferences prefs) {
    showDialog(
      context: context,
      builder: (ctx) => AlertDialog(
        title: const Text('清空设备'),
        content: const Text('确定要清空所有已保存的设备吗？'),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
        actions: [
          TextButton(
              onPressed: () => Navigator.pop(ctx), child: const Text('取消')),
          TextButton(
            onPressed: () {
              prefs.clearAllDevices();
              Navigator.pop(ctx);
            },
            style: TextButton.styleFrom(foregroundColor: AppColors.error),
            child: const Text('清空'),
          ),
        ],
      ),
    );
  }
}

class _DeviceCard extends StatelessWidget {
  final SavedDevice device;
  final VoidCallback onConnect;
  final VoidCallback onDelete;
  final VoidCallback onSetDefault;

  const _DeviceCard({
    required this.device,
    required this.onConnect,
    required this.onDelete,
    required this.onSetDefault,
  });

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;
    final timeDiff = DateTime.now().difference(device.lastConnected);
    final timeAgo = timeDiff.inDays > 0
        ? '${timeDiff.inDays} 天前'
        : timeDiff.inHours > 0
            ? '${timeDiff.inHours} 小时前'
            : '${timeDiff.inMinutes} 分钟前';

    return Container(
      margin: const EdgeInsets.only(bottom: 12),
      decoration: BoxDecoration(
        color: isDark ? AppColors.darkCard : Colors.white,
        borderRadius: BorderRadius.circular(20),
        boxShadow: [
          BoxShadow(
            color: context.cardShadowColor,
            blurRadius: 14,
            offset: const Offset(0, 5),
          ),
        ],
      ),
      child: Material(
        color: Colors.transparent,
        borderRadius: BorderRadius.circular(20),
        child: InkWell(
          onTap: onConnect,
          borderRadius: BorderRadius.circular(20),
          child: Padding(
            padding: const EdgeInsets.all(16),
            child: Row(
              children: [
                // Robot icon
                Container(
                  width: 48,
                  height: 48,
                  decoration: BoxDecoration(
                    color: AppColors.primary.withOpacity(isDark ? 0.15 : 0.08),
                    borderRadius: BorderRadius.circular(14),
                  ),
                  child: const Icon(Icons.smart_toy_outlined,
                      color: AppColors.primary, size: 24),
                ),
                const SizedBox(width: 14),
                // Info
                Expanded(
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Row(
                        children: [
                          Text(
                            device.name,
                            style: TextStyle(
                              fontSize: 15,
                              fontWeight: FontWeight.w600,
                              color: isDark ? Colors.white : Colors.black87,
                            ),
                          ),
                          if (device.isDefault) ...[
                            const SizedBox(width: 6),
                            Container(
                              padding: const EdgeInsets.symmetric(
                                  horizontal: 6, vertical: 2),
                              decoration: BoxDecoration(
                                color: AppColors.primary.withOpacity(0.1),
                                borderRadius: BorderRadius.circular(6),
                              ),
                              child: const Text(
                                '默认',
                                style: TextStyle(
                                  fontSize: 10,
                                  fontWeight: FontWeight.w600,
                                  color: AppColors.primary,
                                ),
                              ),
                            ),
                          ],
                        ],
                      ),
                      const SizedBox(height: 4),
                      Text(
                        '${device.host}:${device.port}',
                        style: TextStyle(
                          fontSize: 13,
                          color: context.subtitleColor,
                          fontFamily: 'monospace',
                        ),
                      ),
                      const SizedBox(height: 2),
                      Text(
                        '上次连接: $timeAgo',
                        style: TextStyle(
                          fontSize: 11,
                          color: context.subtitleColor,
                        ),
                      ),
                    ],
                  ),
                ),
                // Actions
                PopupMenuButton<String>(
                  icon: Icon(Icons.more_vert,
                      size: 20, color: context.subtitleColor),
                  shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(14)),
                  onSelected: (value) {
                    switch (value) {
                      case 'connect':
                        onConnect();
                      case 'default':
                        onSetDefault();
                      case 'delete':
                        onDelete();
                    }
                  },
                  itemBuilder: (_) => [
                    const PopupMenuItem(
                        value: 'connect', child: Text('连接')),
                    const PopupMenuItem(
                        value: 'default', child: Text('设为默认')),
                    const PopupMenuItem(
                        value: 'delete',
                        child:
                            Text('删除', style: TextStyle(color: AppColors.error))),
                  ],
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
