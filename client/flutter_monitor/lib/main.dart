import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:animations/animations.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/storage/settings_preferences.dart';
import 'package:flutter_monitor/core/services/alert_monitor_service.dart';
import 'package:flutter_monitor/core/services/state_logger_service.dart';
import 'package:flutter_monitor/features/connection/splash_screen.dart';
import 'package:flutter_monitor/features/home/main_shell_screen.dart';
import 'package:flutter_monitor/features/connection/scan_screen.dart';
import 'package:flutter_monitor/features/home/robot_detail_screen.dart';
import 'package:flutter_monitor/features/settings/app_settings_screen.dart';
import 'package:flutter_monitor/features/control/control_screen.dart';
import 'package:flutter_monitor/features/robot_select/robot_select_screen.dart';
import 'package:flutter_monitor/core/providers/robot_profile_provider.dart';

import 'package:flutter_monitor/core/services/notification_service.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  SystemChrome.setSystemUIOverlayStyle(
    const SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
    ),
  );

  // 初始化系统通知服务
  await NotificationService().initialize();

  runApp(const RobotMonitorApp());
}

class RobotMonitorApp extends StatelessWidget {
  const RobotMonitorApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => RobotConnectionProvider()),
        ChangeNotifierProvider(create: (_) => ThemeProvider()),
        ChangeNotifierProvider(create: (_) => SettingsPreferences()),
        ChangeNotifierProvider(create: (_) => RobotProfileProvider()),
        ChangeNotifierProvider(create: (_) => AlertMonitorService()),
        ChangeNotifierProvider(create: (_) => StateLoggerService()),
      ],
      child: _AppWithBindings(),
    );
  }
}

/// Binds SettingsPreferences -> RobotConnectionProvider after both are created.
class _AppWithBindings extends StatefulWidget {
  @override
  State<_AppWithBindings> createState() => _AppWithBindingsState();
}

class _AppWithBindingsState extends State<_AppWithBindings> {
  bool _bound = false;
  StreamSubscription<AlertRecord>? _alertSub;

  /// 全局 ScaffoldMessenger key，用于从任意位置弹 SnackBar
  final GlobalKey<ScaffoldMessengerState> _messengerKey =
      GlobalKey<ScaffoldMessengerState>();

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    if (!_bound) {
      final connProvider = context.read<RobotConnectionProvider>();
      final settingsPrefs = context.read<SettingsPreferences>();
      final alertService = context.read<AlertMonitorService>();
      final stateLogger = context.read<StateLoggerService>();

      final profileProvider = context.read<RobotProfileProvider>();
      connProvider.bindSettings(settingsPrefs);
      connProvider.bindProfileProvider(profileProvider);
      alertService.bind(
        connProvider: connProvider,
        settingsPrefs: settingsPrefs,
      );
      stateLogger.bind(connProvider);

      // 监听告警流，展示 SnackBar
      _alertSub = alertService.alertStream.listen(_showAlertSnackBar);

      _bound = true;
    }
  }

  void _showAlertSnackBar(AlertRecord alert) {
    final prefs = context.read<SettingsPreferences>();
    if (!prefs.alertInApp) return; // 用户关闭了 APP 内提醒

    final Color accentColor;
    switch (alert.level) {
      case AlertLevel.critical:
        accentColor = AppColors.error;
      case AlertLevel.warning:
        accentColor = AppColors.warning;
      case AlertLevel.info:
        accentColor = AppColors.primary;
    }

    final brightness = Theme.of(context).brightness;
    final isDark = brightness == Brightness.dark;

    _messengerKey.currentState?.showSnackBar(
      SnackBar(
        content: Row(
          children: [
            // Left accent strip
            Container(
              width: 3,
              height: 36,
              decoration: BoxDecoration(
                color: accentColor,
                borderRadius: BorderRadius.circular(2),
              ),
            ),
            const SizedBox(width: 12),
            Expanded(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(
                    alert.title,
                    style: TextStyle(
                      fontWeight: FontWeight.w600,
                      fontSize: 13,
                      color: isDark ? Colors.white : const Color(0xFF1A1A1A),
                    ),
                  ),
                  const SizedBox(height: 2),
                  Text(
                    alert.message,
                    style: TextStyle(
                      fontSize: 12,
                      color: isDark
                          ? Colors.white.withOpacity(0.5)
                          : Colors.black.withOpacity(0.45),
                    ),
                  ),
                ],
              ),
            ),
            const SizedBox(width: 8),
            // Dismiss button
            GestureDetector(
              onTap: () {
                _messengerKey.currentState?.hideCurrentSnackBar();
              },
              child: Icon(
                Icons.close,
                size: 16,
                color: isDark
                    ? Colors.white.withOpacity(0.35)
                    : Colors.black.withOpacity(0.3),
              ),
            ),
          ],
        ),
        backgroundColor: isDark ? const Color(0xFF1C1C1E) : Colors.white,
        behavior: SnackBarBehavior.floating,
        elevation: 0,
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(12),
          side: BorderSide(
            color: isDark
                ? Colors.white.withOpacity(0.08)
                : const Color(0xFFE5E5E5),
          ),
        ),
        margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
        duration: const Duration(seconds: 5),
      ),
    );
  }

  @override
  void dispose() {
    _alertSub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Consumer<ThemeProvider>(
      builder: (context, themeProvider, _) {
        return MaterialApp(
          scaffoldMessengerKey: _messengerKey,
          title: '大算机器人',
          debugShowCheckedModeBanner: false,
          theme: lightTheme,
          darkTheme: darkTheme,
          themeMode: themeProvider.mode,
          initialRoute: '/',
          onGenerateRoute: _generateRoute,
        );
      },
    );
  }

  Route<dynamic>? _generateRoute(RouteSettings settings) {
    Widget page;
    switch (settings.name) {
      case '/':
        page = const SplashScreen();
      case '/main':
        page = const MainShellScreen();
      case '/scan':
        page = const ScanScreen();
      case '/robot-detail':
        page = const RobotDetailScreen();
      case '/settings':
        page = const AppSettingsScreen();
      case '/control':
        page = const ControlScreen();
      case '/robot-select':
        page = const RobotSelectScreen();
      default:
        page = const MainShellScreen();
    }

    return PageRouteBuilder(
      settings: settings,
      pageBuilder: (_, __, ___) => page,
      transitionsBuilder: (context, animation, secondaryAnimation, child) {
        return SharedAxisTransition(
          animation: animation,
          secondaryAnimation: secondaryAnimation,
          transitionType: SharedAxisTransitionType.horizontal,
          child: child,
        );
      },
      transitionDuration: const Duration(milliseconds: 300),
    );
  }
}
