import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:shared_preferences/shared_preferences.dart';

// ============================================================
// App color constants
// ============================================================

class AppColors {
  // Brand
  static const primary = Color(0xFF007AFF);
  static const secondary = Color(0xFF5856D6);
  static const accent = Color(0xFFAF52DE);

  // Semantic
  static const success = Color(0xFF34C759);
  static const warning = Color(0xFFFF9500);
  static const error = Color(0xFFFF3B30);
  static const info = Color(0xFF5AC8FA);

  // Light surfaces
  static const lightBackground = Color(0xFFF2F2F7);
  static const lightSurface = Colors.white;
  static const lightCard = Color(0xFFFFFFFF);

  // Dark surfaces
  static const darkBackground = Color(0xFF000000);
  static const darkSurface = Color(0xFF1C1C1E);
  static const darkCard = Color(0xFF2C2C2E);
  static const darkElevated = Color(0xFF3A3A3C);

  // Glass
  static Color glassLight = Colors.white.withOpacity(0.85);
  static Color glassDark = const Color(0xFF1C1C1E).withOpacity(0.78);
  static Color glassBorderLight = Colors.white.withOpacity(0.5);
  static Color glassBorderDark = Colors.white.withOpacity(0.08);
}

// ============================================================
// Theme mode provider (ChangeNotifier)
// ============================================================

class ThemeProvider extends ChangeNotifier {
  static const _key = 'theme_mode';
  ThemeMode _mode = ThemeMode.system;

  ThemeMode get mode => _mode;
  bool get isDark => _mode == ThemeMode.dark;

  ThemeProvider() {
    _load();
  }

  Future<void> _load() async {
    final prefs = await SharedPreferences.getInstance();
    final value = prefs.getString(_key);
    if (value == 'light') {
      _mode = ThemeMode.light;
    } else if (value == 'dark') {
      _mode = ThemeMode.dark;
    } else {
      _mode = ThemeMode.system;
    }
    notifyListeners();
  }

  Future<void> setMode(ThemeMode mode) async {
    _mode = mode;
    notifyListeners();
    final prefs = await SharedPreferences.getInstance();
    switch (mode) {
      case ThemeMode.light:
        await prefs.setString(_key, 'light');
      case ThemeMode.dark:
        await prefs.setString(_key, 'dark');
      case ThemeMode.system:
        await prefs.remove(_key);
    }
  }
}

// ============================================================
// Light theme
// ============================================================

final ThemeData lightTheme = ThemeData(
  useMaterial3: true,
  brightness: Brightness.light,
  scaffoldBackgroundColor: AppColors.lightBackground,
  colorScheme: ColorScheme.fromSeed(
    seedColor: AppColors.primary,
    surface: AppColors.lightSurface,
    brightness: Brightness.light,
  ),
  appBarTheme: const AppBarTheme(
    backgroundColor: Colors.transparent,
    elevation: 0,
    scrolledUnderElevation: 0,
    centerTitle: true,
    systemOverlayStyle: SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.dark,
    ),
    titleTextStyle: TextStyle(
      color: Colors.black87,
      fontSize: 17,
      fontWeight: FontWeight.w600,
      letterSpacing: -0.5,
    ),
    iconTheme: IconThemeData(color: Colors.black87),
  ),
  cardTheme: CardThemeData(
    color: AppColors.lightCard,
    elevation: 0,
    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(22)),
    margin: EdgeInsets.zero,
  ),
  dividerTheme: DividerThemeData(
    color: Colors.black.withOpacity(0.06),
    thickness: 0.5,
    space: 0,
  ),
  listTileTheme: const ListTileThemeData(
    contentPadding: EdgeInsets.symmetric(horizontal: 16, vertical: 2),
    minLeadingWidth: 24,
  ),
  switchTheme: SwitchThemeData(
    thumbColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) return AppColors.primary;
      return Colors.white;
    }),
    trackColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) {
        return AppColors.primary.withOpacity(0.4);
      }
      return Colors.black.withOpacity(0.1);
    }),
  ),
);

// ============================================================
// Dark theme
// ============================================================

final ThemeData darkTheme = ThemeData(
  useMaterial3: true,
  brightness: Brightness.dark,
  scaffoldBackgroundColor: AppColors.darkBackground,
  colorScheme: ColorScheme.fromSeed(
    seedColor: AppColors.primary,
    surface: AppColors.darkSurface,
    brightness: Brightness.dark,
  ),
  appBarTheme: const AppBarTheme(
    backgroundColor: Colors.transparent,
    elevation: 0,
    scrolledUnderElevation: 0,
    centerTitle: true,
    systemOverlayStyle: SystemUiOverlayStyle(
      statusBarColor: Colors.transparent,
      statusBarIconBrightness: Brightness.light,
    ),
    titleTextStyle: TextStyle(
      color: Colors.white,
      fontSize: 17,
      fontWeight: FontWeight.w600,
      letterSpacing: -0.5,
    ),
    iconTheme: IconThemeData(color: Colors.white),
  ),
  cardTheme: CardThemeData(
    color: AppColors.darkCard,
    elevation: 0,
    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(22)),
    margin: EdgeInsets.zero,
  ),
  dividerTheme: DividerThemeData(
    color: Colors.white.withOpacity(0.08),
    thickness: 0.5,
    space: 0,
  ),
  listTileTheme: const ListTileThemeData(
    contentPadding: EdgeInsets.symmetric(horizontal: 16, vertical: 2),
    minLeadingWidth: 24,
  ),
  switchTheme: SwitchThemeData(
    thumbColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) return AppColors.primary;
      return Colors.grey;
    }),
    trackColor: WidgetStateProperty.resolveWith((states) {
      if (states.contains(WidgetState.selected)) {
        return AppColors.primary.withOpacity(0.4);
      }
      return Colors.white.withOpacity(0.1);
    }),
  ),
);

// ============================================================
// Theme helper extensions
// ============================================================

extension ThemeContextExtension on BuildContext {
  bool get isDark => Theme.of(this).brightness == Brightness.dark;
  Color get glassColor => isDark ? AppColors.glassDark : AppColors.glassLight;
  Color get glassBorder =>
      isDark ? AppColors.glassBorderDark : AppColors.glassBorderLight;
  Color get subtitleColor =>
      isDark ? Colors.white.withOpacity(0.5) : Colors.black.withOpacity(0.45);
  Color get cardShadowColor => isDark
      ? Colors.black.withOpacity(0.3)
      : Colors.black.withOpacity(0.08);
  Color get dividerColor =>
      isDark ? Colors.white.withOpacity(0.08) : Colors.black.withOpacity(0.06);
  Color get inputFillColor =>
      isDark ? Colors.white.withOpacity(0.06) : Colors.black.withOpacity(0.04);
}
