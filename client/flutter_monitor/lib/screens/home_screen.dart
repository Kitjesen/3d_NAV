import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import '../services/robot_client_base.dart';
import 'status_screen.dart';
import 'control_screen.dart';
import 'map_screen.dart';
import 'events_screen.dart';

class HomeScreen extends StatefulWidget {
  final RobotClientBase client;

  const HomeScreen({super.key, required this.client});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  int _currentIndex = 0;
  late final List<Widget> _screens;

  @override
  void initState() {
    super.initState();
    // 强制主界面竖屏
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);

    _screens = [
      StatusScreen(client: widget.client),
      const SizedBox(), // Placeholder for Control
      MapScreen(client: widget.client),
      EventsScreen(client: widget.client),
    ];
  }

  void _onTabSelected(int index) {
    if (index == 1) {
      // Control Tab clicked -> Push ControlScreen
      Navigator.of(context).push(
        MaterialPageRoute(
          builder: (context) => ControlScreen(client: widget.client),
        ),
      );
    } else {
      setState(() {
        _currentIndex = index;
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      extendBody: true,
      body: Stack(
        children: [
          IndexedStack(
            index: _currentIndex,
            children: _screens,
          ),
          _buildFloatingNavBar(context),
        ],
      ),
    );
  }

  Widget _buildFloatingNavBar(BuildContext context) {
    final bottomPadding = MediaQuery.of(context).padding.bottom;
    return Align(
      alignment: Alignment.bottomCenter,
      child: SafeArea(
        top: false,
        child: Padding(
          padding: EdgeInsets.fromLTRB(16, 0, 16, 12 + bottomPadding * 0.2),
          child: ClipRRect(
            borderRadius: BorderRadius.circular(28),
            child: BackdropFilter(
              filter: ImageFilter.blur(sigmaX: 16, sigmaY: 16),
              child: Container(
                decoration: BoxDecoration(
                  color: Theme.of(context).colorScheme.surface.withOpacity(0.7),
                  borderRadius: BorderRadius.circular(28),
                  border: Border.all(
                    color: Colors.white.withOpacity(0.2),
                  ),
                  boxShadow: [
                    BoxShadow(
                      color: Colors.black.withOpacity(0.15),
                      blurRadius: 16,
                      offset: const Offset(0, 8),
                    ),
                  ],
                ),
                child: NavigationBar(
                  height: 64,
                  backgroundColor: Colors.transparent,
                  elevation: 0,
                  indicatorColor: Colors.white.withOpacity(0.2),
                  labelBehavior: NavigationDestinationLabelBehavior.alwaysHide,
                  selectedIndex: _currentIndex,
                  onDestinationSelected: _onTabSelected,
                  destinations: const [
                    NavigationDestination(
                      icon: Icon(Icons.dashboard_outlined, color: Colors.black54),
                      selectedIcon: Icon(Icons.dashboard, color: Colors.black87),
                      label: 'Status',
                    ),
                    NavigationDestination(
                      icon: Icon(Icons.gamepad_outlined, color: Colors.black54),
                      selectedIcon: Icon(Icons.gamepad, color: Colors.black87),
                      label: 'Control',
                    ),
                    NavigationDestination(
                      icon: Icon(Icons.map_outlined, color: Colors.black54),
                      selectedIcon: Icon(Icons.map, color: Colors.black87),
                      label: 'Map',
                    ),
                    NavigationDestination(
                      icon: Icon(Icons.notifications_outlined, color: Colors.black54),
                      selectedIcon: Icon(Icons.notifications, color: Colors.black87),
                      label: 'Events',
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
