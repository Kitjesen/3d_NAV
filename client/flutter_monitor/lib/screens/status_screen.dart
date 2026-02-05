import 'package:flutter/material.dart';
import 'dart:async';
import '../services/robot_client_base.dart';
import '../generated/telemetry.pb.dart';
import '../widgets/glass_widgets.dart';
import 'package:flutter/services.dart';

class StatusScreen extends StatefulWidget {
  final RobotClientBase client;

  const StatusScreen({super.key, required this.client});

  @override
  State<StatusScreen> createState() => _StatusScreenState();
}

class _StatusScreenState extends State<StatusScreen> {
  StreamSubscription<FastState>? _fastStateSubscription;
  StreamSubscription<SlowState>? _slowStateSubscription;
  
  FastState? _latestFastState;
  SlowState? _latestSlowState;
  
  int _updateCount = 0;
  DateTime? _lastUpdateTime;
  String _connectionStatus = 'Connected';

  @override
  void initState() {
    super.initState();
    _startStreaming();
  }

  void _startStreaming() {
    // 订阅快速状态流（10Hz）
    _fastStateSubscription = widget.client.streamFastState(desiredHz: 10.0).listen(
      (state) {
        if (mounted) {
          setState(() {
            _latestFastState = state;
            _updateCount++;
            _lastUpdateTime = DateTime.now();
            _connectionStatus = 'Connected (${_updateCount} updates)';
          });
        }
      },
      onError: (error) {
        if (mounted) {
          setState(() {
            _connectionStatus = 'Error: $error';
          });
        }
      },
      onDone: () {
        if (mounted) {
          setState(() {
            _connectionStatus = 'Stream closed';
          });
        }
      },
    );

    // 订阅慢速状态流（1Hz）
    _slowStateSubscription = widget.client.streamSlowState().listen(
      (state) {
        if (mounted) {
          setState(() {
            _latestSlowState = state;
          });
        }
      },
      onError: (error) {
        print('SlowState error: $error');
      },
    );
  }

  @override
  void dispose() {
    _fastStateSubscription?.cancel();
    _slowStateSubscription?.cancel();
    // widget.client.disconnect(); // Do not disconnect shared client
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      extendBodyBehindAppBar: true,
      appBar: AppBar(
        title: const Text('Overview'),
        actions: [
          Padding(
            padding: const EdgeInsets.only(right: 16.0),
            child: GlassCard(
              padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
              borderRadius: 30,
              blurSigma: 10,
              color: _latestFastState != null ? Colors.green.withOpacity(0.1) : Colors.red.withOpacity(0.1),
              child: Row(
                children: [
                  Container(
                    width: 8,
                    height: 8,
                    decoration: BoxDecoration(
                      color: _latestFastState != null ? const Color(0xFF34C759) : const Color(0xFFFF3B30),
                      shape: BoxShape.circle,
                    ),
                  ),
                  const SizedBox(width: 8),
                  Text(
                    _latestFastState != null ? 'ONLINE' : 'OFFLINE',
                    style: const TextStyle(
                      fontSize: 12,
                      fontWeight: FontWeight.bold,
                      letterSpacing: 0.5,
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
      body: _latestFastState == null
          ? const Center(child: CircularProgressIndicator())
          : RefreshIndicator(
              onRefresh: () async {
                setState(() {
                  _updateCount = 0;
                });
              },
              child: SingleChildScrollView(
                physics: const AlwaysScrollableScrollPhysics(),
                padding: EdgeInsets.fromLTRB(16, MediaQuery.of(context).padding.top + 60, 16, 100),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    _buildMainStatusCard(),
                    const SizedBox(height: 16),
                    Row(
                      children: [
                        Expanded(child: _buildBatteryCard()),
                        const SizedBox(width: 16),
                        Expanded(child: _buildSystemCard()),
                      ],
                    ),
                    const SizedBox(height: 16),
                    _buildMotionCard(),
                    if (_latestSlowState != null) ...[
                      const SizedBox(height: 16),
                      _buildNetworkCard(),
                    ],
                  ],
                ),
              ),
            ),
    );
  }

  Widget _buildMainStatusCard() {
    final pose = _latestFastState!.pose;
    return GlassCard(
      padding: const EdgeInsets.all(24),
      child: Column(
        children: [
          // Using Icon as fallback/primary if asset not loaded, but user asked for generated icons.
          // Since I can't guarantee asset loading in preview, I'll use a large icon or the asset if I can.
          // Let's use the asset I generated.
          Image.asset('assets/icon_robot_model.png', height: 120, color: Colors.black87),
          const SizedBox(height: 24),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceAround,
            children: [
              ValueDisplay(label: 'Position X', value: pose.position.x.toStringAsFixed(2), unit: 'm'),
              ValueDisplay(label: 'Position Y', value: pose.position.y.toStringAsFixed(2), unit: 'm'),
              ValueDisplay(label: 'Heading', value: _latestFastState!.rpyDeg.z.toStringAsFixed(1), unit: '°'),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildBatteryCard() {
    final battery = _latestSlowState?.resources.batteryPercent ?? 0;
    return GlassCard(
      padding: const EdgeInsets.all(16),
      child: ValueDisplay(
        label: 'Battery',
        value: battery.toStringAsFixed(0),
        unit: '%',
        icon: Icon(
          battery > 20 ? Icons.battery_full : Icons.battery_alert,
          color: battery > 20 ? Colors.green : Colors.red,
        ),
      ),
    );
  }

  Widget _buildSystemCard() {
    final cpu = _latestSlowState?.resources.cpuPercent ?? 0;
    return GlassCard(
      padding: const EdgeInsets.all(16),
      child: ValueDisplay(
        label: 'CPU Load',
        value: cpu.toStringAsFixed(0),
        unit: '%',
        icon: const Icon(Icons.memory, color: Colors.blue),
      ),
    );
  }

  Widget _buildMotionCard() {
    final linear = _latestFastState!.velocity.linear.x;
    final angular = _latestFastState!.velocity.angular.z;
    
    return GlassCard(
      padding: const EdgeInsets.all(20),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: [
          ValueDisplay(
            label: 'Linear Velocity',
            value: linear.toStringAsFixed(2),
            unit: 'm/s',
            icon: const Icon(Icons.speed, color: Colors.orange),
          ),
          Container(width: 1, height: 40, color: Colors.black12),
          ValueDisplay(
            label: 'Angular Velocity',
            value: angular.toStringAsFixed(2),
            unit: 'rad/s',
            icon: const Icon(Icons.rotate_right, color: Colors.purple),
          ),
        ],
      ),
    );
  }

  Widget _buildNetworkCard() {
    final rates = _latestSlowState!.topicRates;
    return GlassCard(
      padding: const EdgeInsets.all(20),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              const Icon(Icons.wifi, size: 20, color: Colors.black54),
              const SizedBox(width: 8),
              Text(
                'TOPIC RATES',
                style: TextStyle(
                  fontSize: 11,
                  fontWeight: FontWeight.w600,
                  color: Colors.black.withOpacity(0.4),
                  letterSpacing: 1.0,
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              _buildMiniRate('Odom', rates.odomHz),
              _buildMiniRate('Lidar', rates.lidarHz),
              _buildMiniRate('Map', rates.terrainMapHz),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildMiniRate(String label, double hz) {
    return Column(
      children: [
        Text(hz.toStringAsFixed(1), style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 16)),
        Text(label, style: const TextStyle(fontSize: 12, color: Colors.black45)),
      ],
    );
  }
}
