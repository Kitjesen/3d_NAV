import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:math' as math;
import '../services/robot_client_base.dart';
import '../generated/telemetry.pb.dart';
import '../widgets/glass_widgets.dart';

class StatusScreen extends StatefulWidget {
  final RobotClientBase client;

  const StatusScreen({super.key, required this.client});

  @override
  State<StatusScreen> createState() => _StatusScreenState();
}

class _StatusScreenState extends State<StatusScreen>
    with SingleTickerProviderStateMixin {
  StreamSubscription<FastState>? _fastStateSubscription;
  StreamSubscription<SlowState>? _slowStateSubscription;

  FastState? _latestFastState;
  SlowState? _latestSlowState;

  int _updateCount = 0;

  late AnimationController _breathingController;
  late Animation<double> _breathingAnimation;

  @override
  void initState() {
    super.initState();
    _startStreaming();

    _breathingController = AnimationController(
      vsync: this,
      duration: const Duration(seconds: 2),
    )..repeat(reverse: true);

    _breathingAnimation = Tween<double>(begin: 0.4, end: 1.0).animate(
      CurvedAnimation(parent: _breathingController, curve: Curves.easeInOut),
    );
  }

  void _startStreaming() {
    _fastStateSubscription =
        widget.client.streamFastState(desiredHz: 10.0).listen(
      (state) {
        if (mounted) {
          setState(() {
            _latestFastState = state;
            _updateCount++;
          });
        }
      },
      onError: (error) {
        debugPrint('FastState error: $error');
      },
    );

    _slowStateSubscription = widget.client.streamSlowState().listen(
      (state) {
        if (mounted) {
          setState(() {
            _latestSlowState = state;
          });
        }
      },
      onError: (error) {
        debugPrint('SlowState error: $error');
      },
    );
  }

  @override
  void dispose() {
    _fastStateSubscription?.cancel();
    _slowStateSubscription?.cancel();
    _breathingController.dispose();
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
            child: _buildStatusBadge(),
          ),
        ],
      ),
      body: _latestFastState == null
          ? const Center(
              child: CircularProgressIndicator(color: Color(0xFF007AFF)),
            )
          : RefreshIndicator(
              onRefresh: () async {
                setState(() => _updateCount = 0);
              },
              child: SingleChildScrollView(
                physics: const AlwaysScrollableScrollPhysics(),
                padding: EdgeInsets.fromLTRB(
                    16, MediaQuery.of(context).padding.top + 60, 16, 100),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    _buildRobotCard(),
                    const SizedBox(height: 16),
                    Row(
                      children: [
                        Expanded(child: _buildMetricCard(
                          icon: Icons.battery_charging_full,
                          iconColor: const Color(0xFF34C759),
                          label: 'BATTERY',
                          value: (_latestSlowState?.resources.batteryPercent ?? 0).toStringAsFixed(0),
                          unit: '%',
                          progress: (_latestSlowState?.resources.batteryPercent ?? 0) / 100,
                          progressColor: const Color(0xFF34C759),
                        )),
                        const SizedBox(width: 12),
                        Expanded(child: _buildMetricCard(
                          icon: Icons.memory,
                          iconColor: const Color(0xFF007AFF),
                          label: 'CPU',
                          value: (_latestSlowState?.resources.cpuPercent ?? 0).toStringAsFixed(0),
                          unit: '%',
                          progress: (_latestSlowState?.resources.cpuPercent ?? 0) / 100,
                          progressColor: const Color(0xFF007AFF),
                        )),
                      ],
                    ),
                    const SizedBox(height: 12),
                    Row(
                      children: [
                        Expanded(child: _buildMetricCard(
                          icon: Icons.thermostat,
                          iconColor: const Color(0xFFFF9500),
                          label: 'TEMP',
                          value: (_latestSlowState?.resources.cpuTemp ?? 0).toStringAsFixed(1),
                          unit: '°C',
                          progress: math.min((_latestSlowState?.resources.cpuTemp ?? 0) / 80, 1.0),
                          progressColor: const Color(0xFFFF9500),
                        )),
                        const SizedBox(width: 12),
                        Expanded(child: _buildMetricCard(
                          icon: Icons.bolt,
                          iconColor: const Color(0xFFFFCC00),
                          label: 'VOLTAGE',
                          value: (_latestSlowState?.resources.batteryVoltage ?? 0).toStringAsFixed(1),
                          unit: 'V',
                          progress: math.min((_latestSlowState?.resources.batteryVoltage ?? 0) / 30, 1.0),
                          progressColor: const Color(0xFFFFCC00),
                        )),
                      ],
                    ),
                    const SizedBox(height: 16),
                    _buildMotionCard(),
                    if (_latestSlowState != null) ...[
                      const SizedBox(height: 16),
                      _buildTopicRatesCard(),
                    ],
                  ],
                ),
              ),
            ),
    );
  }

  Widget _buildStatusBadge() {
    final isOnline = _latestFastState != null;
    return GlassCard(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      borderRadius: 30,
      blurSigma: 10,
      color: isOnline
          ? const Color(0xFF34C759).withOpacity(0.1)
          : const Color(0xFFFF3B30).withOpacity(0.1),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          AnimatedBuilder(
            animation: _breathingAnimation,
            builder: (context, child) {
              return Container(
                width: 8,
                height: 8,
                decoration: BoxDecoration(
                  color: isOnline
                      ? const Color(0xFF34C759)
                      : const Color(0xFFFF3B30),
                  shape: BoxShape.circle,
                  boxShadow: isOnline
                      ? [
                          BoxShadow(
                            color: const Color(0xFF34C759)
                                .withOpacity(0.5 * _breathingAnimation.value),
                            blurRadius: 6,
                            spreadRadius: 1,
                          ),
                        ]
                      : [],
                ),
              );
            },
          ),
          const SizedBox(width: 8),
          Text(
            isOnline ? 'ONLINE' : 'OFFLINE',
            style: TextStyle(
              fontSize: 11,
              fontWeight: FontWeight.w700,
              letterSpacing: 0.5,
              color: isOnline
                  ? const Color(0xFF34C759)
                  : const Color(0xFFFF3B30),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildRobotCard() {
    final pose = _latestFastState!.pose;
    return GlassCard(
      padding: const EdgeInsets.all(20),
      child: Column(
        children: [
          // 简洁的机器人图标
          Container(
            width: 72,
            height: 72,
            decoration: BoxDecoration(
              gradient: const LinearGradient(
                begin: Alignment.topLeft,
                end: Alignment.bottomRight,
                colors: [Color(0xFF007AFF), Color(0xFF5856D6)],
              ),
              borderRadius: BorderRadius.circular(20),
              boxShadow: [
                BoxShadow(
                  color: const Color(0xFF007AFF).withOpacity(0.3),
                  blurRadius: 16,
                  offset: const Offset(0, 6),
                ),
              ],
            ),
            child: const Icon(Icons.smart_toy, size: 36, color: Colors.white),
          ),
          const SizedBox(height: 16),
          Text(
            '大算机器人',
            style: TextStyle(
              fontSize: 18,
              fontWeight: FontWeight.w700,
              color: Colors.black.withOpacity(0.8),
            ),
          ),
          const SizedBox(height: 4),
          Text(
            'Mode: ${_latestSlowState?.currentMode ?? "IDLE"}',
            style: TextStyle(
              fontSize: 12,
              color: Colors.black.withOpacity(0.4),
              letterSpacing: 0.5,
            ),
          ),
          const SizedBox(height: 20),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              _buildPoseValue('X', pose.position.x.toStringAsFixed(2), 'm'),
              _buildDivider(),
              _buildPoseValue('Y', pose.position.y.toStringAsFixed(2), 'm'),
              _buildDivider(),
              _buildPoseValue('YAW', _latestFastState!.rpyDeg.z.toStringAsFixed(1), '°'),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildPoseValue(String label, String value, String unit) {
    return Column(
      children: [
        Text(
          label,
          style: TextStyle(
            fontSize: 10,
            fontWeight: FontWeight.w600,
            color: Colors.black.withOpacity(0.35),
            letterSpacing: 1.0,
          ),
        ),
        const SizedBox(height: 4),
        RichText(
          text: TextSpan(
            children: [
              TextSpan(
                text: value,
                style: const TextStyle(
                  fontSize: 22,
                  fontWeight: FontWeight.w700,
                  color: Colors.black87,
                  letterSpacing: -0.5,
                ),
              ),
              TextSpan(
                text: ' $unit',
                style: TextStyle(
                  fontSize: 13,
                  fontWeight: FontWeight.w500,
                  color: Colors.black.withOpacity(0.4),
                ),
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildDivider() {
    return Container(
      width: 1,
      height: 36,
      color: Colors.black.withOpacity(0.08),
    );
  }

  Widget _buildMetricCard({
    required IconData icon,
    required Color iconColor,
    required String label,
    required String value,
    required String unit,
    required double progress,
    required Color progressColor,
  }) {
    return GlassCard(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                width: 32,
                height: 32,
                decoration: BoxDecoration(
                  color: iconColor.withOpacity(0.12),
                  borderRadius: BorderRadius.circular(10),
                ),
                child: Icon(icon, size: 18, color: iconColor),
              ),
              const Spacer(),
              Text(
                label,
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: Colors.black.withOpacity(0.35),
                  letterSpacing: 1.0,
                ),
              ),
            ],
          ),
          const SizedBox(height: 14),
          RichText(
            text: TextSpan(
              children: [
                TextSpan(
                  text: value,
                  style: const TextStyle(
                    fontSize: 26,
                    fontWeight: FontWeight.w700,
                    color: Colors.black87,
                    letterSpacing: -0.5,
                  ),
                ),
                TextSpan(
                  text: ' $unit',
                  style: TextStyle(
                    fontSize: 14,
                    fontWeight: FontWeight.w500,
                    color: Colors.black.withOpacity(0.4),
                  ),
                ),
              ],
            ),
          ),
          const SizedBox(height: 10),
          ClipRRect(
            borderRadius: BorderRadius.circular(4),
            child: LinearProgressIndicator(
              value: progress.clamp(0.0, 1.0),
              minHeight: 4,
              backgroundColor: Colors.black.withOpacity(0.06),
              valueColor: AlwaysStoppedAnimation<Color>(progressColor),
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildMotionCard() {
    final linear = _latestFastState!.velocity.linear.x;
    final angular = _latestFastState!.velocity.angular.z;

    return GlassCard(
      padding: const EdgeInsets.all(20),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                width: 32,
                height: 32,
                decoration: BoxDecoration(
                  color: const Color(0xFFAF52DE).withOpacity(0.12),
                  borderRadius: BorderRadius.circular(10),
                ),
                child: const Icon(Icons.speed, size: 18, color: Color(0xFFAF52DE)),
              ),
              const SizedBox(width: 10),
              Text(
                'MOTION',
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: Colors.black.withOpacity(0.35),
                  letterSpacing: 1.0,
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          Row(
            children: [
              Expanded(
                child: _buildMotionValue(
                  Icons.arrow_forward,
                  'Linear',
                  linear.toStringAsFixed(2),
                  'm/s',
                  const Color(0xFF007AFF),
                ),
              ),
              Container(width: 1, height: 48, color: Colors.black.withOpacity(0.06)),
              Expanded(
                child: _buildMotionValue(
                  Icons.rotate_right,
                  'Angular',
                  angular.toStringAsFixed(2),
                  'rad/s',
                  const Color(0xFFAF52DE),
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildMotionValue(
    IconData icon,
    String label,
    String value,
    String unit,
    Color color,
  ) {
    return Column(
      children: [
        Icon(icon, size: 20, color: color.withOpacity(0.6)),
        const SizedBox(height: 8),
        RichText(
          textAlign: TextAlign.center,
          text: TextSpan(
            children: [
              TextSpan(
                text: value,
                style: const TextStyle(
                  fontSize: 20,
                  fontWeight: FontWeight.w700,
                  color: Colors.black87,
                ),
              ),
              TextSpan(
                text: '\n$unit',
                style: TextStyle(
                  fontSize: 11,
                  color: Colors.black.withOpacity(0.4),
                ),
              ),
            ],
          ),
        ),
        const SizedBox(height: 4),
        Text(
          label,
          style: TextStyle(
            fontSize: 10,
            fontWeight: FontWeight.w600,
            color: Colors.black.withOpacity(0.3),
            letterSpacing: 0.8,
          ),
        ),
      ],
    );
  }

  Widget _buildTopicRatesCard() {
    final rates = _latestSlowState!.topicRates;
    return GlassCard(
      padding: const EdgeInsets.all(20),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Container(
                width: 32,
                height: 32,
                decoration: BoxDecoration(
                  color: const Color(0xFF5AC8FA).withOpacity(0.12),
                  borderRadius: BorderRadius.circular(10),
                ),
                child: const Icon(Icons.wifi, size: 18, color: Color(0xFF5AC8FA)),
              ),
              const SizedBox(width: 10),
              Text(
                'TOPIC RATES',
                style: TextStyle(
                  fontSize: 10,
                  fontWeight: FontWeight.w600,
                  color: Colors.black.withOpacity(0.35),
                  letterSpacing: 1.0,
                ),
              ),
            ],
          ),
          const SizedBox(height: 16),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceAround,
            children: [
              _buildRateChip('Odom', rates.odomHz, const Color(0xFF007AFF)),
              _buildRateChip('Lidar', rates.lidarHz, const Color(0xFF34C759)),
              _buildRateChip('Map', rates.terrainMapHz, const Color(0xFFFF9500)),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildRateChip(String label, double hz, Color color) {
    return Column(
      children: [
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 6),
          decoration: BoxDecoration(
            color: color.withOpacity(0.1),
            borderRadius: BorderRadius.circular(20),
          ),
          child: Text(
            '${hz.toStringAsFixed(1)} Hz',
            style: TextStyle(
              fontSize: 14,
              fontWeight: FontWeight.w700,
              color: color,
            ),
          ),
        ),
        const SizedBox(height: 6),
        Text(
          label,
          style: TextStyle(
            fontSize: 11,
            color: Colors.black.withOpacity(0.4),
          ),
        ),
      ],
    );
  }
}
