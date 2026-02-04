import 'package:flutter/material.dart';
import 'dart:async';
import '../services/robot_client.dart';
import '../generated/telemetry.pb.dart';

class StatusScreen extends StatefulWidget {
  final RobotClient client;

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
        setState(() {
          _latestFastState = state;
          _updateCount++;
          _lastUpdateTime = DateTime.now();
          _connectionStatus = 'Connected (${_updateCount} updates)';
        });
      },
      onError: (error) {
        setState(() {
          _connectionStatus = 'Error: $error';
        });
      },
      onDone: () {
        setState(() {
          _connectionStatus = 'Stream closed';
        });
      },
    );

    // 订阅慢速状态流（1Hz）
    _slowStateSubscription = widget.client.streamSlowState().listen(
      (state) {
        setState(() {
          _latestSlowState = state;
        });
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
    widget.client.disconnect();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Robot Status Monitor'),
        actions: [
          Padding(
            padding: const EdgeInsets.all(8.0),
            child: Center(
              child: Chip(
                avatar: Icon(
                  _latestFastState != null ? Icons.check_circle : Icons.error,
                  size: 16,
                  color: _latestFastState != null ? Colors.green : Colors.red,
                ),
                label: Text(_connectionStatus),
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
                padding: const EdgeInsets.all(16),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    _buildPoseCard(),
                    const SizedBox(height: 16),
                    _buildVelocityCard(),
                    const SizedBox(height: 16),
                    _buildOrientationCard(),
                    const SizedBox(height: 16),
                    if (_latestSlowState != null) _buildTopicRatesCard(),
                    if (_latestSlowState != null) const SizedBox(height: 16),
                    if (_latestSlowState != null) _buildSystemResourceCard(),
                  ],
                ),
              ),
            ),
    );
  }

  Widget _buildPoseCard() {
    final pose = _latestFastState!.pose;
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                const Icon(Icons.place, color: Colors.blue),
                const SizedBox(width: 8),
                const Text(
                  'Position (odom frame)',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            const Divider(),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: [
                _buildMetric('X', pose.position.x.toStringAsFixed(3), 'm'),
                _buildMetric('Y', pose.position.y.toStringAsFixed(3), 'm'),
                _buildMetric('Z', pose.position.z.toStringAsFixed(3), 'm'),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildVelocityCard() {
    final vel = _latestFastState!.velocity;
    final linearSpeed = vel.linear.x; // 主要线速度
    final angularSpeed = vel.angular.z; // 主要角速度

    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                const Icon(Icons.speed, color: Colors.green),
                const SizedBox(width: 8),
                const Text(
                  'Velocity (body frame)',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            const Divider(),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: [
                _buildMetric('Linear', linearSpeed.toStringAsFixed(3), 'm/s'),
                _buildMetric('Angular', angularSpeed.toStringAsFixed(3), 'rad/s'),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildOrientationCard() {
    final rpy = _latestFastState!.rpyDeg;
    final tfOk = _latestFastState!.tfOk;

    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                const Icon(Icons.explore, color: Colors.orange),
                const SizedBox(width: 8),
                const Text(
                  'Orientation (RPY)',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
                const Spacer(),
                Chip(
                  label: Text(tfOk ? 'TF OK' : 'TF Error'),
                  backgroundColor: tfOk ? Colors.green.shade100 : Colors.red.shade100,
                ),
              ],
            ),
            const Divider(),
            Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: [
                _buildMetric('Roll', rpy.x.toStringAsFixed(1), '°'),
                _buildMetric('Pitch', rpy.y.toStringAsFixed(1), '°'),
                _buildMetric('Yaw', rpy.z.toStringAsFixed(1), '°'),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildTopicRatesCard() {
    final rates = _latestSlowState!.topicRates;

    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                const Icon(Icons.bar_chart, color: Colors.purple),
                const SizedBox(width: 8),
                const Text(
                  'Topic Rates',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            const Divider(),
            _buildRateRow('Odometry', rates.odomHz),
            _buildRateRow('Terrain Map', rates.terrainMapHz),
            _buildRateRow('Path', rates.pathHz),
            _buildRateRow('LiDAR', rates.lidarHz),
          ],
        ),
      ),
    );
  }

  Widget _buildSystemResourceCard() {
    final resources = _latestSlowState!.resources;

    return Card(
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                const Icon(Icons.memory, color: Colors.red),
                const SizedBox(width: 8),
                const Text(
                  'System Resources',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ],
            ),
            const Divider(),
            _buildProgressRow('CPU', resources.cpuPercent, '%'),
            _buildProgressRow('Memory', resources.memPercent, '%'),
            _buildRateRow('CPU Temp', resources.cpuTemp, unit: '°C'),
            _buildRateRow('Battery', resources.batteryPercent, unit: '%'),
          ],
        ),
      ),
    );
  }

  Widget _buildMetric(String label, String value, String unit) {
    return Column(
      children: [
        Text(
          label,
          style: TextStyle(
            fontSize: 12,
            color: Colors.grey[600],
          ),
        ),
        const SizedBox(height: 4),
        Row(
          mainAxisSize: MainAxisSize.min,
          crossAxisAlignment: CrossAxisAlignment.baseline,
          textBaseline: TextBaseline.alphabetic,
          children: [
            Text(
              value,
              style: const TextStyle(
                fontSize: 24,
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(width: 4),
            Text(
              unit,
              style: TextStyle(
                fontSize: 14,
                color: Colors.grey[600],
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _buildRateRow(String label, double value, {String unit = 'Hz'}) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Text(label),
          Text(
            '${value.toStringAsFixed(2)} $unit',
            style: const TextStyle(fontWeight: FontWeight.bold),
          ),
        ],
      ),
    );
  }

  Widget _buildProgressRow(String label, double value, String unit) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Text(label),
              Text(
                '${value.toStringAsFixed(1)}$unit',
                style: const TextStyle(fontWeight: FontWeight.bold),
              ),
            ],
          ),
          const SizedBox(height: 4),
          LinearProgressIndicator(
            value: value / 100,
            backgroundColor: Colors.grey[300],
            valueColor: AlwaysStoppedAnimation<Color>(
              value > 80 ? Colors.red : (value > 60 ? Colors.orange : Colors.green),
            ),
          ),
        ],
      ),
    );
  }
}
