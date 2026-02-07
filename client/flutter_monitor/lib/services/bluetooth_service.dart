import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

/// BLE connection states
enum BleConnectionState {
  disconnected,
  scanning,
  connecting,
  connected,
  error,
}

/// A discovered BLE robot device
class BleRobotDevice {
  final BluetoothDevice device;
  final String name;
  final int rssi;
  final DateTime discoveredAt;

  BleRobotDevice({
    required this.device,
    required this.name,
    required this.rssi,
    DateTime? discoveredAt,
  }) : discoveredAt = discoveredAt ?? DateTime.now();

  String get id => device.remoteId.str;
  String get displayName => name.isNotEmpty ? name : 'Unknown (${id.substring(0, 8)})';

  /// Signal quality: 0.0 to 1.0
  double get signalQuality {
    // RSSI typically ranges from -100 (worst) to -30 (best)
    final clamped = rssi.clamp(-100, -30);
    return (clamped + 100) / 70.0;
  }
}

/// Manages Bluetooth Low Energy connections for robot communication
class BluetoothService extends ChangeNotifier {
  // Robot-specific BLE service UUID (customize for your robot)
  static const String robotServiceUuid = '12345678-1234-5678-1234-56789abcdef0';

  BleConnectionState _state = BleConnectionState.disconnected;
  BleConnectionState get state => _state;

  final List<BleRobotDevice> _discoveredDevices = [];
  List<BleRobotDevice> get discoveredDevices => List.unmodifiable(_discoveredDevices);

  BluetoothDevice? _connectedDevice;
  BluetoothDevice? get connectedDevice => _connectedDevice;

  StreamSubscription? _scanSubscription;
  StreamSubscription? _connectionSubscription;

  String? _errorMessage;
  String? get errorMessage => _errorMessage;

  /// Check if Bluetooth is available and enabled
  Future<bool> get isAvailable async {
    try {
      final supported = await FlutterBluePlus.isSupported;
      if (!supported) return false;
      final state = FlutterBluePlus.adapterStateNow;
      return state == BluetoothAdapterState.on;
    } catch (_) {
      return false;
    }
  }

  /// Start scanning for BLE robot devices
  Future<void> startScan({Duration timeout = const Duration(seconds: 10)}) async {
    if (_state == BleConnectionState.scanning) return;

    _discoveredDevices.clear();
    _state = BleConnectionState.scanning;
    _errorMessage = null;
    notifyListeners();

    try {
      // Check Bluetooth state
      final available = await isAvailable;
      if (!available) {
        _state = BleConnectionState.error;
        _errorMessage = '蓝牙未开启或不可用';
        notifyListeners();
        return;
      }

      await FlutterBluePlus.startScan(
        timeout: timeout,
        androidUsesFineLocation: true,
      );

      _scanSubscription = FlutterBluePlus.scanResults.listen((results) {
        for (final r in results) {
          final name = r.device.platformName;
          // Accept devices with a name (filter empty/unknown)
          if (name.isNotEmpty) {
            final existing = _discoveredDevices.indexWhere(
              (d) => d.id == r.device.remoteId.str,
            );
            final device = BleRobotDevice(
              device: r.device,
              name: name,
              rssi: r.rssi,
            );
            if (existing >= 0) {
              _discoveredDevices[existing] = device;
            } else {
              _discoveredDevices.add(device);
            }
          }
        }
        // Sort by signal strength
        _discoveredDevices.sort((a, b) => b.rssi.compareTo(a.rssi));
        notifyListeners();
      });

      // After timeout, stop
      Future.delayed(timeout, () {
        if (_state == BleConnectionState.scanning) {
          stopScan();
        }
      });
    } catch (e) {
      debugPrint('[BLE] Scan error: $e');
      _state = BleConnectionState.error;
      _errorMessage = '扫描失败: $e';
      notifyListeners();
    }
  }

  /// Stop scanning
  void stopScan() {
    FlutterBluePlus.stopScan();
    _scanSubscription?.cancel();
    _scanSubscription = null;
    if (_state == BleConnectionState.scanning) {
      _state = BleConnectionState.disconnected;
      notifyListeners();
    }
  }

  /// Connect to a BLE device
  Future<bool> connectDevice(BleRobotDevice robot) async {
    _state = BleConnectionState.connecting;
    _errorMessage = null;
    notifyListeners();

    try {
      await robot.device.connect(
        timeout: const Duration(seconds: 10),
        autoConnect: false,
      );

      _connectedDevice = robot.device;
      _state = BleConnectionState.connected;

      // Listen for disconnection
      _connectionSubscription = robot.device.connectionState.listen((state) {
        if (state == BluetoothConnectionState.disconnected) {
          _connectedDevice = null;
          _state = BleConnectionState.disconnected;
          notifyListeners();
        }
      });

      notifyListeners();
      debugPrint('[BLE] Connected to ${robot.displayName}');
      return true;
    } catch (e) {
      debugPrint('[BLE] Connection failed: $e');
      _state = BleConnectionState.error;
      _errorMessage = '连接失败: $e';
      notifyListeners();
      return false;
    }
  }

  /// Disconnect from the current device
  Future<void> disconnectDevice() async {
    try {
      await _connectedDevice?.disconnect();
    } catch (_) {}
    _connectionSubscription?.cancel();
    _connectionSubscription = null;
    _connectedDevice = null;
    _state = BleConnectionState.disconnected;
    notifyListeners();
  }

  @override
  void dispose() {
    stopScan();
    disconnectDevice();
    super.dispose();
  }
}
