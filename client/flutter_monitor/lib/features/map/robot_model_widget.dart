import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:webview_flutter/webview_flutter.dart';
import 'package:robot_proto/src/common.pb.dart';
import 'package:flutter_monitor/app/theme.dart';

/// Displays the real robot 3D model (URDF/STL meshes) in a WebView
/// with full kinematic joint control.
///
/// Joint order (16 joints):
///   0-3  : FR  hip, thigh, calf, foot
///   4-7  : FL  hip, thigh, calf, foot
///   8-11 : RR  hip, thigh, calf, foot
///  12-15 : RL  hip, thigh, calf, foot
class RobotModelWidget extends StatefulWidget {
  final Pose? currentPose;

  /// 16 joint angles in radians. Pass null to keep idle animation.
  final List<double>? jointAngles;

  const RobotModelWidget({
    super.key,
    this.currentPose,
    this.jointAngles,
  });

  @override
  State<RobotModelWidget> createState() => _RobotModelWidgetState();
}

class _RobotModelWidgetState extends State<RobotModelWidget> {
  WebViewController? _controller;
  bool _isLoaded = false;
  DateTime? _lastPoseUpdate;
  DateTime? _lastJointUpdate;
  Timer? _poseThrottle;
  Timer? _jointThrottle;
  HttpServer? _server;
  int? _serverPort;

  @override
  void initState() {
    super.initState();
    _startLocalServer().then((_) => _initializeWebView());
  }

  @override
  void dispose() {
    _poseThrottle?.cancel();
    _jointThrottle?.cancel();
    _server?.close(force: true);
    super.dispose();
  }

  // ==================== Local HTTP server ====================

  Future<void> _startLocalServer() async {
    if (_server != null) return;
    _server = await HttpServer.bind(InternetAddress.loopbackIPv4, 0);
    _serverPort = _server!.port;
    debugPrint('[RobotModel] HTTP server on port $_serverPort');

    _server!.listen((HttpRequest request) async {
      final path =
          request.uri.path == '/' ? '/urdf_viewer_simple.html' : request.uri.path;
      final assetPath = 'assets$path';
      try {
        final data = await rootBundle.load(assetPath);
        request.response.headers.contentType = _contentType(assetPath);
        request.response.add(data.buffer.asUint8List());
      } catch (_) {
        request.response.statusCode = HttpStatus.notFound;
        request.response.write('Not Found');
      } finally {
        await request.response.close();
      }
    });
  }

  static ContentType _contentType(String path) {
    if (path.endsWith('.html')) return ContentType.html;
    if (path.endsWith('.js')) return ContentType('application', 'javascript');
    if (path.endsWith('.stl') || path.endsWith('.STL')) {
      return ContentType('application', 'octet-stream');
    }
    if (path.endsWith('.urdf')) return ContentType('application', 'xml');
    if (path.endsWith('.png')) return ContentType('image', 'png');
    return ContentType('application', 'octet-stream');
  }

  // ==================== WebView ====================

  void _initializeWebView() {
    if (_serverPort == null) return;
    final url = 'http://127.0.0.1:$_serverPort/urdf_viewer_simple.html';
    debugPrint('[RobotModel] WebView URL: $url');

    setState(() {
      _controller = WebViewController()
        ..setJavaScriptMode(JavaScriptMode.unrestricted)
        ..setBackgroundColor(Colors.transparent)
        ..addJavaScriptChannel(
          'FlutterChannel',
          onMessageReceived: (msg) => _handleJS(msg.message),
        )
        ..setNavigationDelegate(
          NavigationDelegate(
            onPageFinished: (_) {
              setState(() => _isLoaded = true);
              Future.delayed(const Duration(milliseconds: 600), () {
                if (widget.currentPose != null) _sendPose();
                if (widget.jointAngles != null) _sendJointAngles();
              });
            },
            onWebResourceError: (e) =>
                debugPrint('[RobotModel] WebView error: ${e.description}'),
          ),
        )
        ..loadRequest(Uri.parse(url));
    });
  }

  void _handleJS(String message) {
    try {
      final data = jsonDecode(message);
      debugPrint('[RobotModel] JS ${data['type']}: ${data['message']}');
    } catch (_) {}
  }

  // ==================== didUpdateWidget ====================

  @override
  void didUpdateWidget(RobotModelWidget oldWidget) {
    super.didUpdateWidget(oldWidget);

    // Pose change → throttled at 2 Hz
    if (widget.currentPose != oldWidget.currentPose &&
        widget.currentPose != null) {
      _schedulePose();
    }

    // Joint angles change → throttled at 5 Hz
    if (widget.jointAngles != oldWidget.jointAngles &&
        widget.jointAngles != null) {
      _scheduleJoints();
    }
  }

  // ==================== Pose (throttled 2 Hz) ====================

  void _schedulePose() {
    final now = DateTime.now();
    if (_lastPoseUpdate != null &&
        now.difference(_lastPoseUpdate!).inMilliseconds < 500) return;
    _poseThrottle?.cancel();
    _poseThrottle = Timer(const Duration(milliseconds: 500), _sendPose);
  }

  void _sendPose() {
    if (_controller == null || !_isLoaded || widget.currentPose == null) return;
    final p = widget.currentPose!.position;
    final o = widget.currentPose!.orientation;

    // ROS (X fwd, Y left, Z up) → Three.js (X, Y up, Z back)
    _controller!.runJavaScript(
      'if(window.updateRobotPose) window.updateRobotPose('
      '${p.x},${p.z},${-p.y},'
      '${o.x},${o.z},${-o.y},${o.w});',
    );
    _lastPoseUpdate = DateTime.now();
  }

  // ==================== Joint angles (throttled 5 Hz) ====================

  void _scheduleJoints() {
    final now = DateTime.now();
    if (_lastJointUpdate != null &&
        now.difference(_lastJointUpdate!).inMilliseconds < 200) return;
    _jointThrottle?.cancel();
    _jointThrottle = Timer(const Duration(milliseconds: 200), _sendJointAngles);
  }

  void _sendJointAngles() {
    if (_controller == null || !_isLoaded || widget.jointAngles == null) return;
    final a = widget.jointAngles!;
    if (a.length < 16) return;

    final csv = a.map((v) => v.toStringAsFixed(4)).join(',');
    _controller!.runJavaScript(
      'if(window.updateJointAngles) window.updateJointAngles([$csv]);',
    );
    _lastJointUpdate = DateTime.now();
  }

  /// Manually set idle animation on/off from parent.
  void setIdleAnimation(bool enable) {
    _controller?.runJavaScript(
      'if(window.enableIdleAnimation) window.enableIdleAnimation(${enable ? 'true' : 'false'});',
    );
  }

  // ==================== Build ====================

  @override
  Widget build(BuildContext context) {
    final isDark = context.isDark;

    if (_controller == null) {
      return Container(
        color: isDark ? AppColors.darkBackground : Colors.grey[100],
        child: Center(
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              CircularProgressIndicator(color: AppColors.primary),
              const SizedBox(height: 14),
              Text(
                '正在启动 3D 查看器…',
                style: TextStyle(
                  color: isDark ? Colors.white60 : Colors.black54,
                  fontSize: 12,
                ),
              ),
            ],
          ),
        ),
      );
    }

    return Stack(
      children: [
        WebViewWidget(controller: _controller!),
        if (!_isLoaded)
          Container(
            color: (isDark ? Colors.black : Colors.white).withOpacity(0.85),
            child: Center(
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  CircularProgressIndicator(color: AppColors.primary),
                  const SizedBox(height: 14),
                  Text(
                    '正在加载机器人模型…',
                    style: TextStyle(
                      color: isDark ? Colors.white60 : Colors.black54,
                      fontSize: 12,
                    ),
                  ),
                ],
              ),
            ),
          ),
      ],
    );
  }
}
