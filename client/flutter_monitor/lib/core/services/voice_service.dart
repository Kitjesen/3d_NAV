import 'dart:async';
import 'package:flutter/foundation.dart';
import 'package:speech_to_text/speech_to_text.dart';
import 'package:speech_to_text/speech_recognition_result.dart';
import 'package:flutter_tts/flutter_tts.dart';
import 'package:flutter_monitor/core/services/app_logger.dart';

/// 语音交互服务：封装 STT (语音→文字) + TTS (文字→语音)。
///
/// 使用方式：
/// - [startListening] 开始录音识别
/// - [stopListening] 停止录音
/// - [speak] 朗读文字
/// - 监听 [onResult] 获取识别结果
class VoiceService extends ChangeNotifier {
  final SpeechToText _stt = SpeechToText();
  final FlutterTts _tts = FlutterTts();

  bool _sttAvailable = false;
  bool _isListening = false;
  bool _isSpeaking = false;
  String _currentText = '';
  String _lastError = '';
  double _confidence = 0.0;

  /// 识别完成回调 (final result)
  void Function(String text)? onFinalResult;

  // ─── Public getters ───

  bool get isAvailable => _sttAvailable;
  bool get isListening => _isListening;
  bool get isSpeaking => _isSpeaking;
  String get currentText => _currentText;
  String get lastError => _lastError;
  double get confidence => _confidence;

  // ─── Initialization ───

  /// 初始化 STT + TTS，应在 app 启动后尽早调用。
  Future<void> initialize() async {
    // ── STT ──
    try {
      _sttAvailable = await _stt.initialize(
        onError: _onSttError,
        onStatus: _onSttStatus,
      );
      if (_sttAvailable) {
        AppLogger.system.info('VoiceService: STT initialized');
      } else {
        AppLogger.system.warning('VoiceService: STT not available on this device');
      }
    } catch (e) {
      AppLogger.system.error('VoiceService: STT init failed: $e');
      _sttAvailable = false;
    }

    // ── TTS ──
    try {
      await _tts.setLanguage('zh-CN');
      await _tts.setSpeechRate(0.5);
      await _tts.setVolume(1.0);
      await _tts.setPitch(1.0);

      _tts.setStartHandler(() {
        _isSpeaking = true;
        notifyListeners();
      });
      _tts.setCompletionHandler(() {
        _isSpeaking = false;
        notifyListeners();
      });
      _tts.setErrorHandler((msg) {
        _isSpeaking = false;
        AppLogger.system.error('VoiceService: TTS error: $msg');
        notifyListeners();
      });

      AppLogger.system.info('VoiceService: TTS initialized (zh-CN)');
    } catch (e) {
      AppLogger.system.error('VoiceService: TTS init failed: $e');
    }

    notifyListeners();
  }

  // ─── STT: 语音识别 ───

  /// 开始监听语音输入。
  Future<void> startListening() async {
    if (!_sttAvailable) {
      _lastError = '语音识别不可用';
      notifyListeners();
      return;
    }
    if (_isListening) return;

    // 如果正在朗读，先停止
    if (_isSpeaking) {
      await _tts.stop();
      _isSpeaking = false;
    }

    _currentText = '';
    _confidence = 0.0;
    _lastError = '';

    try {
      await _stt.listen(
        onResult: _onSttResult,
        localeId: 'zh_CN',
        listenOptions: SpeechListenOptions(
          listenMode: ListenMode.confirmation,
          cancelOnError: true,
          partialResults: true,
        ),
      );
      _isListening = true;
      notifyListeners();
    } catch (e) {
      _lastError = '启动语音识别失败: $e';
      AppLogger.system.error('VoiceService: startListening failed: $e');
      notifyListeners();
    }
  }

  /// 停止监听。
  Future<void> stopListening() async {
    if (!_isListening) return;
    await _stt.stop();
    _isListening = false;
    notifyListeners();
  }

  void _onSttResult(SpeechRecognitionResult result) {
    _currentText = result.recognizedWords;
    _confidence = result.confidence;
    notifyListeners();

    if (result.finalResult && _currentText.isNotEmpty) {
      _isListening = false;
      onFinalResult?.call(_currentText);
      notifyListeners();
    }
  }

  void _onSttError(dynamic error) {
    _isListening = false;
    _lastError = '$error';
    AppLogger.system.warning('VoiceService: STT error: $error');
    notifyListeners();
  }

  void _onSttStatus(String status) {
    if (status == 'done' || status == 'notListening') {
      _isListening = false;
      notifyListeners();
    }
  }

  // ─── TTS: 语音合成 ───

  /// 朗读文字。
  Future<void> speak(String text) async {
    if (text.isEmpty) return;
    // 如果正在识别，先停止
    if (_isListening) {
      await stopListening();
    }
    await _tts.speak(text);
  }

  /// 停止朗读。
  Future<void> stopSpeaking() async {
    await _tts.stop();
    _isSpeaking = false;
    notifyListeners();
  }

  // ─── Lifecycle ───

  @override
  void dispose() {
    _stt.stop();
    _stt.cancel();
    _tts.stop();
    super.dispose();
  }
}
