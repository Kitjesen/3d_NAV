import 'dart:async';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import 'package:robot_proto/robot_proto.dart';
import 'package:flutter_monitor/app/theme.dart';
import 'package:flutter_monitor/core/services/voice_service.dart';
import 'package:flutter_monitor/core/gateway/task_gateway.dart';
import 'package:flutter_monitor/core/providers/robot_connection_provider.dart';
import 'package:flutter_monitor/core/locale/locale_provider.dart';

/// 语音对话面板 — 按住说话 + 聊天气泡 + 文字输入备用。
///
/// 用法：在 ControlScreen 的 Stack 中叠加此 Widget。
class VoiceChatWidget extends StatefulWidget {
  final VoidCallback? onClose;
  const VoiceChatWidget({super.key, this.onClose});

  @override
  State<VoiceChatWidget> createState() => _VoiceChatWidgetState();
}

class _ChatMessage {
  final String text;
  final bool isUser;
  final DateTime time;
  const _ChatMessage({required this.text, required this.isUser, required this.time});
}

class _VoiceChatWidgetState extends State<VoiceChatWidget>
    with SingleTickerProviderStateMixin {
  final TextEditingController _textController = TextEditingController();
  final List<_ChatMessage> _messages = [];
  final ScrollController _scrollController = ScrollController();
  late final AnimationController _pulseController;
  StreamSubscription? _eventSub;

  @override
  void initState() {
    super.initState();
    _pulseController = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 1200),
    );

    // 绑定语音识别回调
    final voice = context.read<VoiceService>();
    voice.onFinalResult = _onVoiceResult;

    // 监听机器人事件流获取语音回复
    _subscribeRobotResponses();
  }

  void _subscribeRobotResponses() {
    final provider = context.read<RobotConnectionProvider>();
    final client = provider.client;
    if (client == null) return;

    _eventSub?.cancel();
    _eventSub = client.streamEvents().listen((event) {
      // 检查是否有语义导航相关的事件
      if (event.hasDescription() && event.description.isNotEmpty) {
        final isNavEvent =
            event.type == EventType.EVENT_TYPE_TASK_COMPLETED ||
            event.type == EventType.EVENT_TYPE_TASK_FAILED ||
            event.type == EventType.EVENT_TYPE_NAV_GOAL_REACHED ||
            event.type == EventType.EVENT_TYPE_NAV_PATH_BLOCKED;
        if (isNavEvent) {
          _addRobotMessage(event.description);
        }
      }
    }, onError: (_) {});
  }

  @override
  void dispose() {
    _pulseController.dispose();
    _textController.dispose();
    _scrollController.dispose();
    _eventSub?.cancel();
    // 清除回调引用
    try {
      context.read<VoiceService>().onFinalResult = null;
    } catch (_) {}
    super.dispose();
  }

  void _onVoiceResult(String text) {
    if (text.isEmpty) return;
    _sendInstruction(text);
  }

  Future<void> _sendInstruction(String text) async {
    // 添加用户消息
    setState(() {
      _messages.add(_ChatMessage(text: text, isUser: true, time: DateTime.now()));
    });
    _scrollToBottom();

    // 通过 gRPC 发送语义导航指令
    final taskGw = context.read<TaskGateway>();
    final voice = context.read<VoiceService>();
    final ok = await taskGw.startSemanticNav(text);

    if (!mounted) return;

    if (ok) {
      _addRobotMessage('收到指令，正在处理...');
      await voice.speak('收到');
    } else {
      final errorMsg = taskGw.statusMessage ?? '发送失败';
      _addRobotMessage(errorMsg);
    }
  }

  void _addRobotMessage(String text) {
    if (!mounted) return;
    setState(() {
      _messages.add(_ChatMessage(text: text, isUser: false, time: DateTime.now()));
    });
    _scrollToBottom();

    // TTS 播报机器人回复
    final voice = context.read<VoiceService>();
    voice.speak(text);
  }

  void _scrollToBottom() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_scrollController.hasClients) {
        _scrollController.animateTo(
          _scrollController.position.maxScrollExtent,
          duration: const Duration(milliseconds: 200),
          curve: Curves.easeOut,
        );
      }
    });
  }

  void _onTextSubmit() {
    final text = _textController.text.trim();
    if (text.isEmpty) return;
    _textController.clear();
    _sendInstruction(text);
  }

  @override
  Widget build(BuildContext context) {
    final voice = context.watch<VoiceService>();
    final locale = context.read<LocaleProvider>();
    final dark = context.isDark;

    return Container(
      width: 340,
      constraints: const BoxConstraints(maxHeight: 480),
      decoration: BoxDecoration(
        color: dark
            ? Colors.black.withValues(alpha: 0.85)
            : Colors.white.withValues(alpha: 0.92),
        borderRadius: BorderRadius.circular(24),
        border: Border.all(
          color: dark
              ? Colors.white.withValues(alpha: 0.1)
              : Colors.black.withValues(alpha: 0.08),
        ),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withValues(alpha: 0.25),
            blurRadius: 30,
            offset: const Offset(0, 10),
          ),
        ],
      ),
      child: ClipRRect(
        borderRadius: BorderRadius.circular(24),
        child: BackdropFilter(
          filter: ImageFilter.blur(sigmaX: 20, sigmaY: 20),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              // ── Header ──
              _buildHeader(locale, dark),

              // ── Chat messages ──
              Flexible(
                child: _messages.isEmpty
                    ? _buildEmptyState(locale, dark)
                    : ListView.builder(
                        controller: _scrollController,
                        padding: const EdgeInsets.symmetric(
                            horizontal: 12, vertical: 8),
                        itemCount: _messages.length,
                        itemBuilder: (_, i) =>
                            _buildMessageBubble(_messages[i], dark),
                      ),
              ),

              // ── Live transcription ──
              if (voice.isListening && voice.currentText.isNotEmpty)
                Padding(
                  padding:
                      const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
                  child: Text(
                    voice.currentText,
                    style: TextStyle(
                      fontSize: 13,
                      color: AppColors.primary,
                      fontStyle: FontStyle.italic,
                    ),
                    maxLines: 2,
                    overflow: TextOverflow.ellipsis,
                  ),
                ),

              const Divider(height: 1),

              // ── Input area ──
              _buildInputArea(voice, locale, dark),
            ],
          ),
        ),
      ),
    );
  }

  Widget _buildHeader(LocaleProvider locale, bool dark) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
      decoration: BoxDecoration(
        border: Border(
          bottom: BorderSide(
            color: dark
                ? Colors.white.withValues(alpha: 0.08)
                : Colors.black.withValues(alpha: 0.06),
          ),
        ),
      ),
      child: Row(
        children: [
          Icon(Icons.mic_rounded, size: 20, color: AppColors.primary),
          const SizedBox(width: 8),
          Text(
            locale.tr('语音指令', 'Voice Command'),
            style: TextStyle(
              fontSize: 15,
              fontWeight: FontWeight.w600,
              color: dark ? Colors.white : Colors.black87,
            ),
          ),
          const Spacer(),
          GestureDetector(
            onTap: widget.onClose,
            child: Icon(
              Icons.close_rounded,
              size: 20,
              color: dark ? Colors.white54 : Colors.black45,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildEmptyState(LocaleProvider locale, bool dark) {
    return Padding(
      padding: const EdgeInsets.all(24),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Icon(
            Icons.record_voice_over_rounded,
            size: 48,
            color: dark ? Colors.white24 : Colors.black12,
          ),
          const SizedBox(height: 12),
          Text(
            locale.tr(
              '按住麦克风说话\n或输入文字指令',
              'Hold mic to speak\nor type a command',
            ),
            textAlign: TextAlign.center,
            style: TextStyle(
              fontSize: 13,
              color: dark ? Colors.white38 : Colors.black38,
              height: 1.5,
            ),
          ),
          const SizedBox(height: 12),
          Wrap(
            spacing: 6,
            runSpacing: 6,
            alignment: WrapAlignment.center,
            children: [
              _buildQuickChip('导航到大门', dark),
              _buildQuickChip('停下来', dark),
              _buildQuickChip('巡逻', dark),
              _buildQuickChip('当前状态', dark),
            ],
          ),
        ],
      ),
    );
  }

  Widget _buildQuickChip(String text, bool dark) {
    return GestureDetector(
      onTap: () => _sendInstruction(text),
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
        decoration: BoxDecoration(
          color: dark
              ? Colors.white.withValues(alpha: 0.08)
              : AppColors.primary.withValues(alpha: 0.08),
          borderRadius: BorderRadius.circular(12),
          border: Border.all(
            color: dark
                ? Colors.white.withValues(alpha: 0.12)
                : AppColors.primary.withValues(alpha: 0.15),
          ),
        ),
        child: Text(
          text,
          style: TextStyle(
            fontSize: 12,
            color: dark ? Colors.white70 : AppColors.primary,
          ),
        ),
      ),
    );
  }

  Widget _buildMessageBubble(_ChatMessage msg, bool dark) {
    final isUser = msg.isUser;
    return Align(
      alignment: isUser ? Alignment.centerRight : Alignment.centerLeft,
      child: Container(
        margin: const EdgeInsets.symmetric(vertical: 3),
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
        constraints: const BoxConstraints(maxWidth: 260),
        decoration: BoxDecoration(
          color: isUser
              ? AppColors.primary.withValues(alpha: 0.15)
              : (dark
                  ? Colors.white.withValues(alpha: 0.08)
                  : Colors.grey.withValues(alpha: 0.1)),
          borderRadius: BorderRadius.only(
            topLeft: const Radius.circular(14),
            topRight: const Radius.circular(14),
            bottomLeft: Radius.circular(isUser ? 14 : 4),
            bottomRight: Radius.circular(isUser ? 4 : 14),
          ),
        ),
        child: Text(
          msg.text,
          style: TextStyle(
            fontSize: 13,
            color: dark ? Colors.white : Colors.black87,
            height: 1.4,
          ),
        ),
      ),
    );
  }

  Widget _buildInputArea(VoiceService voice, LocaleProvider locale, bool dark) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(8, 6, 8, 8),
      child: Row(
        children: [
          // ── Text input ──
          Expanded(
            child: Container(
              height: 40,
              decoration: BoxDecoration(
                color: dark
                    ? Colors.white.withValues(alpha: 0.08)
                    : Colors.grey.withValues(alpha: 0.1),
                borderRadius: BorderRadius.circular(20),
              ),
              child: TextField(
                controller: _textController,
                style: TextStyle(
                  fontSize: 13,
                  color: dark ? Colors.white : Colors.black87,
                ),
                decoration: InputDecoration(
                  hintText: locale.tr('输入指令...', 'Type command...'),
                  hintStyle: TextStyle(
                    fontSize: 13,
                    color: dark ? Colors.white30 : Colors.black26,
                  ),
                  border: InputBorder.none,
                  contentPadding: const EdgeInsets.symmetric(
                      horizontal: 16, vertical: 10),
                ),
                textInputAction: TextInputAction.send,
                onSubmitted: (_) => _onTextSubmit(),
              ),
            ),
          ),
          const SizedBox(width: 6),

          // ── Send button (text) ──
          if (_textController.text.isNotEmpty)
            GestureDetector(
              onTap: _onTextSubmit,
              child: Container(
                width: 40,
                height: 40,
                decoration: BoxDecoration(
                  color: AppColors.primary,
                  shape: BoxShape.circle,
                ),
                child:
                    const Icon(Icons.send_rounded, size: 18, color: Colors.white),
              ),
            )
          else
            // ── Mic button (push-to-talk) ──
            GestureDetector(
              onLongPressStart: (_) {
                HapticFeedback.mediumImpact();
                _pulseController.repeat(reverse: true);
                voice.startListening();
              },
              onLongPressEnd: (_) {
                _pulseController.stop();
                _pulseController.value = 0;
                voice.stopListening();
              },
              onTap: () {
                // 短按切换
                HapticFeedback.lightImpact();
                if (voice.isListening) {
                  voice.stopListening();
                } else {
                  voice.startListening();
                }
              },
              child: AnimatedBuilder(
                animation: _pulseController,
                builder: (_, child) {
                  final scale = 1.0 + _pulseController.value * 0.15;
                  return Transform.scale(scale: scale, child: child);
                },
                child: Container(
                  width: 40,
                  height: 40,
                  decoration: BoxDecoration(
                    color: voice.isListening
                        ? Colors.red
                        : AppColors.primary,
                    shape: BoxShape.circle,
                    boxShadow: voice.isListening
                        ? [
                            BoxShadow(
                              color: Colors.red.withValues(alpha: 0.4),
                              blurRadius: 12,
                            ),
                          ]
                        : null,
                  ),
                  child: Icon(
                    voice.isListening
                        ? Icons.stop_rounded
                        : Icons.mic_rounded,
                    size: 20,
                    color: Colors.white,
                  ),
                ),
              ),
            ),
        ],
      ),
    );
  }
}
