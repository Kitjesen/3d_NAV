import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:intl/intl.dart';
import '../services/robot_client_base.dart';
import '../generated/telemetry.pb.dart';
import '../generated/common.pb.dart';
import '../widgets/glass_widgets.dart';

class EventsScreen extends StatefulWidget {
  final RobotClientBase client;

  const EventsScreen({super.key, required this.client});

  @override
  State<EventsScreen> createState() => _EventsScreenState();
}

class _EventsScreenState extends State<EventsScreen> {
  final List<Event> _events = [];
  StreamSubscription? _subscription;
  bool _isConnected = false;

  @override
  void initState() {
    super.initState();
    _startListening();
  }

  @override
  void dispose() {
    _subscription?.cancel();
    super.dispose();
  }

  void _startListening() {
    try {
      final lastId = _events.isNotEmpty ? _events.last.eventId : '';
      
      _subscription = widget.client.streamEvents(lastEventId: lastId).listen((event) {
        if (!mounted) return;
        setState(() {
          if (!_events.any((e) => e.eventId == event.eventId)) {
             _events.insert(0, event); // Newest on top
          }
          _isConnected = true;
        });
      }, onError: (e) {
        print('Events stream error: $e');
        if (mounted) {
          setState(() {
            _isConnected = false;
          });
          Future.delayed(const Duration(seconds: 3), _startListening);
        }
      });
    } catch (e) {
      print('Failed to start event stream: $e');
    }
  }

  Color _getSeverityColor(EventSeverity severity) {
    switch (severity) {
      case EventSeverity.EVENT_SEVERITY_DEBUG: return Colors.grey;
      case EventSeverity.EVENT_SEVERITY_INFO: return const Color(0xFF007AFF); // iOS Blue
      case EventSeverity.EVENT_SEVERITY_WARNING: return const Color(0xFFFF9500); // iOS Orange
      case EventSeverity.EVENT_SEVERITY_ERROR: return const Color(0xFFFF3B30); // iOS Red
      case EventSeverity.EVENT_SEVERITY_CRITICAL: return const Color(0xFFAF52DE); // iOS Purple
      default: return Colors.black;
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      extendBodyBehindAppBar: true,
      appBar: AppBar(
        title: const Text('Timeline'),
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh),
            onPressed: () {
               _subscription?.cancel();
               setState(() {
                 _events.clear();
               });
               _startListening();
            },
          )
        ],
      ),
      body: _events.isEmpty
          ? Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Icon(Icons.history, size: 48, color: Colors.grey.withOpacity(0.3)),
                  const SizedBox(height: 16),
                  Text('No events recorded', style: TextStyle(color: Colors.grey.withOpacity(0.5))),
                  if (!_isConnected)
                    const Padding(
                      padding: EdgeInsets.only(top: 8.0),
                      child: Text('Connecting...', style: TextStyle(color: Colors.grey, fontSize: 12)),
                    ),
                ],
              ),
            )
          : Stack(
              children: [
                // Vertical Timeline Line
                Positioned(
                  left: 24,
                  top: 0,
                  bottom: 0,
                  child: Container(
                    width: 2,
                    color: Colors.grey.withOpacity(0.2),
                  ),
                ),
                // Event List
                ListView.builder(
                  padding: EdgeInsets.fromLTRB(16, MediaQuery.of(context).padding.top + 60, 16, 20),
                  itemCount: _events.length,
                  itemBuilder: (context, index) {
                    final event = _events[index];
                    final time = event.timestamp.toDateTime().toLocal();
                    final timeStr = DateFormat('HH:mm:ss').format(time);
                    final color = _getSeverityColor(event.severity);

                    return Padding(
                      padding: const EdgeInsets.only(bottom: 24.0),
                      child: Row(
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          // Timeline Dot
                          Container(
                            margin: const EdgeInsets.only(top: 6),
                            width: 14,
                            height: 14,
                            decoration: BoxDecoration(
                              color: color,
                              shape: BoxShape.circle,
                              border: Border.all(color: Colors.white, width: 2.5),
                              boxShadow: [
                                BoxShadow(
                                  color: color.withOpacity(0.4),
                                  blurRadius: 8,
                                  offset: const Offset(0, 2),
                                ),
                              ],
                            ),
                          ),
                          const SizedBox(width: 16),
                          // Event Card
                          Expanded(
                            child: GlassCard(
                              padding: const EdgeInsets.all(16),
                              borderRadius: 16,
                              child: Column(
                                crossAxisAlignment: CrossAxisAlignment.start,
                                children: [
                                  Row(
                                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                                    children: [
                                      Expanded(
                                        child: Text(
                                          event.title.isNotEmpty ? event.title : event.type.toString(),
                                          style: const TextStyle(
                                            fontSize: 15,
                                            fontWeight: FontWeight.bold,
                                            color: Colors.black87,
                                          ),
                                          overflow: TextOverflow.ellipsis,
                                        ),
                                      ),
                                      Text(
                                        timeStr,
                                        style: TextStyle(
                                          fontSize: 12,
                                          color: Colors.black.withOpacity(0.4),
                                          fontFeatures: const [FontFeature.tabularFigures()],
                                        ),
                                      ),
                                    ],
                                  ),
                                  const SizedBox(height: 8),
                                  Text(
                                    event.description,
                                    style: TextStyle(
                                      fontSize: 14,
                                      color: Colors.black.withOpacity(0.7),
                                      height: 1.4,
                                    ),
                                  ),
                                  const SizedBox(height: 12),
                                  Align(
                                    alignment: Alignment.centerRight,
                                    child: InkWell(
                                      onTap: () async {
                                        await widget.client.ackEvent(event.eventId);
                                        if (mounted) {
                                          ScaffoldMessenger.of(context).showSnackBar(
                                            const SnackBar(
                                              content: Text('Event acknowledged'),
                                              duration: Duration(milliseconds: 500),
                                              behavior: SnackBarBehavior.floating,
                                            ),
                                          );
                                        }
                                      },
                                      child: Container(
                                        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                                        decoration: BoxDecoration(
                                          color: Colors.black.withOpacity(0.05),
                                          borderRadius: BorderRadius.circular(20),
                                        ),
                                        child: Row(
                                          mainAxisSize: MainAxisSize.min,
                                          children: [
                                            Icon(Icons.check, size: 14, color: Colors.black.withOpacity(0.6)),
                                            const SizedBox(width: 4),
                                            Text(
                                              'ACKNOWLEDGE',
                                              style: TextStyle(
                                                fontSize: 10,
                                                fontWeight: FontWeight.w600,
                                                color: Colors.black.withOpacity(0.6),
                                              ),
                                            ),
                                          ],
                                        ),
                                      ),
                                    ),
                                  )
                                ],
                              ),
                            ),
                          ),
                        ],
                      ),
                    );
                  },
                ),
              ],
            ),
    );
  }
}
