import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_monitor/main.dart';

void main() {
  testWidgets('App starts correctly', (WidgetTester tester) async {
    await tester.pumpWidget(const RobotMonitorApp());
    await tester.pump();
    // App should start and render without errors
    expect(find.byType(RobotMonitorApp), findsOneWidget);
  });
}
