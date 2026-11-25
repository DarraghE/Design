import 'package:bike_app/app.dart';
import 'package:bike_app/closepass/incident.dart';
import 'package:bike_app/link/close_pass_package.dart';
import 'package:bike_app/link/demo_scene.dart';
import 'package:bike_app/services/place_search.dart';
import 'package:bike_app/ui/home_screen.dart';
import 'package:bike_app/ui/incident_screen.dart';
import 'package:bike_app/ui/navigation_screen.dart';
import 'package:bike_app/ui/recordings_screen.dart';
import 'package:bike_app/ui/settings_screen.dart';
import 'package:bike_app/ui/theme.dart';
import 'package:bike_app/ui/widgets/lcd_preview.dart';
import 'package:bike_app/ui/widgets/lidar_replay.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:material_ui/material_ui.dart';

import 'helpers.dart';

/// Pixel 9a screen: 1080 x 2424 px at 2.625 = about 411 x 923 dp.
void pixel9a(WidgetTester tester) {
  tester.view.physicalSize = const Size(1080, 2424);
  tester.view.devicePixelRatio = 2.625;
  addTearDown(tester.view.reset);
}

Widget host(AppServices app, Widget child) => AppScope(
      services: app,
      child: MaterialApp(theme: buildTheme(), home: child),
    );

void main() {
  HomeScreen.askPermissions = false;

  testWidgets('home: not connected, then demo module connected', (tester) async {
    pixel9a(tester);
    final app = await tester.runAsync(() => testServices());
    await tester.pumpWidget(host(app!, const HomeScreen()));
    expect(find.text('Main Module not connected'), findsOneWidget);
    expect(find.text('Navigation'), findsOneWidget);
    expect(find.text('Close Pass'), findsOneWidget);
    expect(find.byType(LcdPreview), findsOneWidget);
    expect(find.text('WAITING'), findsOneWidget);

    await tester.runAsync(() => app.connectDemo(photos: fakePhotos));
    await tester.pump();
    expect(find.text('Demo module connected'), findsOneWidget);
    expect(find.text('ARMED'), findsOneWidget);
    expect(find.text('Simulate pass'), findsOneWidget);
    await tester.runAsync(() => app.disconnect());
    await tester.pumpWidget(const SizedBox());
    app.display.dispose();
  });

  testWidgets('navigation: plan, start a simulated ride, end it', (tester) async {
    pixel9a(tester);
    final app = await tester.runAsync(() => testServices());
    await tester.pumpWidget(host(app!, const NavigationScreen(mapTiles: false)));
    expect(find.text('Where to?'), findsOneWidget);
    final route = dublinRoute();
    await tester.runAsync(() => app.nav.choose(Place("St Patrick's Cathedral", route.line.points.last), route.line.points.first));
    await tester.pump();
    expect(find.text('Start navigation'), findsOneWidget);
    expect(find.text('1.7 km'), findsOneWidget);

    await tester.runAsync(() async {
      await app.nav.start(route.line.points.first, simulate: true);
      await until(() => app.nav.state != null);
    });
    await tester.pump();
    expect(find.text('End navigation'), findsOneWidget);
    expect(find.text('Simulated'), findsOneWidget);
    expect(find.textContaining('preview only'), findsOneWidget);
    await tester.tap(find.text('End navigation'));
    await tester.runAsync(() => Future<void>.delayed(const Duration(milliseconds: 20)));
    await tester.pump();
    expect(app.nav.active, isFalse);
    expect(find.text('Start navigation'), findsOneWidget);
    await tester.pumpWidget(const SizedBox());
    app.display.dispose();
  });

  testWidgets('recordings and the incident screen with the LiDAR replay', (tester) async {
    pixel9a(tester);
    final app = await tester.runAsync(() => testServices());
    await tester.pumpWidget(host(app!, const RecordingsScreen()));
    expect(find.text('No close passes yet'), findsOneWidget);

    final incident = await tester.runAsync(() async {
      final i = await app.store.add(
        ClosePassPackage(eventId: 1, minDistanceM: 0.82, speedKmh: 47, photos: [PackagePhoto(300, tinyPng), PackagePhoto(600, tinyPng)], clip: const DemoScene().clip()),
        time: DateTime(2026, 11, 25, 14, 30, 12),
        simulated: true,
      );
      i.plate = confidentPlate;
      i.plateStatus = PlateStatus.found;
      await app.store.save(i);
      return i;
    });
    await tester.pump();
    expect(find.text('262-D-12345'), findsOneWidget);
    expect(find.text('0.82 m  ·  47 km/h'), findsOneWidget);

    await tester.pumpWidget(host(app, IncidentScreen(incidentId: incident!.id)));
    // The screen reads the clip and photos from disk in several steps.
    for (var n = 0; n < 40 && find.byType(LidarReplay).evaluate().isEmpty; n++) {
      await tester.runAsync(() => Future<void>.delayed(const Duration(milliseconds: 20)));
      await tester.pump();
    }
    expect(find.byType(LidarReplay), findsOneWidget);
    expect(find.text('0.82 m'), findsOneWidget);
    expect(find.text('47 km/h'), findsOneWidget);
    expect(find.text('262-D-12345'), findsOneWidget);
    expect(find.textContaining('93% confident'), findsOneWidget);
    await tester.dragUntilVisible(find.text('PHOTOS (2)'), find.byType(ListView).first, const Offset(0, -250));
    expect(find.text('PHOTOS (2)'), findsOneWidget);
    await tester.dragUntilVisible(find.text('Upload to website'), find.byType(ListView).first, const Offset(0, -250));
    expect(find.text('Upload to website'), findsOneWidget);
    // The replay animates.
    await tester.pump(const Duration(milliseconds: 800));
    await tester.pump(const Duration(milliseconds: 800));
    await tester.pumpWidget(const SizedBox());
    app.display.dispose();
  });

  testWidgets('settings screen shows the setup fields', (tester) async {
    pixel9a(tester);
    final app = await tester.runAsync(() => testServices());
    await tester.pumpWidget(host(app!, const SettingsScreen()));
    expect(find.text('OpenRouteService API key'), findsOneWidget);
    expect(find.text('OpenAI API key'), findsOneWidget);
    expect(find.text('Upload address'), findsOneWidget);
    await tester.pumpWidget(const SizedBox());
    app.display.dispose();
  });
}
