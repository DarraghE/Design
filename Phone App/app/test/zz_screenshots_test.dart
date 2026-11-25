// Renders the main screens at Pixel 9a size with the real Roboto font, for
// the docs and the demo. Skipped in normal test runs. To regenerate:
//   SCREENSHOTS=1 flutter test test/zz_screenshots_test.dart --update-goldens
// Images are written to Phone App/docs/screenshots/.
import 'dart:io';

import 'package:bike_app/app.dart';
import 'package:bike_app/closepass/incident.dart';
import 'package:bike_app/link/close_pass_package.dart';
import 'package:bike_app/link/demo_scene.dart';
import 'package:bike_app/services/place_search.dart';
import 'package:bike_app/ui/demo_photos.dart';
import 'package:bike_app/ui/home_screen.dart';
import 'package:bike_app/ui/incident_screen.dart';
import 'package:bike_app/ui/navigation_screen.dart';
import 'package:bike_app/ui/recordings_screen.dart';
import 'package:bike_app/ui/settings_screen.dart';
import 'package:bike_app/ui/theme.dart';
import 'package:bike_app/ui/widgets/lidar_replay.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:material_ui/material_ui.dart';

import 'helpers.dart';

final _skip = !Platform.environment.containsKey('SCREENSHOTS');

Future<void> _loadFonts() async {
  final dir = '${Platform.environment['FLUTTER_ROOT'] ?? '/d/bikebuild/flutter'}/bin/cache/artifacts/material_fonts';
  final roboto = FontLoader('Roboto');
  for (final w in ['regular', 'medium', 'bold', 'black']) {
    roboto.addFont(Future.value(ByteData.sublistView(File('$dir/roboto-$w.ttf').readAsBytesSync())));
  }
  await roboto.load();
  final icons = FontLoader('MaterialIcons')
    ..addFont(Future.value(ByteData.sublistView(File('$dir/materialicons-regular.otf').readAsBytesSync())));
  await icons.load();
}

void _pixel9a(WidgetTester tester) {
  tester.view.physicalSize = const Size(1080, 2424);
  tester.view.devicePixelRatio = 2.625;
  addTearDown(tester.view.reset);
}

Widget _host(AppServices app, Widget child) => AppScope(
      services: app,
      child: MaterialApp(debugShowCheckedModeBanner: false, theme: buildTheme(), home: child),
    );

Future<void> _shot(WidgetTester tester, String name) =>
    expectLater(find.byType(MaterialApp), matchesGoldenFile('../../docs/screenshots/$name.png'));

void main() {
  HomeScreen.askPermissions = false;
  setUpAll(_loadFonts);

  testWidgets('screenshots', skip: _skip, (tester) async {
    _pixel9a(tester);
    final app = (await tester.runAsync(() => testServices()))!;
    final route = dublinRoute();

    // Recorded close passes with drawn demo photos.
    await tester.runAsync(() async {
      final photos = await drawDemoPhotos(5);
      for (final (gap, speed, minutesAgo, plate) in [(0.82, 47.0, 2, confidentPlate), (1.21, 33.0, 95, null)]) {
        final i = await app.store.add(
          ClosePassPackage(
            eventId: 1,
            minDistanceM: gap,
            speedKmh: speed,
            photos: [for (var n = 0; n < photos.length; n++) PackagePhoto(300 + 300 * n, photos[n])],
            clip: DemoScene(gapM: gap, speedKmh: speed).clip(),
          ),
          time: DateTime.now().subtract(Duration(minutes: minutesAgo)),
          lat: 53.3405,
          lng: -6.2672,
          accuracyM: 5,
          simulated: true,
        );
        if (plate != null) {
          i.plate = plate;
          i.plateStatus = PlateStatus.found;
          i.uploadStatus = UploadStatus.uploaded;
          i.uploadedAt = DateTime.now();
          i.uploadedUrl = 'https://bikenav.example/passes/42';
        } else {
          i.plate = const PlateResult(found: false, plate: '', notes: 'The vehicle had already turned away.');
          i.plateStatus = PlateStatus.none;
        }
        await app.store.save(i);
      }
      await app.store.flush();
    });

    // 1. Home, demo module connected.
    await tester.runAsync(() => app.connectDemo(photos: fakePhotos));
    await tester.pumpWidget(_host(app, const HomeScreen()));
    await tester.pump();
    await _shot(tester, '1_home');

    // 2. Planning a route.
    await tester.runAsync(() => app.nav.choose(Place("St Patrick's Cathedral, Dublin", route.line.points.last), route.line.points.first));
    await tester.pumpWidget(_host(app, const NavigationScreen(mapTiles: false)));
    await tester.pump();
    await _shot(tester, '2_navigation_plan');

    // 3. Riding: wait for a turn to be shown on the handlebar.
    await tester.runAsync(() async {
      await app.nav.start(route.line.points.first, simulate: true);
      await until(() => (app.nav.state?.distanceToNextM ?? 999) < 60 && (app.nav.state?.distanceToNextM ?? 0) > 20 && app.nav.state!.next!.kind.name.contains('ight'));
    });
    await tester.pump();
    await _shot(tester, '3_navigation_ride');
    await tester.runAsync(() => app.nav.stop());

    // 4. Recordings.
    await tester.pumpWidget(_host(app, const RecordingsScreen()));
    await tester.pump();
    for (var n = 0; n < 5; n++) {
      await tester.runAsync(() => Future<void>.delayed(const Duration(milliseconds: 50)));
      await tester.pump();
    }
    await _shot(tester, '4_recordings');

    // 5. Incident with the LiDAR replay at the moment of the pass.
    final id = app.store.incidents.firstWhere((i) => i.plate?.found == true).id;
    await tester.pumpWidget(_host(app, IncidentScreen(incidentId: id)));
    for (var n = 0; n < 40 && find.byType(LidarReplay).evaluate().isEmpty; n++) {
      await tester.runAsync(() => Future<void>.delayed(const Duration(milliseconds: 20)));
      await tester.pump();
    }
    await tester.pump(const Duration(milliseconds: 4300)); // half speed: -2.0 s .. +0.15 s
    await _shot(tester, '5_incident_replay');
    await tester.dragUntilVisible(find.text('Upload again'), find.byType(ListView).first, const Offset(0, -300));
    for (var n = 0; n < 5; n++) {
      await tester.runAsync(() => Future<void>.delayed(const Duration(milliseconds: 50)));
      await tester.pump();
    }
    await _shot(tester, '6_incident_plate');

    // 7. Settings.
    await tester.pumpWidget(_host(app, const SettingsScreen()));
    await tester.pump();
    await _shot(tester, '7_settings');

    await tester.runAsync(() => app.disconnect());
    await tester.pumpWidget(const SizedBox());
    app.display.dispose();
  });
}
