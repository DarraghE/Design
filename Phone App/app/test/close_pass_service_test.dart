import 'dart:io';

import 'package:bike_app/app.dart';
import 'package:bike_app/closepass/incident.dart';
import 'package:bike_app/closepass/incident_store.dart';
import 'package:bike_app/closepass/plate_reader.dart';
import 'package:bike_app/display/display_controller.dart';
import 'package:bike_app/link/close_pass_package.dart';
import 'package:bike_app/link/demo_main_module.dart';
import 'package:bike_app/link/demo_scene.dart';
import 'package:bike_app/services/place_search.dart';
import 'package:bike_app/settings.dart';
import 'package:bike_navigation/bike_navigation.dart';
import 'package:flutter_test/flutter_test.dart';

import 'helpers.dart';

Future<DemoMainModule> connectDemo(AppServices app) async {
  final demo = DemoMainModule(photoMaker: fakePhotos, notifyGap: Duration.zero, photoDelay: const Duration(milliseconds: 20));
  await app.module.attach(demo);
  return demo;
}

void main() {
  test('full close pass: handlebar alert, saved incident, plate read, uploaded', () async {
    final reader = FakePlateReader(result: confidentPlate);
    final uploader = FakeUploader();
    final app = await testServices(reader: reader, uploader: uploader);
    final demo = await connectDemo(app);

    await demo.triggerClosePass(scene: const DemoScene(gapM: 0.7, speedKmh: 52), photos: 5);
    // The handlebar showed the close-pass screen straight away.
    expect(app.display.mode, DisplayMode.closePass);
    await app.module.flush();
    expect(sameFrame(demo.lcd.value!.frame, renderCloseAlert(distanceM: 0.7, speedKmh: 52)), isTrue);

    await until(() => app.store.incidents.isNotEmpty && app.store.incidents.first.uploadStatus == UploadStatus.uploaded);
    final i = app.store.incidents.single;
    expect(i.simulated, isTrue);
    expect(i.distanceM, closeTo(0.7, 0.001));
    expect(i.speedKmh, 52);
    expect(i.lat, 53.3438);
    expect(i.photoFiles, ['photo_1.png', 'photo_2.png', 'photo_3.png', 'photo_4.png', 'photo_5.png']);
    expect(i.plateStatus, PlateStatus.found);
    expect(i.plate!.plate, '262-D-12345');
    expect(i.bestPhotoIndex, 1);
    expect(i.uploadedUrl, 'https://example.test/passes/42');
    expect(reader.calls, 1);
    expect(reader.lastPhotos!.length, 5);
    expect(reader.lastPhotos!.first.mimeType, 'image/png');
    expect(uploader.uploaded.single.id, i.id);

    // Everything is on disk and comes back after a restart.
    await app.closePass.settle();
    final again = IncidentStore(app.store.root);
    await again.load();
    final saved = again.incidents.single;
    expect(saved.plate!.plate, '262-D-12345');
    expect(saved.uploadStatus, UploadStatus.uploaded);
    final clip = await again.clip(saved);
    expect(clip!.scans.length, const DemoScene().clip().scans.length);
    expect((await again.photos(saved)).length, 5);
  });

  test('an unsure plate is kept but not uploaded automatically; upload by hand works', () async {
    final uploader = FakeUploader();
    final app = await testServices(
      reader: FakePlateReader(result: const PlateResult(found: true, plate: '25?-D-1234', confidence: 0.4)),
      uploader: uploader,
    );
    final demo = await connectDemo(app);
    await demo.triggerClosePass();
    await until(() => app.store.incidents.isNotEmpty && app.store.incidents.first.plateStatus == PlateStatus.uncertain);
    final i = app.store.incidents.single;
    expect(i.uploadStatus, UploadStatus.notNeeded);
    expect(uploader.uploaded, isEmpty);
    await app.closePass.upload(i);
    expect(i.uploadStatus, UploadStatus.uploaded);
  });

  test('no plate found', () async {
    final app = await testServices(reader: FakePlateReader(result: const PlateResult(found: false, plate: '', notes: 'No vehicle visible')));
    final demo = await connectDemo(app);
    await demo.triggerClosePass();
    await until(() => app.store.incidents.isNotEmpty && app.store.incidents.first.plateStatus == PlateStatus.none);
    expect(app.store.incidents.single.uploadStatus, UploadStatus.notNeeded);
  });

  test('without keys or website it waits, then finishes once they are added', () async {
    final settings = Settings.memory();
    final uploader = FakeUploader();
    final app = await testServices(settings: settings, uploader: uploader);
    final demo = await connectDemo(app);
    await demo.triggerClosePass();
    await until(() => app.store.incidents.isNotEmpty && app.store.incidents.first.plateStatus == PlateStatus.noKey);
    expect(app.store.incidents.single.plateError, contains('Settings'));

    await settings.update(openAiKey: 'sk-new');
    await app.closePass.resumePending();
    final i = app.store.incidents.single;
    expect(i.plateStatus, PlateStatus.found);
    expect(i.uploadStatus, UploadStatus.notConfigured);

    await settings.update(uploadUrl: 'https://example.test/upload');
    await app.closePass.resumePending();
    expect(i.uploadStatus, UploadStatus.uploaded);
    expect(uploader.uploaded.length, 1);
  });

  test('errors are shown, not lost', () async {
    final app = await testServices(
      reader: FakePlateReader(error: const PlateReaderException('OpenAI rejected the API key. Check it in Settings.')),
    );
    final demo = await connectDemo(app);
    await demo.triggerClosePass();
    await until(() => app.store.incidents.isNotEmpty && app.store.incidents.first.plateStatus == PlateStatus.failed);
    expect(app.store.incidents.single.plateError, contains('rejected'));

    final app2 = await testServices(uploader: FakeUploader(error: Exception('Website replied 500')));
    final demo2 = await connectDemo(app2);
    await demo2.triggerClosePass();
    await until(() => app2.store.incidents.isNotEmpty && app2.store.incidents.first.uploadStatus == UploadStatus.failed);
    expect(app2.store.incidents.single.uploadError, contains('500'));
  });

  test('while navigating, a close pass is recorded but the handlebar keeps directions', () async {
    final app = await testServices();
    final demo = await connectDemo(app);
    final route = dublinRoute();
    await app.nav.choose(Place('Destination', route.line.points.last), route.line.points.first);
    await app.nav.start(route.line.points.first, simulate: true);
    await until(() => app.nav.state != null);
    await demo.triggerClosePass();
    expect(app.display.mode, DisplayMode.navigation);
    await until(() => app.store.incidents.isNotEmpty);
    await app.nav.stop();
  });

  test('store: unique ids, skips broken folders, deletes', () async {
    final dir = await Directory.systemTemp.createTemp('bikenav_store_');
    await Directory('${dir.path}/broken').create();
    await File('${dir.path}/broken/incident.json').writeAsString('{not json');
    final store = IncidentStore(dir);
    await store.load();
    expect(store.incidents, isEmpty);
    final pkg = ClosePassPackage(eventId: 1, photos: [PackagePhoto(0, tinyPng)], clip: const DemoScene().clip());
    final t = DateTime(2026, 11, 25, 14, 30, 12);
    final a = await store.add(pkg, time: t);
    final b = await store.add(pkg, time: t);
    expect(a.id, '20261125-143012');
    expect(b.id, '20261125-143012-2');
    expect(store.photoFile(a, 0).existsSync(), isTrue);
    await store.delete(a);
    expect(store.incidents.map((i) => i.id), [b.id]);
    expect(Directory('${dir.path}/${a.id}').existsSync(), isFalse);
  });
}
