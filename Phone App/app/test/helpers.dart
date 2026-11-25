import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import 'package:bike_app/app.dart';
import 'package:bike_app/closepass/close_pass_service.dart';
import 'package:bike_app/closepass/incident.dart';
import 'package:bike_app/closepass/incident_store.dart';
import 'package:bike_app/closepass/plate_reader.dart';
import 'package:bike_app/closepass/uploader.dart';
import 'package:bike_app/display/display_controller.dart';
import 'package:bike_app/link/main_module.dart';
import 'package:bike_app/navigation/nav_controller.dart';
import 'package:bike_app/settings.dart';
import 'package:bike_navigation/bike_navigation.dart';

/// A valid 1x1 PNG (so Image widgets can decode it in tests).
final Uint8List tinyPng = base64Decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNkYAAAAAYAAjCB0C8AAAAASUVORK5CYII=');

/// Stand-in photo maker for the demo module.
Future<List<Uint8List>> fakePhotos(int n) async => List.generate(n, (_) => tinyPng);

/// The real Trinity College -> St Patrick's Cathedral route saved from OpenRouteService.
NavRoute dublinRoute() => parseOrsGeoJson(jsonDecode(
        File('../navigation/test/fixtures/dublin_trinity_to_st_patricks.json').readAsStringSync())
    as Map<String, dynamic>);

/// Plate reader that answers without the network.
class FakePlateReader extends PlateReader {
  final PlateResult? result;
  final Object? error;
  int calls = 0;
  List<PlatePhoto>? lastPhotos;
  FakePlateReader({this.result, this.error}) : super(apiKey: 'test', model: 'test-model');

  @override
  Future<PlateResult> read(List<PlatePhoto> photos, {double? distanceM, double? speedKmh, DateTime? time}) async {
    calls++;
    lastPhotos = photos;
    if (error != null) throw error!;
    return result!;
  }
}

class FakeUploader extends IncidentUploader {
  final List<Incident> uploaded = [];
  final Object? error;
  FakeUploader({this.error}) : super(endpoint: Uri.parse('https://example.test/upload'));

  @override
  Future<UploadResult> upload(Incident i, {required List<Uint8List> photos, Uint8List? lidar}) async {
    if (error != null) throw error!;
    uploaded.add(i);
    return const UploadResult(remoteId: '42', url: 'https://example.test/passes/42');
  }
}

const confidentPlate = PlateResult(
  found: true,
  plate: '262-D-12345',
  country: 'IE',
  confidence: 0.93,
  bestImage: 2,
  vehicleType: 'car',
  vehicleColour: 'blue',
  plausibleFormat: true,
);

/// App services with no platform plugins: memory settings, a temp folder,
/// a stub route provider and no Bluetooth.
Future<AppServices> testServices({
  Settings? settings,
  FakePlateReader? reader,
  FakeUploader? uploader,
  NavRoute? route,
  Duration simulationInterval = const Duration(milliseconds: 1),
}) async {
  final s = settings ?? Settings.memory(orsKey: 'ors', openAiKey: 'sk-test', uploadUrl: 'https://example.test/upload');
  final module = MainModule();
  final r = route ?? dublinRoute();
  final nav = NavController(
    routeProvider: () => (a, b) async => r,
    gps: () => const Stream.empty(),
    simulationInterval: simulationInterval,
  );
  final display = DisplayController(module: module, nav: nav);
  final dir = await Directory.systemTemp.createTemp('bikenav_test_');
  final store = IncidentStore(dir);
  await store.load();
  final closePass = ClosePassService(
    module: module,
    store: store,
    settings: s,
    display: display,
    locate: () async => const Where(53.3438, -6.2546, 8),
    makeReader: (_) => reader ?? FakePlateReader(result: confidentPlate),
    makeUploader: (uri, set) => uploader ?? FakeUploader(),
  );
  return AppServices(settings: s, module: module, nav: nav, display: display, store: store, closePass: closePass);
}

/// Waits (in real time) until [condition] holds.
Future<void> until(bool Function() condition, {Duration timeout = const Duration(seconds: 10)}) async {
  final end = DateTime.now().add(timeout);
  while (!condition()) {
    if (DateTime.now().isAfter(end)) throw StateError('Timed out waiting');
    await Future<void>.delayed(const Duration(milliseconds: 5));
  }
}

bool sameFrame(OledFrame a, OledFrame b) {
  for (var y = 0; y < OledFrame.height; y++) {
    for (var x = 0; x < OledFrame.width; x++) {
      if (a.pixel(x, y) != b.pixel(x, y)) return false;
    }
  }
  return true;
}
