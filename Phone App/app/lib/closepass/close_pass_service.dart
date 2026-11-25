import 'dart:async';

import 'package:flutter/foundation.dart';

import '../display/display_controller.dart';
import '../link/close_pass_package.dart';
import '../link/main_module.dart';
import '../link/protocol.dart';
import '../settings.dart';
import 'clip_analysis.dart';
import 'incident.dart';
import 'incident_store.dart';
import 'plate_reader.dart';
import 'uploader.dart';

/// Where the phone was, for the incident record.
class Where {
  final double lat, lng, accuracyM;
  const Where(this.lat, this.lng, this.accuracyM);
}

typedef Locator = Future<Where?> Function();
typedef PlateReaderFactory = PlateReader Function(Settings s);
typedef UploaderFactory = IncidentUploader Function(Uri endpoint, Settings s);

/// Close Pass mode. It is always on while the Main Module is connected:
///
/// 1. Alert from the Main Module → the handlebar shows "CLOSE PASS" (unless
///    navigating) and the phone notes the time and its GPS position.
/// 2. Package (photos + LiDAR clip) → saved as an incident.
/// 3. The photos go to OpenAI to read the overtaking vehicle's plate.
/// 4. If a plate was read confidently and a website is set up, the plate,
///    best photo, all photos, LiDAR clip and details are uploaded.
class ClosePassService extends ChangeNotifier {
  final MainModule module;
  final IncidentStore store;
  final Settings settings;
  final DisplayController display;
  final Locator locate;
  final PlateReaderFactory makeReader;
  final UploaderFactory makeUploader;

  ClosePassService({
    required this.module,
    required this.store,
    required this.settings,
    required this.display,
    required this.locate,
    PlateReaderFactory? makeReader,
    UploaderFactory? makeUploader,
  })  : makeReader = makeReader ?? ((s) => PlateReader(apiKey: s.openAiKey, model: s.openAiModel)),
        makeUploader = makeUploader ?? ((u, s) => IncidentUploader(endpoint: u, token: s.uploadToken)) {
    _alertSub = module.alerts.listen(onAlert);
    _packageSub = module.packages.listen((p) => onPackage(p));
  }

  late final StreamSubscription<ClosePassAlert> _alertSub;
  late final StreamSubscription<ClosePassPackage> _packageSub;
  final Map<int, (DateTime, Future<Where?>)> _pending = {};

  ClosePassAlert? lastAlert;
  DateTime? lastAlertAt;

  /// Fires for each alert, so the app can show a banner.
  final _alertEvents = StreamController<ClosePassAlert>.broadcast();
  Stream<ClosePassAlert> get alertEvents => _alertEvents.stream;

  /// Fires when a new incident has been saved.
  final _incidentEvents = StreamController<Incident>.broadcast();
  Stream<Incident> get incidentEvents => _incidentEvents.stream;

  void onAlert(ClosePassAlert a) {
    lastAlert = a;
    lastAlertAt = DateTime.now();
    display.showCloseAlert(a);
    _pending[a.eventId] = (lastAlertAt!, locate().catchError((_) => null));
    if (_pending.length > 20) _pending.remove(_pending.keys.first);
    _alertEvents.add(a);
    notifyListeners();
  }

  Future<Incident> onPackage(ClosePassPackage p) async {
    final pending = _pending.remove(p.eventId);
    final time = pending?.$1 ?? DateTime.now();
    final where = await (pending?.$2 ?? locate().catchError((_) => null));
    // The phone's own reconstruction fills in what the module didn't send.
    final analysis = ClipAnalysis.of(p.clip);
    final incident = await store.add(
      p,
      time: time,
      lat: where?.lat,
      lng: where?.lng,
      accuracyM: where?.accuracyM,
      distanceM: p.minDistanceM ?? analysis.closestM,
      speedKmh: p.speedKmh ?? analysis.passSpeedKmh,
      simulated: module.isDemo,
    );
    _incidentEvents.add(incident);
    notifyListeners();
    unawaited(process(incident));
    return incident;
  }

  /// Incidents being processed right now, so the same one never runs twice
  /// at once (e.g. automatic processing and a tap on "Read plate").
  final Map<String, Future<void>> _running = {};

  /// Reads the plate, then uploads if appropriate.
  Future<void> process(Incident i) => _once('process', i, () async {
        await _readPlate(i);
        if (i.plate?.confident ?? false) await _upload(i, automatic: true);
      });

  Future<void> readPlate(Incident i) => _once('process', i, () => _readPlate(i));

  Future<void> upload(Incident i, {bool automatic = false}) => _once('upload', i, () => _upload(i, automatic: automatic));

  Future<void> _once(String what, Incident i, Future<void> Function() job) {
    final key = '$what:${i.id}';
    final running = _running[key];
    if (running != null) return running;
    // Block body: returning remove()'s value (this very future) would make
    // whenComplete wait for itself.
    final f = job().whenComplete(() {
      _running.remove(key);
    });
    _running[key] = f;
    return f;
  }

  Future<void> _readPlate(Incident i) async {
    if (!settings.hasOpenAiKey) {
      i.plateStatus = PlateStatus.noKey;
      i.plateError = 'Add your OpenAI API key in Settings, then tap Read plate.';
      await store.save(i);
      return;
    }
    if (i.photoFiles.isEmpty) {
      i.plateStatus = PlateStatus.none;
      i.plateError = 'No photos were received for this close pass.';
      await store.save(i);
      return;
    }
    i.plateStatus = PlateStatus.reading;
    i.plateError = null;
    await store.save(i);
    try {
      final photos = await store.photos(i);
      final result = await makeReader(settings).read(
        [
          for (var n = 0; n < photos.length; n++)
            PlatePhoto(photos[n],
                mimeType: i.photoFiles[n].endsWith('.png') ? 'image/png' : 'image/jpeg',
                offsetMs: n < i.photoOffsetsMs.length ? i.photoOffsetsMs[n] : 0),
        ],
        distanceM: i.distanceM,
        speedKmh: i.speedKmh,
        time: i.time,
      );
      i.plate = result;
      i.plateStatus = !result.found
          ? PlateStatus.none
          : (result.confident ? PlateStatus.found : PlateStatus.uncertain);
    } catch (e) {
      i.plateStatus = PlateStatus.failed;
      i.plateError = e.toString();
    }
    await store.save(i);
  }

  /// Uploads [i]. Automatic uploads only happen for confident plates; the
  /// user can upload any incident by hand.
  Future<void> _upload(Incident i, {bool automatic = false}) async {
    final uri = settings.uploadUri;
    if (uri == null) {
      i.uploadStatus = UploadStatus.notConfigured;
      i.uploadError = 'Add the website upload address in Settings.';
      await store.save(i);
      return;
    }
    if (automatic && !(i.plate?.confident ?? false)) return;
    i.uploadStatus = UploadStatus.uploading;
    i.uploadError = null;
    await store.save(i);
    try {
      final result = await makeUploader(uri, settings).upload(i,
          photos: await store.photos(i), lidar: await store.lidarBytes(i));
      i.uploadStatus = UploadStatus.uploaded;
      i.uploadedUrl = result.url;
      i.uploadedAt = DateTime.now();
    } catch (e) {
      i.uploadStatus = UploadStatus.failed;
      i.uploadError = e.toString();
    }
    await store.save(i);
  }

  /// Picks up work left unfinished (no key or website yet, app closed,
  /// no signal): reads waiting plates and retries waiting uploads.
  Future<void> resumePending() async {
    // Let anything already running finish first, then look at what's left.
    await settle();
    for (final i in store.incidents) {
      if (i.plateStatus == PlateStatus.waiting || (i.plateStatus == PlateStatus.noKey && settings.hasOpenAiKey)) {
        await process(i);
      } else if ((i.uploadStatus == UploadStatus.waiting || i.uploadStatus == UploadStatus.notConfigured) &&
          (i.plate?.confident ?? false) &&
          settings.uploadUri != null) {
        await upload(i, automatic: true);
      }
    }
  }

  /// Completes when no incident is being processed and everything is saved.
  Future<void> settle() async {
    while (_running.isNotEmpty) {
      await Future.wait(_running.values.toList());
    }
    await store.flush();
  }

  @override
  void dispose() {
    _alertSub.cancel();
    _packageSub.cancel();
    _alertEvents.close();
    _incidentEvents.close();
    super.dispose();
  }
}
