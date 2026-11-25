import 'dart:convert';
import 'dart:io';

import 'package:flutter/foundation.dart';

import '../link/close_pass_package.dart';
import 'incident.dart';

/// Keeps close passes on the phone, one folder each:
///   `incidents/ID/incident.json`   details, plate result, upload state
///   `incidents/ID/photo_1.jpg` ...  photos from the Camera Module
///   `incidents/ID/lidar.bin`       LiDAR clip (format in close_pass_package.dart)
class IncidentStore extends ChangeNotifier {
  final Directory root;
  IncidentStore(this.root);

  List<Incident> _items = [];

  /// Newest first.
  List<Incident> get incidents => List.unmodifiable(_items);

  Future<void> load() async {
    await root.create(recursive: true);
    final loaded = <Incident>[];
    await for (final e in root.list()) {
      if (e is! Directory) continue;
      final f = File('${e.path}/incident.json');
      if (!await f.exists()) continue;
      try {
        loaded.add(Incident.fromJson(jsonDecode(await f.readAsString()) as Map<String, dynamic>));
      } catch (_) {
        // A half-written incident: skip it.
      }
    }
    loaded.sort((a, b) => b.time.compareTo(a.time));
    _items = loaded;
    notifyListeners();
  }

  Directory dirOf(String id) => Directory('${root.path}/$id');

  /// Saves a new close pass from a package, and returns it.
  Future<Incident> add(ClosePassPackage pkg, {required DateTime time, double? lat, double? lng, double? accuracyM, double? distanceM, double? speedKmh, bool simulated = false}) async {
    final id = _newId(time);
    final dir = await dirOf(id).create(recursive: true);
    final names = <String>[];
    for (var i = 0; i < pkg.photos.length; i++) {
      final name = 'photo_${i + 1}.${pkg.photos[i].extension}';
      await File('${dir.path}/$name').writeAsBytes(pkg.photos[i].bytes, flush: true);
      names.add(name);
    }
    await File('${dir.path}/lidar.bin').writeAsBytes(pkg.clip.encode(), flush: true);
    final incident = Incident(
      id: id,
      time: time,
      lat: lat,
      lng: lng,
      accuracyM: accuracyM,
      distanceM: distanceM ?? pkg.minDistanceM,
      speedKmh: speedKmh ?? pkg.speedKmh,
      photoFiles: names,
      photoOffsetsMs: [for (final p in pkg.photos) p.offsetMs],
      simulated: simulated,
    );
    await save(incident);
    return incident;
  }

  String _newId(DateTime t) {
    String two(int v) => v.toString().padLeft(2, '0');
    final base = '${t.year}${two(t.month)}${two(t.day)}-${two(t.hour)}${two(t.minute)}${two(t.second)}';
    var id = base;
    for (var n = 2; _items.any((i) => i.id == id) || dirOf(id).existsSync(); n++) {
      id = '$base-$n';
    }
    return id;
  }

  /// Writes run one at a time, so two saves of one incident never collide.
  Future<void> _writes = Future.value();

  /// Completes when every save made so far is on disk.
  Future<void> flush() => _writes;

  /// Writes [incident] and refreshes listeners.
  Future<void> save(Incident incident) {
    // Snapshot now, so the file matches the incident at the time of the call.
    final json = const JsonEncoder.withIndent('  ').convert(incident.toJson());
    final write = _writes.then((_) async {
      final dir = await dirOf(incident.id).create(recursive: true);
      final tmp = File('${dir.path}/incident.json.tmp');
      await tmp.writeAsString(json, flush: true);
      await tmp.rename('${dir.path}/incident.json');
    });
    _writes = write.catchError((_) {});
    _remember(incident);
    return write;
  }

  void _remember(Incident incident) {
    final i = _items.indexWhere((x) => x.id == incident.id);
    if (i >= 0) {
      _items[i] = incident;
    } else {
      _items.insert(0, incident);
      _items.sort((a, b) => b.time.compareTo(a.time));
    }
    notifyListeners();
  }

  Future<List<Uint8List>> photos(Incident i) async => [
        for (final name in i.photoFiles)
          if (await File('${dirOf(i.id).path}/$name').exists()) await File('${dirOf(i.id).path}/$name').readAsBytes(),
      ];

  File photoFile(Incident i, int index) => File('${dirOf(i.id).path}/${i.photoFiles[index]}');

  Future<Uint8List?> lidarBytes(Incident i) async {
    final f = File('${dirOf(i.id).path}/lidar.bin');
    return await f.exists() ? f.readAsBytes() : null;
  }

  Future<LidarClip?> clip(Incident i) async {
    final b = await lidarBytes(i);
    if (b == null) return null;
    try {
      return LidarClip.parse(b);
    } catch (_) {
      return null;
    }
  }

  Future<void> delete(Incident i) async {
    await _writes;
    final d = dirOf(i.id);
    if (await d.exists()) await d.delete(recursive: true);
    _items.removeWhere((x) => x.id == i.id);
    notifyListeners();
  }

  Incident? byId(String id) {
    for (final i in _items) {
      if (i.id == id) return i;
    }
    return null;
  }
}
