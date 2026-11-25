// Decodes the notifications the Main Module firmware builds, using the app's
// real decoder. The file is written by Tools/Phone Link Test/run_test.py,
// which compiles the "PHONE WIRE" block of Main Module/main_module.ino on the
// computer (see host_test.cpp for the close pass it describes).
import 'dart:io';

import 'package:bike_app/link/close_pass_package.dart';
import 'package:bike_app/link/protocol.dart';
import 'package:flutter_test/flutter_test.dart';

const _file = '../../Tools/Phone Link Test/firmware_notifications.txt';

Map<String, List<List<int>>> _load() {
  final groups = <String, List<List<int>>>{};
  for (final line in File(_file).readAsLinesSync()) {
    if (line.isEmpty || line.startsWith('#')) continue;
    final parts = line.split(' ');
    final hex = parts[1];
    groups.putIfAbsent(parts[0], () => []).add([
      for (var i = 0; i < hex.length; i += 2) int.parse(hex.substring(i, i + 2), radix: 16),
    ]);
  }
  return groups;
}

void main() {
  final groups = _load();

  test('close-pass alert from the firmware', () {
    final m = decodeMainMessage(groups['alert']!.single) as ClosePassAlert;
    expect(m.eventId, 7);
    expect(m.minDistanceM, 0.85);
    expect(m.speedKmh, 38.0);
  });

  test('status from the firmware', () {
    final m = decodeMainMessage(groups['status']!.single) as ModuleStatus;
    expect(m.lidarOk, isTrue);
    expect(m.cameraOk, isFalse);
    expect(m.frontOk, isTrue);
    expect(m.batteryPercent, isNull);
    expect(m.firmware, '1.25');
  });

  for (final mtu in [23, 185, 247, 517]) {
    test('close-pass package from the firmware at MTU $mtu', () {
      final notifications = groups['transfer-mtu$mtu']!;
      for (final n in notifications) {
        expect(n.length, lessThanOrEqualTo(mtu - 3), reason: 'a notification larger than the MTU allows');
      }
      final receiver = TransferReceiver();
      TransferEvent? last;
      for (final n in notifications) {
        last = receiver.handle(decodeMainMessage(n)) ?? last;
      }
      expect(last, isA<TransferComplete>(), reason: '$last');
      final done = last as TransferComplete;
      expect(done.transferId, 3);
      expect(done.kind, transferClosePass);

      final pkg = ClosePassPackage.parse(done.payload);
      expect(pkg.eventId, 7);
      expect(pkg.eventUptimeMs, 123456);
      expect(pkg.minDistanceM, 0.85);
      expect(pkg.speedKmh, 38.0);

      expect(pkg.photos.map((p) => p.offsetMs), [300, 800, 1300]);
      expect(pkg.photos.map((p) => p.bytes.length), [1000, 2500, 7]);
      for (final p in pkg.photos) {
        expect(p.bytes.take(2), [0xFF, 0xD8]);
        expect(p.bytes.skip(p.bytes.length - 2), [0xFF, 0xD9]);
      }

      final scans = pkg.clip.scans;
      expect(scans.map((s) => s.offsetMs), [-2000, -1000, 0, 700, 1500]);
      expect(scans.map((s) => s.points.length), everyElement(200));
      expect(scans[0].incoming, isTrue);
      expect(scans[0].speedKmh, 38.0);
      expect(scans[2].close, isTrue);
      expect(scans[2].speedKmh, isNull);
      expect(scans[2].points.where((p) => p.kind == 1), hasLength(20));
      expect(scans[2].points.where((p) => p.kind == 1).every((p) => p.x == 0.85), isTrue);
      expect(scans[0].points.where((p) => p.kind == 2), hasLength(6));
      expect(scans[4].points.every((p) => p.kind == 0), isTrue);
    });
  }
}
