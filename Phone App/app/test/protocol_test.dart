import 'dart:convert';
import 'dart:math' as math;

import 'package:bike_app/link/close_pass_package.dart';
import 'package:bike_app/link/demo_scene.dart';
import 'package:bike_app/link/protocol.dart';
import 'package:flutter_test/flutter_test.dart';

import 'helpers.dart';

void main() {
  test('CRC-32 matches zlib (test vector for the ESP32 side)', () {
    expect(crc32(ascii.encode('123456789')), 0xCBF43926);
    expect(crc32(const []), 0);
  });

  test('little-endian helpers', () {
    expect(le16(0x1234), [0x34, 0x12]);
    expect(le32(0x12345678), [0x78, 0x56, 0x34, 0x12]);
    expect(i16(le16(-1234), 0), -1234);
    expect(i32(le32(-3000), 0), -3000);
  });

  group('messages', () {
    test('close-pass alert', () {
      final bytes = const ClosePassAlert(7, minDistanceM: 0.845, speedKmh: 41.6).encode();
      expect(bytes, [0xA1, 7, 0, 0x4D, 0x03, 0xA0, 0x01]); // 845 mm, 416 = 41.6 km/h
      final m = decodeMainMessage(bytes) as ClosePassAlert;
      expect(m.eventId, 7);
      expect(m.minDistanceM, closeTo(0.845, 1e-9));
      expect(m.speedKmh, closeTo(41.6, 1e-9));
      final unknown = decodeMainMessage([0xA1, 1, 0, 0xFF, 0xFF, 0xFF, 0xFF]) as ClosePassAlert;
      expect(unknown.minDistanceM, isNull);
      expect(unknown.speedKmh, isNull);
    });

    test('status', () {
      final m = decodeMainMessage(const ModuleStatus(lidarOk: true, cameraOk: false, frontOk: true, batteryPercent: 64, firmware: '1.14').encode())
          as ModuleStatus;
      expect([m.lidarOk, m.cameraOk, m.frontOk, m.batteryPercent, m.firmware], [true, false, true, 64, '1.14']);
      expect((decodeMainMessage([0xC1, 0, 255, 1, 0]) as ModuleStatus).batteryPercent, isNull);
    });

    test('short or unknown messages are ignored, not crashes', () {
      expect(decodeMainMessage([]), isA<UnknownMessage>());
      expect(decodeMainMessage([0xA1, 1]), isA<UnknownMessage>());
      expect(decodeMainMessage([0x77, 1, 2, 3]), isA<UnknownMessage>());
    });

    test('phone messages', () {
      expect(encodeHello(), [0x40, 1]);
      expect(encodeAck(0x0102, ackResend), [0x30, 0x02, 0x01, 1]);
    });
  });

  group('transfers', () {
    final payload = List<int>.generate(5000, (i) => (i * 7) & 0xFF);

    test('arrive whole, also out of order', () {
      final parts = encodeTransfer(9, transferClosePass, payload, maxNotify: 100);
      expect(parts.every((p) => p.length <= 100), isTrue);
      expect(parts.first.sublist(0, 4), [0xB1, 9, 0, 1]);
      final r = TransferReceiver();
      final data = parts.sublist(1, parts.length - 1)..shuffle(math.Random(1));
      expect(r.handle(decodeMainMessage(parts.first)), isNull);
      for (final p in data) {
        expect(r.handle(decodeMainMessage(p)), isNull);
      }
      expect(r.progress(9), 1.0);
      final done = r.handle(decodeMainMessage(parts.last)) as TransferComplete;
      expect(done.kind, transferClosePass);
      expect(done.payload, payload);
      expect(r.busy, isFalse);
    });

    test('missing part, bad CRC, missed start, oversize', () {
      final parts = encodeTransfer(3, 1, payload, maxNotify: 200);
      var r = TransferReceiver();
      for (final p in [...parts.take(3), ...parts.skip(4)]) {
        final e = r.handle(decodeMainMessage(p));
        if (p == parts.last) expect((e as TransferFailed).reason, contains('Missing'));
      }
      r = TransferReceiver();
      final bad = List<int>.of(parts.last)..[3] ^= 0xFF;
      for (final p in [...parts.take(parts.length - 1), bad]) {
        final e = r.handle(decodeMainMessage(p));
        if (p == bad) expect((e as TransferFailed).reason, 'CRC mismatch');
      }
      r = TransferReceiver();
      expect((r.handle(decodeMainMessage(parts.last)) as TransferFailed).reason, contains('start'));
      expect(r.handle(TransferStart(1, 1, TransferReceiver.maxLength + 1)), isA<TransferFailed>());
    });
  });

  group('close-pass package', () {
    test('round-trips with photos and the LiDAR clip', () {
      final clip = const DemoScene().clip();
      final pkg = ClosePassPackage(
        eventId: 513,
        eventUptimeMs: 123456,
        minDistanceM: 0.85,
        speedKmh: 46,
        photos: [PackagePhoto(300, tinyPng), PackagePhoto(-200, tinyPng)],
        clip: clip,
      );
      final bytes = pkg.encode();
      expect(bytes.sublist(0, 4), ascii.encode('CPK1'));
      final back = ClosePassPackage.parse(bytes);
      expect(back.eventId, 513);
      expect(back.eventUptimeMs, 123456);
      expect(back.minDistanceM, 0.85);
      expect(back.speedKmh, 46);
      expect(back.photos.map((p) => p.offsetMs), [300, -200]);
      expect(back.photos.first.bytes, tinyPng);
      expect(back.photos.first.mimeType, 'image/png');
      expect(back.clip.scans.length, clip.scans.length);
      final s0 = clip.scans[20], b0 = back.clip.scans[20];
      expect(b0.offsetMs, s0.offsetMs);
      expect(b0.points.length, s0.points.length);
      expect(b0.points.first.x, closeTo(s0.points.first.x, 0.001));
      expect(b0.points.first.kind, s0.points.first.kind);
      expect([b0.close, b0.incoming], [s0.close, s0.incoming]);
    });

    test('rejects damaged packages', () {
      final bytes = ClosePassPackage(eventId: 1, photos: [PackagePhoto(0, tinyPng)], clip: const DemoScene().clip()).encode();
      expect(() => ClosePassPackage.parse(bytes.sublist(0, 30)), throwsFormatException);
      expect(() => ClosePassPackage.parse(bytes.sublist(0, bytes.length - 3)), throwsFormatException);
      expect(() => ClosePassPackage.parse([0, 1, 2, 3, ...bytes.skip(4)]), throwsFormatException);
    });
  });
}
