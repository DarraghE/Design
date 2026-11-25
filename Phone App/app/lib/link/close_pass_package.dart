// The close-pass package: what the Main Module sends after a close pass,
// as one transfer of kind 1 (see protocol.dart). Pure Dart.
//
// Layout, little-endian:
//
//   0   4  magic "CPK1"
//   4   2  eventId (same number as the close-pass alert)
//   6   4  eventUptimeMs (Main Module millis() when the pass was detected)
//   10  2  minDistanceMm  (0xFFFF = unknown)
//   12  2  speedKmhX10    (0xFFFF = unknown)
//   14  1  photoCount N
//   15  1  reserved (0)
//   then N photos, each:
//       4  i32 offsetMs (photo time minus event time; negative = before)
//       4  u32 length L
//       L  JPEG bytes
//   then the LiDAR clip (also stored on its own as lidar.bin):
//       2  scanCount S
//       2  reserved (0)
//       S scans, each:
//           4  i32 offsetMs (scan time minus event time)
//           1  flags: bit 0 = close pass active, bit 1 = vehicle approaching
//           2  speedKmhX10 of the approaching vehicle (0xFFFF = none)
//           2  pointCount P (max 1024)
//           P points, 5 bytes each: i16 x mm, i16 y mm, u8 class
//              (x = right, y = forward, LiDAR at 0,0; class 0 normal,
//               1 close-pass return, 2 approaching-vehicle return)
import 'dart:typed_data';

import 'protocol.dart';

const List<int> packageMagic = [0x43, 0x50, 0x4B, 0x31]; // "CPK1"

class LidarPoint {
  final double x; // metres, right of the LiDAR
  final double y; // metres, ahead of the LiDAR (negative = behind)
  final int kind; // 0 normal, 1 close pass, 2 approaching vehicle
  const LidarPoint(this.x, this.y, this.kind);
}

class LidarScan {
  final int offsetMs;
  final bool close;
  final bool incoming;
  final double? speedKmh;
  final List<LidarPoint> points;
  const LidarScan({
    required this.offsetMs,
    this.close = false,
    this.incoming = false,
    this.speedKmh,
    required this.points,
  });
}

class LidarClip {
  final List<LidarScan> scans;
  const LidarClip(this.scans);

  int get startMs => scans.isEmpty ? 0 : scans.first.offsetMs;
  int get endMs => scans.isEmpty ? 0 : scans.last.offsetMs;

  Uint8List encode() {
    final b = BytesBuilder(copy: false)
      ..add(le16(scans.length))
      ..add(le16(0));
    for (final s in scans) {
      if (s.points.length > 1024) throw ArgumentError('A scan has more than 1024 points');
      b
        ..add(le32(s.offsetMs))
        ..addByte((s.close ? 1 : 0) | (s.incoming ? 2 : 0))
        ..add(le16(s.speedKmh == null ? unknown16 : (s.speedKmh! * 10).round().clamp(0, 0xFFFE)))
        ..add(le16(s.points.length));
      for (final p in s.points) {
        b
          ..add(le16((p.x * 1000).round().clamp(-32768, 32767)))
          ..add(le16((p.y * 1000).round().clamp(-32768, 32767)))
          ..addByte(p.kind);
      }
    }
    return b.toBytes();
  }

  /// Parses a clip starting at [offset]; returns it with the offset after it.
  static (LidarClip, int) parseAt(List<int> b, int offset) {
    var o = offset;
    void need(int n) {
      if (o + n > b.length) throw const FormatException('LiDAR clip is truncated');
    }

    need(4);
    final count = u16(b, o);
    o += 4;
    final scans = <LidarScan>[];
    for (var s = 0; s < count; s++) {
      need(9);
      final offsetMs = i32(b, o);
      final flags = b[o + 4];
      final speed = u16(b, o + 5);
      final n = u16(b, o + 7);
      o += 9;
      if (n > 1024) throw const FormatException('Too many points in a LiDAR scan');
      need(n * 5);
      final points = List<LidarPoint>.generate(n, (i) {
        final p = o + i * 5;
        return LidarPoint(i16(b, p) / 1000, i16(b, p + 2) / 1000, b[p + 4]);
      });
      o += n * 5;
      scans.add(LidarScan(
        offsetMs: offsetMs,
        close: flags & 1 != 0,
        incoming: flags & 2 != 0,
        speedKmh: speed == unknown16 ? null : speed / 10,
        points: points,
      ));
    }
    return (LidarClip(scans), o);
  }

  static LidarClip parse(List<int> b) => parseAt(b, 0).$1;
}

class PackagePhoto {
  final int offsetMs;
  final Uint8List bytes;
  const PackagePhoto(this.offsetMs, this.bytes);

  /// image/jpeg normally; the demo module sends PNGs.
  String get mimeType => bytes.length > 3 && bytes[0] == 0x89 && bytes[1] == 0x50 ? 'image/png' : 'image/jpeg';
  String get extension => mimeType == 'image/png' ? 'png' : 'jpg';
}

class ClosePassPackage {
  final int eventId;
  final int eventUptimeMs;
  final double? minDistanceM;
  final double? speedKmh;
  final List<PackagePhoto> photos;
  final LidarClip clip;

  const ClosePassPackage({
    required this.eventId,
    this.eventUptimeMs = 0,
    this.minDistanceM,
    this.speedKmh,
    required this.photos,
    required this.clip,
  });

  Uint8List encode() {
    final b = BytesBuilder(copy: false)
      ..add(packageMagic)
      ..add(le16(eventId))
      ..add(le32(eventUptimeMs))
      ..add(le16(minDistanceM == null ? unknown16 : (minDistanceM! * 1000).round().clamp(0, 0xFFFE)))
      ..add(le16(speedKmh == null ? unknown16 : (speedKmh! * 10).round().clamp(0, 0xFFFE)))
      ..addByte(photos.length)
      ..addByte(0);
    for (final p in photos) {
      b
        ..add(le32(p.offsetMs))
        ..add(le32(p.bytes.length))
        ..add(p.bytes);
    }
    b.add(clip.encode());
    return b.toBytes();
  }

  static ClosePassPackage parse(List<int> b) {
    if (b.length < 16 || !_startsWith(b, packageMagic)) {
      throw const FormatException('Not a close-pass package (bad magic)');
    }
    final dist = u16(b, 10), speed = u16(b, 12);
    final photoCount = b[14];
    var o = 16;
    final photos = <PackagePhoto>[];
    for (var i = 0; i < photoCount; i++) {
      if (o + 8 > b.length) throw const FormatException('Photo header is truncated');
      final offsetMs = i32(b, o);
      final len = u32(b, o + 4);
      o += 8;
      if (o + len > b.length) throw const FormatException('Photo is truncated');
      photos.add(PackagePhoto(offsetMs, Uint8List.fromList(b.sublist(o, o + len))));
      o += len;
    }
    final (clip, _) = LidarClip.parseAt(b, o);
    return ClosePassPackage(
      eventId: u16(b, 4),
      eventUptimeMs: u32(b, 6),
      minDistanceM: dist == unknown16 ? null : dist / 1000,
      speedKmh: speed == unknown16 ? null : speed / 10,
      photos: photos,
      clip: clip,
    );
  }

  static bool _startsWith(List<int> b, List<int> prefix) {
    for (var i = 0; i < prefix.length; i++) {
      if (b[i] != prefix[i]) return false;
    }
    return true;
  }
}
