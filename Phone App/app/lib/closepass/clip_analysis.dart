import 'dart:math' as math;

import '../link/close_pass_package.dart';

/// One step of the passing vehicle's path: where its LiDAR returns were
/// centred in one scan.
class TrackPoint {
  final int scanIndex;
  final int offsetMs;
  final double x;
  final double y;
  const TrackPoint(this.scanIndex, this.offsetMs, this.x, this.y);
}

/// What the phone works out from a LiDAR clip to reconstruct the pass.
class ClipAnalysis {
  /// Closest distance between the LiDAR and the vehicle, metres.
  final double? closestM;
  final int? closestScan;
  final LidarPoint? closestPoint;

  /// Speed of the vehicle as it came up behind the bike, km/h.
  final double? passSpeedKmh;

  /// The vehicle's centre, scan by scan.
  final List<TrackPoint> track;

  /// Bounding box of everything worth showing, metres (x right, y forward).
  final double minX, maxX, minY, maxY;

  const ClipAnalysis({
    this.closestM,
    this.closestScan,
    this.closestPoint,
    this.passSpeedKmh,
    required this.track,
    required this.minX,
    required this.maxX,
    required this.minY,
    required this.maxY,
  });

  static ClipAnalysis of(LidarClip clip) {
    double? best;
    int? bestScan;
    LidarPoint? bestPoint;
    // Closest close-pass return; if the clip has none, the closest vehicle return.
    for (final kind in [1, 2]) {
      for (var s = 0; s < clip.scans.length; s++) {
        for (final p in clip.scans[s].points) {
          if (p.kind != kind) continue;
          final d = math.sqrt(p.x * p.x + p.y * p.y);
          if (best == null || d < best) {
            best = d;
            bestScan = s;
            bestPoint = p;
          }
        }
      }
      if (best != null) break;
    }

    // Pass speed: the last estimate before the pass; otherwise the highest.
    double? speed;
    for (final s in clip.scans) {
      if (s.speedKmh != null && s.offsetMs <= 0) speed = s.speedKmh;
    }
    speed ??= clip.scans.map((s) => s.speedKmh).whereType<double>().fold<double?>(
        null, (a, b) => a == null || b > a ? b : a);

    final track = <TrackPoint>[];
    for (var s = 0; s < clip.scans.length; s++) {
      final pts = clip.scans[s].points.where((p) => p.kind != 0).toList();
      if (pts.length < 2) continue;
      final cx = pts.map((p) => p.x).reduce((a, b) => a + b) / pts.length;
      final cy = pts.map((p) => p.y).reduce((a, b) => a + b) / pts.length;
      track.add(TrackPoint(s, clip.scans[s].offsetMs, cx, cy));
    }

    // View box: the bike, the vehicle's track and most of the scene, ignoring
    // far-away outliers (1st-99th percentile of the points).
    final xs = <double>[0], ys = <double>[0];
    for (final s in clip.scans) {
      for (final p in s.points) {
        xs.add(p.x);
        ys.add(p.y);
      }
    }
    xs.sort();
    ys.sort();
    double pct(List<double> v, double q) => v[((v.length - 1) * q).round()];
    var minX = math.min(pct(xs, 0.01), -2.0), maxX = math.max(pct(xs, 0.99), 3.0);
    var minY = math.min(pct(ys, 0.01), -6.0), maxY = math.max(pct(ys, 0.99), 3.0);
    for (final t in track) {
      minX = math.min(minX, t.x - 1);
      maxX = math.max(maxX, t.x + 1);
      minY = math.min(minY, t.y - 1);
      maxY = math.max(maxY, t.y + 1);
    }
    return ClipAnalysis(
      closestM: best,
      closestScan: bestScan,
      closestPoint: bestPoint,
      passSpeedKmh: speed,
      track: track,
      minX: minX.clamp(-12, 0),
      maxX: maxX.clamp(0, 12),
      minY: minY.clamp(-14, 0),
      maxY: maxY.clamp(0, 12),
    );
  }
}
