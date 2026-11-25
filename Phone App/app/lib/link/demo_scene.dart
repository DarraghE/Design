import 'dart:math' as math;

import 'close_pass_package.dart';

/// Builds a realistic LiDAR clip of a car overtaking the bike too closely on
/// the right, by ray-casting the scene the way the STL-19P sees it (raw
/// angles 55-215°, x = r·sin(a), y = r·cos(a)). Used by the demo module.
class DemoScene {
  /// Gap between the bike and the car's left side, metres.
  final double gapM;
  final double speedKmh;
  final int seed;

  const DemoScene({this.gapM = 0.85, this.speedKmh = 38, this.seed = 3});

  static const double carWidth = 1.8, carLength = 4.4;
  static const int scanIntervalMs = 100; // 10 Hz

  /// Clip from [beforeMs] before the pass to [afterMs] after it. Time 0 is
  /// when the car's front draws level with the LiDAR.
  LidarClip clip({int beforeMs = 2000, int afterMs = 1500}) {
    final rnd = math.Random(seed);
    final speed = speedKmh / 3.6;
    final carLeft = gapM, carRight = gapM + carWidth;
    // Static scene: kerb/wall on the left, parked van and a sign post on the right.
    final walls = <_Seg>[
      const _Seg(-2.6, -14, -2.6, 8),
      const _Seg(4.6, -11, 4.6, -6.5),
      const _Seg(4.6, -6.5, 6.4, -6.5),
      const _Seg(6.4, -6.5, 6.4, -11),
      const _Seg(5.2, 2.5, 5.4, 2.5),
    ];
    final scans = <LidarScan>[];
    var closeSince = -1;
    for (var t = -beforeMs; t <= afterMs; t += scanIntervalMs) {
      final front = speed * t / 1000; // y of the car's front
      final car = <_Seg>[
        _Seg(carLeft, front, carLeft, front - carLength), // left side
        _Seg(carLeft, front, carRight, front), // front
        _Seg(carLeft, front - carLength, carRight, front - carLength), // back
      ];
      final points = <LidarPoint>[];
      var closeHits = 0;
      for (var a = 55.0; a <= 215.0; a += 0.75) {
        final rad = a * math.pi / 180;
        final dx = math.sin(rad), dy = math.cos(rad);
        double? hit;
        var isCar = false;
        for (final s in [...walls, ...car]) {
          final r = s.intersect(dx, dy);
          if (r != null && r <= 12 && (hit == null || r < hit)) {
            hit = r;
            isCar = car.contains(s);
          }
        }
        if (hit == null || hit < 0.03) continue;
        final r = hit + (rnd.nextDouble() - 0.5) * 0.03; // ±1.5 cm noise
        final x = r * dx, y = r * dy;
        var kind = 0;
        if (isCar) {
          if (r <= 1.5 && a < 180) {
            kind = 1;
            closeHits++;
          } else if (y < 0) {
            kind = 2;
          }
        }
        points.add(LidarPoint(x, y, kind));
      }
      if (closeHits >= 12) {
        if (closeSince < 0) closeSince = t;
      }
      final behind = front < 0 && front > -12;
      scans.add(LidarScan(
        offsetMs: t,
        close: closeSince >= 0 && t - closeSince <= 2000 && closeHits > 0,
        incoming: behind,
        speedKmh: behind ? speedKmh + (rnd.nextDouble() - 0.5) * 2 : null,
        points: points,
      ));
    }
    return LidarClip(scans);
  }
}

class _Seg {
  final double x0, y0, x1, y1;
  const _Seg(this.x0, this.y0, this.x1, this.y1);

  /// Distance along the ray (dx, dy) from the origin to this segment, or null.
  double? intersect(double dx, double dy) {
    final ex = x1 - x0, ey = y1 - y0;
    final den = dx * ey - dy * ex;
    if (den.abs() < 1e-9) return null;
    final t = (x0 * ey - y0 * ex) / den; // along the ray
    final u = (x0 * dy - y0 * dx) / den; // along the segment
    if (t <= 0 || u < 0 || u > 1) return null;
    return t;
  }
}
