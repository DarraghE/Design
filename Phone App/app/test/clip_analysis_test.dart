import 'dart:math' as math;

import 'package:bike_app/closepass/clip_analysis.dart';
import 'package:bike_app/link/close_pass_package.dart';
import 'package:bike_app/link/demo_scene.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('demo scene looks like a real close pass to the LiDAR', () {
    final clip = const DemoScene(gapM: 0.85, speedKmh: 38).clip();
    expect(clip.startMs, -2000);
    expect(clip.endMs, 1500);
    expect(clip.scans.length, 36); // 10 Hz
    // Every point is inside the sensor's crop (raw 55-215°, 30 mm - 12 m).
    for (final s in clip.scans) {
      for (final p in s.points) {
        final r = math.sqrt(p.x * p.x + p.y * p.y);
        expect(r, inInclusiveRange(0.03, 12.05));
        var raw = math.atan2(p.x, p.y) * 180 / math.pi;
        if (raw < 0) raw += 360;
        expect(raw, inInclusiveRange(54.9, 215.1));
      }
    }
    // The car is seen approaching from behind, then close on the right.
    expect(clip.scans.where((s) => s.offsetMs < 0).any((s) => s.incoming), isTrue);
    expect(clip.scans.first.incoming, isFalse, reason: 'the car starts out of LiDAR range');
    expect(clip.scans.any((s) => s.close), isTrue);
    expect(clip.scans.where((s) => s.points.any((p) => p.kind == 1)).length, greaterThan(3));
  });

  test('analysis finds the closest approach, speed and track', () {
    final clip = const DemoScene(gapM: 0.85, speedKmh: 38).clip();
    final a = ClipAnalysis.of(clip);
    expect(a.closestM, closeTo(0.85, 0.05));
    expect(a.closestPoint!.kind, 1);
    expect(a.passSpeedKmh, closeTo(38, 1.5));
    expect(a.track.length, greaterThan(10));
    // The vehicle moves forward (y increases) over the clip.
    expect(a.track.last.y, greaterThan(a.track.first.y));
    expect(a.track.every((t) => t.x > 0), isTrue, reason: 'it passes on the right');
    expect(a.minY, lessThan(-6));
    expect(a.maxX, greaterThan(2));
  });

  test('analysis copes with an empty clip or no vehicle', () {
    final empty = ClipAnalysis.of(const LidarClip([]));
    expect(empty.closestM, isNull);
    expect(empty.track, isEmpty);
    final wallOnly = ClipAnalysis.of(const LidarClip([
      LidarScan(offsetMs: 0, points: [LidarPoint(-2, 1, 0), LidarPoint(-2, 2, 0)]),
    ]));
    expect(wallOnly.closestM, isNull);
    expect(wallOnly.passSpeedKmh, isNull);
  });
}
