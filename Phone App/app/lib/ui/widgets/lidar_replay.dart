import 'dart:async';
import 'dart:math' as math;

import 'package:material_ui/material_ui.dart';

import '../../closepass/clip_analysis.dart';
import '../../link/close_pass_package.dart';
import '../theme.dart';

/// Animated top-down reconstruction of a close pass from the LiDAR clip.
class LidarReplay extends StatefulWidget {
  final LidarClip clip;
  final ClipAnalysis analysis;
  final bool autoplay;
  const LidarReplay({super.key, required this.clip, required this.analysis, this.autoplay = true});

  @override
  State<LidarReplay> createState() => _LidarReplayState();
}

class _LidarReplayState extends State<LidarReplay> with SingleTickerProviderStateMixin {
  late final AnimationController _anim;
  double _speed = 0.5;
  Timer? _loop;

  int get _startMs => widget.clip.startMs;
  int get _endMs => math.max(widget.clip.endMs, _startMs + 1);
  double get _tMs => _startMs + (_endMs - _startMs) * _anim.value;

  @override
  void initState() {
    super.initState();
    _anim = AnimationController(vsync: this, duration: _duration())
      ..addListener(() => setState(() {}))
      ..addStatusListener((s) {
        if (s == AnimationStatus.completed && widget.autoplay) {
          _loop = Timer(const Duration(milliseconds: 1400), () {
            if (mounted && _playing) _anim.forward(from: 0);
          });
        }
      });
    if (widget.autoplay && widget.clip.scans.length > 1) {
      _playing = true;
      _anim.forward();
    }
  }

  bool _playing = false;

  Duration _duration() => Duration(milliseconds: ((_endMs - _startMs) / _speed).round().clamp(1, 600000));

  void _toggle() {
    setState(() => _playing = !_playing);
    if (_playing) {
      _anim.forward(from: _anim.value >= 1 ? 0 : _anim.value);
    } else {
      _loop?.cancel();
      _anim.stop();
    }
  }

  void _setSpeed() {
    setState(() {
      _speed = _speed == 0.5 ? 1 : (_speed == 1 ? 0.25 : 0.5);
      _anim.duration = _duration();
    });
    if (_playing) _anim.forward(from: _anim.value);
  }

  @override
  void dispose() {
    _loop?.cancel();
    _anim.dispose();
    super.dispose();
  }

  int _scanAt(double t) {
    final scans = widget.clip.scans;
    var i = 0;
    while (i + 1 < scans.length && scans[i + 1].offsetMs <= t) {
      i++;
    }
    return i;
  }

  @override
  Widget build(BuildContext context) {
    final scans = widget.clip.scans;
    if (scans.isEmpty) {
      return const AspectRatio(aspectRatio: 1, child: Center(child: Text('No LiDAR data in this recording')));
    }
    final t = _tMs;
    final index = _scanAt(t);
    final scan = scans[index];
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        ClipRRect(
          borderRadius: BorderRadius.circular(22),
          child: AspectRatio(
            aspectRatio: 0.92,
            child: Stack(children: [
              Positioned.fill(
                child: CustomPaint(
                  painter: _ReplayPainter(clip: widget.clip, analysis: widget.analysis, index: index, tMs: t),
                ),
              ),
              Positioned(
                left: 14,
                top: 12,
                child: Text(
                  '${t >= 0 ? '+' : '−'}${(t.abs() / 1000).toStringAsFixed(1)} s',
                  style: const TextStyle(fontSize: 26, fontWeight: FontWeight.w700, fontFeatures: [FontFeature.tabularFigures()]),
                ),
              ),
              Positioned(
                right: 12,
                top: 12,
                child: Column(crossAxisAlignment: CrossAxisAlignment.end, children: [
                  if (scan.incoming)
                    _Badge('APPROACHING${scan.speedKmh == null ? '' : ' · ${scan.speedKmh!.round()} km/h'}', Palette.warning),
                  if (scan.close) ...[const SizedBox(height: 6), const _Badge('CLOSE PASS', Palette.danger)],
                ]),
              ),
              const Positioned(
                left: 12,
                bottom: 10,
                child: Row(children: [
                  _Legend(Color(0xFF7F93AA), 'Scene'),
                  SizedBox(width: 12),
                  _Legend(Palette.warning, 'Approaching'),
                  SizedBox(width: 12),
                  _Legend(Palette.danger, 'Close pass'),
                ]),
              ),
            ]),
          ),
        ),
        const SizedBox(height: 6),
        Row(children: [
          IconButton.filledTonal(
            onPressed: _toggle,
            icon: Icon(_playing && _anim.isAnimating ? Icons.pause : Icons.play_arrow),
            tooltip: _playing ? 'Pause' : 'Play',
          ),
          Expanded(
            child: Slider(
              value: _anim.value,
              onChanged: (v) {
                _loop?.cancel();
                _anim.stop();
                setState(() {
                  _playing = false;
                  _anim.value = v;
                });
              },
            ),
          ),
          TextButton(onPressed: _setSpeed, child: Text('${_speed == 0.25 ? '¼' : (_speed == 0.5 ? '½' : '1')}×')),
        ]),
      ],
    );
  }
}

class _Badge extends StatelessWidget {
  final String text;
  final Color colour;
  const _Badge(this.text, this.colour);

  @override
  Widget build(BuildContext context) => Container(
        padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
        decoration: BoxDecoration(
          color: colour.withValues(alpha: 0.18),
          borderRadius: BorderRadius.circular(20),
          border: Border.all(color: colour.withValues(alpha: 0.8)),
        ),
        child: Text(text, style: TextStyle(color: colour, fontSize: 12, fontWeight: FontWeight.w800, letterSpacing: 0.6)),
      );
}

class _Legend extends StatelessWidget {
  final Color colour;
  final String label;
  const _Legend(this.colour, this.label);

  @override
  Widget build(BuildContext context) => Row(children: [
        Container(width: 8, height: 8, decoration: BoxDecoration(color: colour, shape: BoxShape.circle)),
        const SizedBox(width: 5),
        Text(label, style: const TextStyle(fontSize: 11, color: Palette.muted)),
      ]);
}

class _ReplayPainter extends CustomPainter {
  final LidarClip clip;
  final ClipAnalysis analysis;
  final int index;
  final double tMs;
  _ReplayPainter({required this.clip, required this.analysis, required this.index, required this.tMs});

  static const _scene = Color(0xFF7F93AA);

  @override
  void paint(Canvas canvas, Size size) {
    // World (metres, x right, y forward) -> screen, same scale on both axes.
    final a = analysis;
    final worldW = a.maxX - a.minX, worldH = a.maxY - a.minY + 1.8; // room for the bike
    final scale = math.min(size.width / (worldW + 1.2), size.height / (worldH + 1.6));
    final ox = size.width / 2 - (a.minX + a.maxX) / 2 * scale;
    final oy = size.height / 2 + (a.minY + a.maxY + 1.8) / 2 * scale;
    Offset p(double x, double y) => Offset(ox + x * scale, oy - y * scale);
    final bike = p(0, 0);

    // Background.
    canvas.drawRect(Offset.zero & size,
        Paint()..shader = RadialGradient(colors: const [Color(0xFF0F1A26), Color(0xFF070A0F)], radius: 0.9, center: Alignment((bike.dx / size.width) * 2 - 1, (bike.dy / size.height) * 2 - 1)).createShader(Offset.zero & size));

    // Approach zone (1.5-12 m behind/right, x <= 5 m) and close zone (1.5 m, right side).
    final approach = Path();
    for (var deg = -125.0; deg <= 0; deg += 2) {
      final r = deg * math.pi / 180;
      var x = 12 * math.cos(r), y = 12 * math.sin(r);
      if (x > 5) {
        y = y * 5 / x;
        x = 5;
      }
      final q = p(x, y);
      deg == -125 ? approach.moveTo(q.dx, q.dy) : approach.lineTo(q.dx, q.dy);
    }
    for (var deg = 0.0; deg >= -125; deg -= 5) {
      final r = deg * math.pi / 180;
      final q = p(1.5 * math.cos(r), 1.5 * math.sin(r));
      approach.lineTo(q.dx, q.dy);
    }
    approach.close();
    canvas.drawPath(approach, Paint()..color = Palette.warning.withValues(alpha: 0.05));
    canvas.drawPath(approach, Paint()..style = PaintingStyle.stroke..strokeWidth = 1..color = Palette.warning.withValues(alpha: 0.25));
    final scan = clip.scans[index];
    final closeZone = Path()..moveTo(bike.dx, bike.dy);
    for (var raw = 55.0; raw <= 180; raw += 3) {
      final r = raw * math.pi / 180;
      final q = p(1.5 * math.sin(r), 1.5 * math.cos(r));
      closeZone.lineTo(q.dx, q.dy);
    }
    closeZone.close();
    canvas.drawPath(closeZone, Paint()..color = Palette.danger.withValues(alpha: scan.close ? 0.22 : 0.08));
    canvas.drawPath(closeZone, Paint()..style = PaintingStyle.stroke..strokeWidth = 1.2..color = Palette.danger.withValues(alpha: 0.5));

    // Range rings.
    final ring = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1
      ..color = const Color(0xFF1E2C3B);
    for (final r in [3.0, 6.0, 9.0, 12.0]) {
      canvas.drawCircle(bike, r * scale, ring);
      _label(canvas, '${r.round()} m', p(-r * 0.71, -r * 0.71) + const Offset(-14, 2), const Color(0xFF3A4C60), 10);
    }

    // Trail of earlier scans, fading out.
    for (var back = 6; back >= 1; back--) {
      final i = index - back;
      if (i < 0) continue;
      final alpha = 0.32 * (1 - back / 7);
      for (final pt in clip.scans[i].points) {
        final colour = pt.kind == 1 ? Palette.danger : (pt.kind == 2 ? Palette.warning : _scene);
        canvas.drawCircle(p(pt.x, pt.y), pt.kind == 0 ? 1.4 : 2.0, Paint()..color = colour.withValues(alpha: alpha));
      }
    }

    // Vehicle path so far.
    final track = analysis.track.where((tp) => tp.offsetMs <= tMs).toList();
    if (track.length >= 2) {
      final path = Path()..moveTo(p(track.first.x, track.first.y).dx, p(track.first.x, track.first.y).dy);
      for (final tp in track.skip(1)) {
        final q = p(tp.x, tp.y);
        path.lineTo(q.dx, q.dy);
      }
      canvas.drawPath(path, Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2
        ..strokeCap = StrokeCap.round
        ..color = Colors.white.withValues(alpha: 0.35));
      for (final tp in track) {
        canvas.drawCircle(p(tp.x, tp.y), 1.8, Paint()..color = Colors.white.withValues(alpha: 0.45));
      }
    }

    // Current scan.
    final glow = Paint()..maskFilter = const MaskFilter.blur(BlurStyle.normal, 4);
    for (final pt in scan.points) {
      final q = p(pt.x, pt.y);
      switch (pt.kind) {
        case 1:
          canvas.drawCircle(q, 5, glow..color = Palette.danger.withValues(alpha: 0.6));
          canvas.drawCircle(q, 2.8, Paint()..color = Palette.danger);
        case 2:
          canvas.drawCircle(q, 4, glow..color = Palette.warning.withValues(alpha: 0.45));
          canvas.drawCircle(q, 2.4, Paint()..color = Palette.warning);
        default:
          canvas.drawCircle(q, 1.7, Paint()..color = _scene);
      }
    }

    // Closest approach, once it has happened.
    final cs = analysis.closestScan, cp = analysis.closestPoint, cm = analysis.closestM;
    if (cs != null && cp != null && cm != null && clip.scans[cs].offsetMs <= tMs) {
      final q = p(cp.x, cp.y);
      _dashed(canvas, bike, q, Paint()
        ..color = Palette.danger
        ..strokeWidth = 2);
      canvas.drawCircle(q, 6, Paint()
        ..style = PaintingStyle.stroke
        ..strokeWidth = 2
        ..color = Palette.danger);
      final mid = Offset((bike.dx + q.dx) / 2, (bike.dy + q.dy) / 2);
      final label = '${cm.toStringAsFixed(2)} m';
      final tp = _layout(label, Colors.white, 13, FontWeight.w800);
      final box = Rect.fromCenter(center: mid + const Offset(0, -16), width: tp.width + 14, height: tp.height + 6);
      canvas.drawRRect(RRect.fromRectAndRadius(box, const Radius.circular(8)), Paint()..color = Palette.danger);
      tp.paint(canvas, box.topLeft + const Offset(7, 3));
    }

    // The bike, top-down, front up: a slim body with a heading arrow. The
    // LiDAR (origin) is at its back.
    final front = p(0, 1.7);
    final bodyRect = Rect.fromPoints(p(-0.16, 0.05), p(0.16, 1.45));
    canvas.drawRRect(RRect.fromRectAndRadius(bodyRect.inflate(5), Radius.circular(bodyRect.width)),
        Paint()..color = Palette.accent.withValues(alpha: 0.18)..maskFilter = const MaskFilter.blur(BlurStyle.normal, 8));
    canvas.drawRRect(RRect.fromRectAndRadius(bodyRect, Radius.circular(bodyRect.width / 2)), Paint()..color = Colors.white);
    final head = Path()
      ..moveTo(front.dx, front.dy)
      ..lineTo(p(0.22, 1.42).dx, p(0.22, 1.42).dy)
      ..lineTo(p(-0.22, 1.42).dx, p(-0.22, 1.42).dy)
      ..close();
    canvas.drawPath(head, Paint()..color = Colors.white);
    canvas.drawCircle(bike, 4, Paint()..color = Palette.accent);
    _label(canvas, 'YOU', p(0, 1.75) + const Offset(-11, -16), Colors.white70, 10);
  }

  void _dashed(Canvas c, Offset a, Offset b, Paint paint) {
    final len = (b - a).distance;
    const dash = 6.0, gap = 4.0;
    final dir = (b - a) / len;
    for (var d = 0.0; d < len; d += dash + gap) {
      c.drawLine(a + dir * d, a + dir * math.min(d + dash, len), paint);
    }
  }

  TextPainter _layout(String s, Color colour, double size, [FontWeight weight = FontWeight.w600]) =>
      TextPainter(text: TextSpan(text: s, style: TextStyle(fontFamily: 'Roboto', color: colour, fontSize: size, fontWeight: weight)), textDirection: TextDirection.ltr)
        ..layout();

  void _label(Canvas c, String s, Offset at, Color colour, double size) => _layout(s, colour, size).paint(c, at);

  @override
  bool shouldRepaint(_ReplayPainter old) => old.index != index || old.tMs != tMs || old.clip != clip;
}
