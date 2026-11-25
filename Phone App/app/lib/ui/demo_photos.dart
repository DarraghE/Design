import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:material_ui/material_ui.dart';

/// The plate shown in demo photos. Demo incidents are marked "simulated"
/// everywhere, including uploads.
const demoPlate = '262-D-12345';

/// Draws [count] 320x240 "camera" frames of a car pulling away after a close
/// pass, for the demo module. PNG bytes.
Future<List<Uint8List>> drawDemoPhotos(int count) async {
  final out = <Uint8List>[];
  for (var i = 0; i < count; i++) {
    out.add(await _frame(i, count));
  }
  return out;
}

Future<Uint8List> _frame(int i, int count) async {
  const w = 320.0, h = 240.0;
  final rec = ui.PictureRecorder();
  final c = Canvas(rec, const Rect.fromLTWH(0, 0, w, h));

  // Sky and buildings.
  c.drawRect(const Rect.fromLTWH(0, 0, w, h * 0.45),
      Paint()..shader = const LinearGradient(colors: [Color(0xFF9FB8CF), Color(0xFFD8E2EA)], begin: Alignment.topCenter, end: Alignment.bottomCenter).createShader(const Rect.fromLTWH(0, 0, w, h * 0.45)));
  final building = Paint()..color = const Color(0xFF8A7F74);
  c.drawRect(const Rect.fromLTWH(0, 40, 70, 70), building);
  c.drawRect(const Rect.fromLTWH(250, 30, 70, 80), Paint()..color = const Color(0xFF9B8C7A));
  // Road in perspective, horizon at y = 108.
  final road = Path()
    ..moveTo(0, h)
    ..lineTo(w, h)
    ..lineTo(w * 0.62, 108)
    ..lineTo(w * 0.38, 108)
    ..close();
  c.drawRect(const Rect.fromLTWH(0, 108, w, h - 108), Paint()..color = const Color(0xFF6F7A70));
  c.drawPath(road, Paint()..color = const Color(0xFF4A4E54));
  final line = Paint()
    ..color = const Color(0xFFEDEDED)
    ..strokeWidth = 3;
  for (var k = 0; k < 6; k++) {
    final t0 = k / 6, t1 = t0 + 0.07;
    double y(double t) => h - (h - 108) * t;
    double x(double t) => w * 0.5 + (w * 0.5 - w * 0.5) * t;
    c.drawLine(Offset(x(t0), y(t0)), Offset(x(t1), y(t1)), line);
  }

  // The car, getting smaller and moving ahead (up the frame) photo by photo.
  final t = count <= 1 ? 0.0 : i / (count - 1);
  final scale = 1.0 - 0.45 * t;
  final cx = w * (0.62 - 0.08 * t);
  final bottom = h - 18 - 70 * t;
  final cw = 190 * scale, ch = 120 * scale;
  final body = RRect.fromRectAndCorners(Rect.fromLTWH(cx - cw / 2, bottom - ch, cw, ch * 0.78),
      topLeft: Radius.circular(18 * scale), topRight: Radius.circular(18 * scale), bottomLeft: Radius.circular(8 * scale), bottomRight: Radius.circular(8 * scale));
  c.drawOval(Rect.fromLTWH(cx - cw * 0.55, bottom - ch * 0.16, cw * 1.1, ch * 0.24), Paint()..color = const Color(0x55000000));
  c.drawRRect(body, Paint()..color = const Color(0xFF1F3B63));
  // Rear window.
  c.drawRRect(
      RRect.fromRectAndRadius(Rect.fromLTWH(cx - cw * 0.36, bottom - ch * 0.98, cw * 0.72, ch * 0.30), Radius.circular(10 * scale)),
      Paint()..color = const Color(0xFF223040));
  // Tail lights and bumper.
  final red = Paint()..color = const Color(0xFFD7263D);
  c.drawRRect(RRect.fromRectAndRadius(Rect.fromLTWH(cx - cw * 0.48, bottom - ch * 0.62, cw * 0.18, ch * 0.10), Radius.circular(3 * scale)), red);
  c.drawRRect(RRect.fromRectAndRadius(Rect.fromLTWH(cx + cw * 0.30, bottom - ch * 0.62, cw * 0.18, ch * 0.10), Radius.circular(3 * scale)), red);
  c.drawRect(Rect.fromLTWH(cx - cw * 0.5, bottom - ch * 0.30, cw, ch * 0.08), Paint()..color = const Color(0xFF15161A));
  // Wheels.
  final tyre = Paint()..color = const Color(0xFF111111);
  c.drawRRect(RRect.fromRectAndRadius(Rect.fromLTWH(cx - cw * 0.46, bottom - ch * 0.24, cw * 0.16, ch * 0.24), Radius.circular(4 * scale)), tyre);
  c.drawRRect(RRect.fromRectAndRadius(Rect.fromLTWH(cx + cw * 0.30, bottom - ch * 0.24, cw * 0.16, ch * 0.24), Radius.circular(4 * scale)), tyre);

  // Irish plate: white, black letters, blue IRL band.
  final pw = cw * 0.52, ph = ch * 0.13;
  final plate = Rect.fromLTWH(cx - pw / 2, bottom - ch * 0.47, pw, ph);
  c.drawRRect(RRect.fromRectAndRadius(plate, Radius.circular(2 * scale)), Paint()..color = const Color(0xFFF7F7F2));
  c.drawRect(Rect.fromLTWH(plate.left, plate.top, pw * 0.11, ph), Paint()..color = const Color(0xFF1546A0));
  _text(c, demoPlate, Rect.fromLTWH(plate.left + pw * 0.12, plate.top, pw * 0.86, ph), Colors.black, ph * 0.82, FontWeight.w800);
  _text(c, 'IRL', Rect.fromLTWH(plate.left, plate.top + ph * 0.55, pw * 0.11, ph * 0.45), Colors.white, ph * 0.3, FontWeight.w700);

  // Camera overlay, like a dash cam.
  _text(c, 'CAM  +${(0.3 + 0.3 * i).toStringAsFixed(1)}s', const Rect.fromLTWH(6, 4, 140, 16), Colors.white, 11, FontWeight.w600, left: true);

  final img = await rec.endRecording().toImage(w.toInt(), h.toInt());
  final data = await img.toByteData(format: ui.ImageByteFormat.png);
  return data!.buffer.asUint8List();
}

void _text(Canvas c, String s, Rect r, Color colour, double size, FontWeight weight, {bool left = false}) {
  final b = ui.ParagraphBuilder(ui.ParagraphStyle(textAlign: left ? TextAlign.left : TextAlign.center, maxLines: 1))
    ..pushStyle(ui.TextStyle(fontFamily: 'Roboto', color: colour, fontSize: size, fontWeight: weight, letterSpacing: size * 0.04))
    ..addText(s);
  final p = b.build()..layout(ui.ParagraphConstraints(width: r.width));
  c.drawParagraph(p, Offset(r.left, r.top + (r.height - p.height) / 2));
}
