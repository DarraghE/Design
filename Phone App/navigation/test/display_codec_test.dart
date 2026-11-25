import 'dart:math' as math;
import 'dart:typed_data';

import 'package:bike_navigation/bike_navigation.dart';
import 'package:test/test.dart';

NavState turn(ManoeuvreKind kind, double distance, {int? exit, bool flash = false}) => NavState(
      next: Manoeuvre(kind: kind, instruction: '', pointIndex: 0, alongM: 0, exitNumber: exit),
      distanceToNextM: distance,
      showTurn: true,
      symbols: const [],
      offRoute: false,
      arrived: false,
      flash: flash,
      alongM: 0,
      offsetM: 0,
    );

bool sameFrame(OledFrame a, OledFrame b) {
  for (var y = 0; y < 64; y++) {
    for (var x = 0; x < 128; x++) {
      if (a.pixel(x, y) != b.pixel(x, y)) return false;
    }
  }
  return true;
}

void main() {
  test('row-major layout: MSB is the leftmost pixel, 16 bytes per row', () {
    final f = OledFrame()
      ..set(0, 0)
      ..set(9, 0)
      ..set(127, 63);
    final b = frameToRowMajor(f);
    expect(b.length, 1024);
    expect(b[0], 0x80);
    expect(b[1], 0x40); // x = 9 is bit 6 of byte 1
    expect(b[1023], 0x01);
    expect(sameFrame(rowMajorToFrame(b), f), isTrue);
  });

  test('row XOR is undone exactly', () {
    final f = renderOled(turn(ManoeuvreKind.sharpRight, 340));
    final rows = frameToRowMajor(f);
    final delta = xorRows(rows);
    expect(delta.sublist(0, 16), rows.sublist(0, 16), reason: 'row 0 unchanged');
    expect(delta.where((b) => b == 0).length, greaterThan(rows.where((b) => b == 0).length));
    expect(unxorRows(delta), rows);
  });

  group('PackBits', () {
    test('known vector', () {
      // Standard PackBits example (Apple TN1023).
      final input = [0xAA, 0xAA, 0xAA, 0x80, 0x00, 0x2A, 0xAA, 0xAA, 0xAA, 0xAA, 0x80, 0x00, 0x2A, 0x22, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA];
      final packed = packBits(input);
      expect(packed, [0xFE, 0xAA, 0x02, 0x80, 0x00, 0x2A, 0xFD, 0xAA, 0x03, 0x80, 0x00, 0x2A, 0x22, 0xF7, 0xAA]);
      expect(unpackBits(packed), input);
    });

    test('blank frame is tiny, random data round-trips', () {
      expect(packBits(Uint8List(1024)).length, 16); // 8 runs of 128 zeros
      final rnd = math.Random(7);
      for (var trial = 0; trial < 50; trial++) {
        final len = rnd.nextInt(1500);
        final data = List<int>.generate(len, (_) => rnd.nextInt(4) == 0 ? rnd.nextInt(256) : 0);
        expect(unpackBits(packBits(data)), data);
      }
      // Worst case (no repeats) grows by at most 1 byte per 128.
      final noisy = List<int>.generate(1024, (i) => i & 0xFF);
      expect(packBits(noisy).length, lessThanOrEqualTo(1024 + 8));
      expect(unpackBits(packBits(noisy)), noisy);
    });

    test('rejects truncated input', () {
      expect(() => unpackBits([0x05, 1, 2]), throwsFormatException);
      expect(() => unpackBits([0xFE]), throwsFormatException);
    });
  });

  test('every screen round-trips through the display packets', () {
    final screens = <OledFrame>[
      for (final k in ManoeuvreKind.values)
        for (final d in [4.0, 87.0, 640.0, 2300.0]) renderOled(turn(k, d, exit: k == ManoeuvreKind.roundabout ? 3 : null)),
      renderCloseAlert(distanceM: 0.7, speedKmh: 55),
      renderIdle(),
      OledFrame(),
    ];
    var largest = 0;
    for (var i = 0; i < screens.length; i++) {
      final encoded = encodeDisplayPackets(screens[i], frameNumber: i, flash: i.isOdd);
      largest = math.max(largest, encoded.compressedBytes);
      final assembler = DisplayAssembler();
      DisplayFrame? out;
      for (final p in encoded.packets) {
        expect(p.length, lessThanOrEqualTo(displayHeaderBytes + maxChunkData));
        expect(p[0], displayChunkType);
        expect(p[1], i & 0xFF);
        expect(p[3], encoded.packets.length);
        out = assembler.add(p);
      }
      expect(out, isNotNull);
      expect(out!.flash, i.isOdd);
      expect(sameFrame(out.frame, screens[i]), isTrue);
    }
    expect(largest, lessThan(450), reason: 'row-XOR + PackBits compresses every screen to at most 3 packets');
  });

  test('small chunks, out-of-order chunks and a new frame mid-way', () {
    final frame = renderOled(turn(ManoeuvreKind.left, 120));
    final encoded = encodeDisplayPackets(frame, frameNumber: 200, chunkData: 20);
    expect(encoded.packets.length, greaterThan(3));
    final assembler = DisplayAssembler();
    // Half of an older frame, then the new frame arriving backwards.
    final other = encodeDisplayPackets(renderIdle(), frameNumber: 199, chunkData: 20);
    assembler.add(other.packets.first);
    DisplayFrame? out;
    for (final p in encoded.packets.reversed) {
      out = assembler.add(p);
    }
    expect(out!.frameNumber, 200);
    expect(sameFrame(out.frame, frame), isTrue);
    expect(() => assembler.add([0x99, 0, 0, 1, 0]), throwsFormatException);
  });
}
