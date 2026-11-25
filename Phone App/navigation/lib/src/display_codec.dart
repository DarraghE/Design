import 'dart:typed_data';

import 'oled.dart';

/// How the phone sends a handlebar screen to the Main Module, which passes
/// each packet on unchanged (over ESP-NOW) to the Front Module.
///
/// 1. The 128x64 frame is turned into 1024 bytes, **row by row from the top,
///    8 pixels per byte, leftmost pixel in the most significant bit, 1 = lit**.
///    This is exactly the buffer Adafruit_GFX `drawBitmap()` takes, so the
///    Front Module can draw it directly.
/// 2. Each row (16 bytes) is XORed with the row above it ([xorRows]); row 0
///    stays as it is. Shapes change little from row to row, so most bytes
///    become 0.
/// 3. The result is compressed with **PackBits** run-length encoding
///    ([packBits]). Navigation screens shrink from 1024 bytes to about
///    100-430 bytes (median ~180), so most fit in one or two packets.
/// 4. The compressed bytes are cut into chunks of at most [maxChunkData]
///    bytes, and each chunk is sent as one display packet ([encodeDisplayPackets]):
///
/// | Byte | Meaning |
/// |------|---------|
/// | 0 | `0x10` (display chunk) |
/// | 1 | Frame number, 0-255, +1 for every new frame (wraps) |
/// | 2 | Chunk index, 0-based |
/// | 3 | Chunk count (1-255) |
/// | 4 | Flags: bit 0 = flash (invert the screen 4 times a second) |
/// | 5.. | Chunk data (part of the compressed frame) |
///
/// The receiver joins chunks 0..count-1 of one frame number in index order,
/// unpacks them ([unpackBits]), must get exactly 1024 bytes, then undoes
/// step 2 ([unxorRows]: for rows 1 to 63 in order, XOR each byte with the
/// byte above it) and draws the buffer.
const int displayChunkType = 0x10;
const int displayHeaderBytes = 5;

/// Fits in one ESP-NOW packet (250 bytes) and a BLE write at MTU >= 188.
const int maxChunkData = 180;

const int frameBytes = OledFrame.width * OledFrame.height ~/ 8; // 1024

/// Frame pixels as row-major bytes, MSB = leftmost pixel (drawBitmap layout).
Uint8List frameToRowMajor(OledFrame f) {
  final out = Uint8List(frameBytes);
  for (var y = 0; y < OledFrame.height; y++) {
    for (var x = 0; x < OledFrame.width; x++) {
      if (f.pixel(x, y)) out[y * 16 + (x >> 3)] |= 0x80 >> (x & 7);
    }
  }
  return out;
}

/// The reverse of [frameToRowMajor].
OledFrame rowMajorToFrame(List<int> bytes) {
  if (bytes.length != frameBytes) {
    throw FormatException('A frame is $frameBytes bytes, got ${bytes.length}');
  }
  final f = OledFrame();
  for (var y = 0; y < OledFrame.height; y++) {
    for (var x = 0; x < OledFrame.width; x++) {
      if (bytes[y * 16 + (x >> 3)] & (0x80 >> (x & 7)) != 0) f.set(x, y);
    }
  }
  return f;
}

/// Step 2: XOR every row with the row above it (row 0 unchanged).
Uint8List xorRows(List<int> rowMajor) {
  final out = Uint8List.fromList(rowMajor);
  for (var i = rowMajor.length - 1; i >= 16; i--) {
    out[i] ^= rowMajor[i - 16];
  }
  return out;
}

/// Undoes [xorRows]: rows 1..63 in order, each byte XOR the (already
/// restored) byte above it.
Uint8List unxorRows(List<int> delta) {
  final out = Uint8List.fromList(delta);
  for (var i = 16; i < out.length; i++) {
    out[i] ^= out[i - 16];
  }
  return out;
}

/// PackBits run-length encoding.
///
/// Output is a series of runs, each starting with a header byte `h`:
/// - `h` 0-127: the next `h + 1` bytes are copied as they are (literal run);
/// - `h` 129-255: the next single byte is repeated `257 - h` times (2-128);
/// - `h` 128: never written (skip it if seen).
Uint8List packBits(List<int> input) {
  final out = BytesBuilder(copy: false);
  var i = 0;
  final n = input.length;
  while (i < n) {
    // Length of the run of equal bytes starting at i (max 128).
    var run = 1;
    while (i + run < n && run < 128 && input[i + run] == input[i]) {
      run++;
    }
    if (run >= 2) {
      out.add([257 - run, input[i]]);
      i += run;
      continue;
    }
    // Literal run: up to 128 bytes, stopping before a repeat of 2 or more.
    final start = i;
    var len = 0;
    while (i < n && len < 128) {
      if (i + 1 < n && input[i + 1] == input[i]) break;
      i++;
      len++;
    }
    out.addByte(len - 1);
    out.add(input.sublist(start, start + len));
  }
  return out.toBytes();
}

/// Decodes [packBits] output. Throws [FormatException] on truncated input.
Uint8List unpackBits(List<int> input) {
  final out = BytesBuilder(copy: false);
  var i = 0;
  while (i < input.length) {
    final h = input[i++];
    if (h < 128) {
      final count = h + 1;
      if (i + count > input.length) throw const FormatException('Truncated literal run');
      out.add(input.sublist(i, i + count));
      i += count;
    } else if (h > 128) {
      if (i >= input.length) throw const FormatException('Truncated repeat run');
      out.add(List<int>.filled(257 - h, input[i++]));
    }
  }
  return out.toBytes();
}

/// One frame ready to send: its packets plus the compressed size, for logs.
class DisplayPackets {
  final List<Uint8List> packets;
  final int compressedBytes;
  const DisplayPackets(this.packets, this.compressedBytes);
}

/// Builds the display packets for [frame]. [chunkData] is the most chunk
/// data per packet (default [maxChunkData]; smaller when the BLE MTU is small).
DisplayPackets encodeDisplayPackets(OledFrame frame,
    {required int frameNumber, bool flash = false, int chunkData = maxChunkData}) {
  if (chunkData < 1) throw ArgumentError.value(chunkData, 'chunkData');
  final packed = packBits(xorRows(frameToRowMajor(frame)));
  final count = (packed.length + chunkData - 1) ~/ chunkData;
  if (count > 255) throw StateError('Frame needs more than 255 chunks');
  final packets = <Uint8List>[];
  for (var c = 0; c < count; c++) {
    final start = c * chunkData;
    final end = (start + chunkData).clamp(0, packed.length);
    packets.add(Uint8List.fromList([
      displayChunkType,
      frameNumber & 0xFF,
      c,
      count,
      flash ? 1 : 0,
      ...packed.sublist(start, end),
    ]));
  }
  return DisplayPackets(packets, packed.length);
}

/// A decoded display frame (used by the demo module and by tests; the
/// Front Module firmware does the same in C++).
class DisplayFrame {
  final int frameNumber;
  final bool flash;
  final OledFrame frame;
  const DisplayFrame(this.frameNumber, this.flash, this.frame);
}

/// Collects display packets and returns a [DisplayFrame] once every chunk of
/// a frame has arrived. Starting a new frame number discards a half-received one.
class DisplayAssembler {
  int? _frame;
  int _count = 0;
  bool _flash = false;
  final Map<int, List<int>> _chunks = {};

  DisplayFrame? add(List<int> packet) {
    if (packet.length < displayHeaderBytes || packet[0] != displayChunkType) {
      throw const FormatException('Not a display packet');
    }
    final number = packet[1], index = packet[2], count = packet[3];
    if (count == 0 || index >= count) throw const FormatException('Bad chunk index');
    if (number != _frame || count != _count) {
      _frame = number;
      _count = count;
      _chunks.clear();
    }
    _flash = packet[4] & 1 == 1;
    _chunks[index] = packet.sublist(displayHeaderBytes);
    if (_chunks.length < _count) return null;
    final packed = <int>[for (var c = 0; c < _count; c++) ..._chunks[c]!];
    _chunks.clear();
    _frame = null;
    return DisplayFrame(number, _flash, rowMajorToFrame(unxorRows(unpackBits(packed))));
  }
}
