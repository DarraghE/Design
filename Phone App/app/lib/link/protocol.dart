// Bluetooth LE protocol between the phone and the Main Module, version 1.
//
// Pure Dart (no Flutter) so it can be tested on a laptop. The full byte-level
// description, written for whoever implements the ESP32 side, is in
// Phone App/SPECIFICATION.md section 5 and Main Module/SPECIFICATION.md.
//
// GATT layout (the Main Module is the BLE peripheral):
//   Service            6e3b2000-8f7a-4c1e-9b52-2d1c0b7a1f10
//   Phone -> Main      6e3b2001-...  write / write without response
//   Main -> Phone      6e3b2002-...  notify
// Advertised name: "BikeNav Main". All numbers are little-endian.
import 'dart:typed_data';

const String mainServiceUuid = '6e3b2000-8f7a-4c1e-9b52-2d1c0b7a1f10';
const String phoneToMainUuid = '6e3b2001-8f7a-4c1e-9b52-2d1c0b7a1f10';
const String mainToPhoneUuid = '6e3b2002-8f7a-4c1e-9b52-2d1c0b7a1f10';
const String mainModuleName = 'BikeNav Main';
const int protocolVersion = 1;

// ---- message types ----------------------------------------------------------
// Phone -> Main
/// Display chunk, forwarded unchanged to the Front Module (see the navigation
/// package's display_codec.dart).
const int msgDisplayChunk = 0x10;
/// Acknowledge a finished transfer: [0x30][u16 transferId][u8 status].
const int msgTransferAck = 0x30;
/// Sent once after connecting: [0x40][u8 protocolVersion].
const int msgHello = 0x40;

// Main -> Phone
/// A close pass was just detected: [0xA1][u16 eventId][u16 minDistanceMm][u16 speedKmhX10].
const int msgClosePassAlert = 0xA1;
/// Start of a data transfer: [0xB1][u16 transferId][u8 kind][u32 totalLength].
const int msgTransferStart = 0xB1;
/// Part of a transfer: [0xB2][u16 transferId][u32 offset][bytes...].
const int msgTransferData = 0xB2;
/// End of a transfer: [0xB3][u16 transferId][u32 crc32 of all the bytes].
const int msgTransferEnd = 0xB3;
/// Module status: [0xC1][u8 flags][u8 batteryPercent (255 = unknown)][u8 fwMajor][u8 fwMinor].
const int msgStatus = 0xC1;

/// Transfer kinds.
const int transferClosePass = 1;

/// Ack status values.
const int ackOk = 0;
const int ackResend = 1;

/// "Unknown" value for 16-bit distance and speed fields.
const int unknown16 = 0xFFFF;

// ---- helpers ------------------------------------------------------------------

int u16(List<int> b, int o) => b[o] | (b[o + 1] << 8);
int u32(List<int> b, int o) => u16(b, o) | (u16(b, o + 2) << 16);
int i16(List<int> b, int o) {
  final v = u16(b, o);
  return v >= 0x8000 ? v - 0x10000 : v;
}

int i32(List<int> b, int o) {
  final v = u32(b, o);
  return v >= 0x80000000 ? v - 0x100000000 : v;
}

List<int> le16(int v) => [v & 0xFF, (v >> 8) & 0xFF];
List<int> le32(int v) => [v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF];

final Uint32List _crcTable = () {
  final t = Uint32List(256);
  for (var n = 0; n < 256; n++) {
    var c = n;
    for (var k = 0; k < 8; k++) {
      c = (c & 1) != 0 ? 0xEDB88320 ^ (c >> 1) : c >> 1;
    }
    t[n] = c;
  }
  return t;
}();

/// Standard CRC-32 (zlib / Ethernet / Python `binascii.crc32`).
/// crc32("123456789") == 0xCBF43926.
int crc32(List<int> data) {
  var c = 0xFFFFFFFF;
  for (final b in data) {
    c = _crcTable[(c ^ b) & 0xFF] ^ (c >> 8);
  }
  return (c ^ 0xFFFFFFFF) & 0xFFFFFFFF;
}

// ---- decoded messages from the Main Module ------------------------------------

sealed class MainMessage {
  const MainMessage();
}

class ClosePassAlert extends MainMessage {
  final int eventId;
  final double? minDistanceM;
  final double? speedKmh;
  const ClosePassAlert(this.eventId, {this.minDistanceM, this.speedKmh});

  List<int> encode() => [
        msgClosePassAlert,
        ...le16(eventId),
        ...le16(minDistanceM == null ? unknown16 : (minDistanceM! * 1000).round().clamp(0, 0xFFFE)),
        ...le16(speedKmh == null ? unknown16 : (speedKmh! * 10).round().clamp(0, 0xFFFE)),
      ];
}

class ModuleStatus extends MainMessage {
  final bool lidarOk;
  final bool cameraOk;
  final bool frontOk;
  final int? batteryPercent;
  final String firmware;
  const ModuleStatus({
    required this.lidarOk,
    required this.cameraOk,
    required this.frontOk,
    this.batteryPercent,
    this.firmware = '',
  });

  List<int> encode() {
    final parts = firmware.split('.');
    return [
      msgStatus,
      (lidarOk ? 1 : 0) | (cameraOk ? 2 : 0) | (frontOk ? 4 : 0),
      batteryPercent ?? 255,
      int.tryParse(parts.first) ?? 0,
      parts.length > 1 ? int.tryParse(parts[1]) ?? 0 : 0,
    ];
  }
}

class TransferStart extends MainMessage {
  final int transferId;
  final int kind;
  final int totalLength;
  const TransferStart(this.transferId, this.kind, this.totalLength);
}

class TransferData extends MainMessage {
  final int transferId;
  final int offset;
  final List<int> bytes;
  const TransferData(this.transferId, this.offset, this.bytes);
}

class TransferEnd extends MainMessage {
  final int transferId;
  final int crc;
  const TransferEnd(this.transferId, this.crc);
}

class UnknownMessage extends MainMessage {
  final List<int> bytes;
  const UnknownMessage(this.bytes);
}

/// Decodes one notification from the Main Module.
MainMessage decodeMainMessage(List<int> b) {
  if (b.isEmpty) return const UnknownMessage([]);
  double? opt16(int v, double scale) => v == unknown16 ? null : v / scale;
  switch (b[0]) {
    case msgClosePassAlert when b.length >= 7:
      return ClosePassAlert(u16(b, 1), minDistanceM: opt16(u16(b, 3), 1000), speedKmh: opt16(u16(b, 5), 10));
    case msgStatus when b.length >= 5:
      return ModuleStatus(
        lidarOk: b[1] & 1 != 0,
        cameraOk: b[1] & 2 != 0,
        frontOk: b[1] & 4 != 0,
        batteryPercent: b[2] == 255 ? null : b[2],
        firmware: '${b[3]}.${b[4]}',
      );
    case msgTransferStart when b.length >= 8:
      return TransferStart(u16(b, 1), b[3], u32(b, 4));
    case msgTransferData when b.length >= 7:
      return TransferData(u16(b, 1), u32(b, 3), b.sublist(7));
    case msgTransferEnd when b.length >= 7:
      return TransferEnd(u16(b, 1), u32(b, 3));
  }
  return UnknownMessage(b);
}

// ---- messages the phone sends ---------------------------------------------------

List<int> encodeHello() => [msgHello, protocolVersion];
List<int> encodeAck(int transferId, int status) => [msgTransferAck, ...le16(transferId), status];

// ---- sending a transfer (what the Main Module does; used by the demo module and tests)

/// Splits [payload] into the notifications the Main Module sends:
/// one START, DATA parts of at most [maxNotify] bytes each, one END.
List<List<int>> encodeTransfer(int transferId, int kind, List<int> payload, {int maxNotify = 180}) {
  final dataPerPart = maxNotify - 7;
  if (dataPerPart < 1) throw ArgumentError.value(maxNotify, 'maxNotify');
  return [
    [msgTransferStart, ...le16(transferId), kind, ...le32(payload.length)],
    for (var o = 0; o < payload.length; o += dataPerPart)
      [msgTransferData, ...le16(transferId), ...le32(o), ...payload.sublist(o, (o + dataPerPart).clamp(0, payload.length))],
    [msgTransferEnd, ...le16(transferId), ...le32(crc32(payload))],
  ];
}

// ---- receiving a transfer ----------------------------------------------------------

/// Result of feeding a message to [TransferReceiver].
sealed class TransferEvent {
  const TransferEvent();
}

class TransferComplete extends TransferEvent {
  final int transferId;
  final int kind;
  final Uint8List payload;
  const TransferComplete(this.transferId, this.kind, this.payload);
}

class TransferFailed extends TransferEvent {
  final int transferId;
  final String reason;
  const TransferFailed(this.transferId, this.reason);
}

/// Reassembles transfers. DATA parts may arrive in any order; the END
/// message's CRC-32 decides whether the transfer is good.
class TransferReceiver {
  /// Largest transfer accepted (a close-pass package is normally 50-250 KB).
  static const int maxLength = 2 * 1024 * 1024;

  final Map<int, _Incoming> _active = {};

  TransferEvent? handle(MainMessage m) {
    switch (m) {
      case TransferStart(:final transferId, :final kind, :final totalLength):
        if (totalLength > maxLength) {
          _active.remove(transferId);
          return TransferFailed(transferId, 'Transfer too large ($totalLength bytes)');
        }
        _active[transferId] = _Incoming(kind, totalLength);
        return null;
      case TransferData(:final transferId, :final offset, :final bytes):
        final t = _active[transferId];
        if (t == null) return null; // missed START; END will report it
        if (offset + bytes.length > t.buffer.length) {
          return TransferFailed(transferId, 'Data past the end of the transfer');
        }
        t.buffer.setRange(offset, offset + bytes.length, bytes);
        t.received += bytes.length;
        return null;
      case TransferEnd(:final transferId, :final crc):
        final t = _active.remove(transferId);
        if (t == null) return TransferFailed(transferId, 'Missed the start of the transfer');
        if (t.received < t.buffer.length) {
          return TransferFailed(transferId, 'Missing ${t.buffer.length - t.received} bytes');
        }
        if (crc32(t.buffer) != crc) return TransferFailed(transferId, 'CRC mismatch');
        return TransferComplete(transferId, t.kind, t.buffer);
      default:
        return null;
    }
  }

  /// Bytes received so far for a transfer in progress, 0-1 (for progress UI).
  double? progress(int transferId) {
    final t = _active[transferId];
    return t == null || t.buffer.isEmpty ? null : (t.received / t.buffer.length).clamp(0, 1);
  }

  bool get busy => _active.isNotEmpty;
}

class _Incoming {
  final int kind;
  final Uint8List buffer;
  int received = 0;
  _Incoming(this.kind, int length) : buffer = Uint8List(length);
}
