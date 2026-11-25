import 'dart:async';

import 'package:bike_navigation/bike_navigation.dart';
import 'package:flutter/foundation.dart';

import 'close_pass_package.dart';
import 'demo_scene.dart';
import 'protocol.dart';
import 'transport.dart';

/// Makes the demo photos (the app draws them; tests pass simple bytes).
typedef DemoPhotoMaker = Future<List<Uint8List>> Function(int count);

/// A Main Module that lives inside the app, for demos and tests without
/// hardware. It speaks exactly the Bluetooth protocol the real one must
/// (see protocol.dart), so everything after the radio is the real code path.
class DemoMainModule implements LinkTransport {
  final DemoPhotoMaker? photoMaker;

  /// Delay between notifications, to look like a real BLE link (~40 KB/s).
  final Duration notifyGap;
  final Duration photoDelay;

  DemoMainModule({this.photoMaker, this.notifyGap = const Duration(milliseconds: 4), this.photoDelay = const Duration(milliseconds: 1500)});

  final _notifications = StreamController<List<int>>.broadcast();
  final _done = Completer<void>();
  final _assembler = DisplayAssembler();

  /// What the (simulated) Front Module is showing.
  final ValueNotifier<DisplayFrame?> lcd = ValueNotifier(null);

  /// Every write the phone made, for tests.
  final List<List<int>> received = [];
  final List<int> acks = [];

  int _event = 0, _transfer = 0;
  Uint8List? _lastPackage;
  int? _lastTransferId;
  bool _closed = false;

  @override
  String get name => 'Demo module';

  @override
  bool get isDemo => true;

  @override
  Stream<List<int>> get notifications => _notifications.stream;

  @override
  Future<void> get done => _done.future;

  @override
  int get maxWrite => 244; // like a BLE link with MTU 247

  @override
  Future<void> write(List<int> data) async {
    if (_closed) throw StateError('Demo module disconnected');
    received.add(List.of(data));
    if (data.isEmpty) return;
    switch (data[0]) {
      case msgDisplayChunk:
        final frame = _assembler.add(data);
        if (frame != null) lcd.value = frame;
      case msgHello:
        _notify(const ModuleStatus(lidarOk: true, cameraOk: true, frontOk: true, batteryPercent: 87, firmware: '1.25').encode());
      case msgTransferAck when data.length >= 4:
        acks.add(data[3]);
        if (data[3] == ackResend && _lastPackage != null && u16(data, 1) == _lastTransferId) {
          await _sendTransfer(_lastTransferId!, _lastPackage!);
        }
    }
  }

  /// Plays a close pass: the alert at once, then (like the real module,
  /// after the camera burst) the package with photos and the LiDAR clip.
  Future<void> triggerClosePass({DemoScene scene = const DemoScene(), int photos = 5}) async {
    final eventId = ++_event;
    final clip = scene.clip();
    _notify(ClosePassAlert(eventId, minDistanceM: scene.gapM, speedKmh: scene.speedKmh).encode());
    await Future<void>.delayed(photoDelay);
    if (_closed) return;
    final images = await (photoMaker?.call(photos) ?? Future.value(<Uint8List>[]));
    final pkg = ClosePassPackage(
      eventId: eventId,
      eventUptimeMs: DateTime.now().millisecondsSinceEpoch & 0x7FFFFFFF,
      minDistanceM: scene.gapM,
      speedKmh: scene.speedKmh,
      photos: [for (var i = 0; i < images.length; i++) PackagePhoto(300 + i * 300, images[i])],
      clip: clip,
    ).encode();
    _lastPackage = pkg;
    _lastTransferId = ++_transfer & 0xFFFF;
    await _sendTransfer(_lastTransferId!, pkg);
  }

  Future<void> _sendTransfer(int id, Uint8List payload) async {
    for (final n in encodeTransfer(id, transferClosePass, payload, maxNotify: maxWrite)) {
      if (_closed) return;
      _notify(n);
      if (notifyGap > Duration.zero) await Future<void>.delayed(notifyGap);
    }
  }

  void _notify(List<int> bytes) {
    if (!_closed) _notifications.add(bytes);
  }

  @override
  Future<void> close() async {
    _closed = true;
    if (!_done.isCompleted) _done.complete();
  }
}
