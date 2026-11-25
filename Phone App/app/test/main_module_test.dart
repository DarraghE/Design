import 'dart:async';

import 'package:bike_app/link/close_pass_package.dart';
import 'package:bike_app/link/demo_main_module.dart';
import 'package:bike_app/link/main_module.dart';
import 'package:bike_app/link/protocol.dart';
import 'package:bike_app/link/transport.dart';
import 'package:bike_navigation/bike_navigation.dart';
import 'package:flutter_test/flutter_test.dart';

import 'helpers.dart';

/// A Bluetooth stand-in: fails to open [failures] times, then hands out demo modules.
class FakeFinder implements ModuleFinder {
  int failures;
  int opens = 0;
  final List<DemoMainModule> opened = [];
  FakeFinder({this.failures = 0});

  @override
  Future<bool> ensureOn() async => true;
  @override
  Stream<List<FoundModule>> scan({Duration timeout = const Duration(seconds: 15)}) =>
      Stream.value(const [FoundModule('AA:BB', 'BikeNav Main', -55)]);
  @override
  Future<void> stopScan() async {}
  @override
  Future<LinkTransport> open(String id, {String? name}) async {
    opens++;
    if (failures > 0) {
      failures--;
      throw StateError('not in range');
    }
    final d = DemoMainModule(photoMaker: fakePhotos, notifyGap: Duration.zero, photoDelay: Duration.zero);
    opened.add(d);
    return d;
  }
}

void main() {
  test('sends hello, gets status, and frames arrive pixel-exact at the module', () async {
    final m = MainModule();
    final demo = DemoMainModule(notifyGap: Duration.zero, photoDelay: Duration.zero);
    await m.attach(demo);
    expect(m.connected, isTrue);
    expect(demo.received.first, encodeHello());
    await until(() => m.status != null);
    expect(m.status!.firmware, '1.25');

    final frame = renderCloseAlert(distanceM: 0.9, speedKmh: 40);
    m.sendFrame(frame, flash: true);
    await m.flush();
    expect(demo.lcd.value, isNotNull);
    expect(sameFrame(demo.lcd.value!.frame, frame), isTrue);
    expect(demo.lcd.value!.flash, isTrue);
    expect(m.framesSent, 1);
    expect(demo.received.skip(1).every((p) => p.length <= demo.maxWrite), isTrue);
  });

  test('frames sent while busy collapse to the newest', () async {
    final m = MainModule();
    final demo = DemoMainModule(notifyGap: Duration.zero, photoDelay: Duration.zero);
    await m.attach(demo);
    for (var d = 100; d >= 10; d -= 10) {
      m.sendFrame(renderMessage('$d'));
    }
    await m.flush();
    expect(m.framesSent, lessThanOrEqualTo(2));
    expect(sameFrame(demo.lcd.value!.frame, renderMessage('10')), isTrue);
  });

  test('a frame chosen before connecting is sent on connect', () async {
    final m = MainModule();
    m.sendFrame(renderIdle());
    final demo = DemoMainModule(notifyGap: Duration.zero, photoDelay: Duration.zero);
    await m.attach(demo);
    await m.flush();
    expect(sameFrame(demo.lcd.value!.frame, renderIdle()), isTrue);
  });

  test('close pass: alert, then the package, then an OK ack', () async {
    final m = MainModule();
    final demo = DemoMainModule(photoMaker: fakePhotos, notifyGap: Duration.zero, photoDelay: Duration.zero);
    await m.attach(demo);
    final alerts = <ClosePassAlert>[];
    final packages = <ClosePassPackage>[];
    m.alerts.listen(alerts.add);
    m.packages.listen(packages.add);
    await demo.triggerClosePass(photos: 5);
    await until(() => packages.isNotEmpty);
    expect(alerts.single.eventId, packages.single.eventId);
    expect(alerts.single.minDistanceM, closeTo(0.85, 0.001));
    expect(packages.single.photos.length, 5);
    expect(packages.single.clip.scans, isNotEmpty);
    await m.flush();
    expect(demo.acks, [ackOk]);
    expect(m.transferProgress, isNull);
  });

  test('a damaged transfer is asked for again and then arrives', () async {
    final m = MainModule();
    final demo = _CorruptOnceDemo();
    await m.attach(demo);
    final packages = <ClosePassPackage>[];
    m.packages.listen(packages.add);
    await demo.triggerClosePass();
    await until(() => packages.isNotEmpty);
    await m.flush();
    expect(demo.acks, [ackResend, ackOk]);
  });

  test('remembers the chosen module and reconnects after drops', () async {
    final finder = FakeFinder(failures: 2);
    String? remembered;
    final m = MainModule(finder: finder, onRemember: (id, _) => remembered = id, reconnectDelay: const Duration(milliseconds: 10));
    final found = await finder.scan().first;
    await m.connectTo(found.single);
    expect(remembered, 'AA:BB');
    await until(() => m.connected);
    expect(finder.opens, 3, reason: 'two failed attempts, then success');
    // The link drops (bike switched off): the app keeps trying and gets it back.
    await finder.opened.last.close();
    await until(() => !m.connected);
    await until(() => m.connected);
    expect(finder.opened.length, 2);
    // Disconnecting by hand stops the retries.
    await m.disconnect();
    final before = finder.opens;
    await Future<void>.delayed(const Duration(milliseconds: 60));
    expect(finder.opens, before);
    expect(m.state, LinkState.disconnected);
  });
}

/// Sends the first package with one byte flipped, like radio corruption.
class _CorruptOnceDemo extends DemoMainModule {
  _CorruptOnceDemo() : super(photoMaker: fakePhotos, notifyGap: Duration.zero, photoDelay: Duration.zero);
  bool _corrupted = false;
  final _out = StreamController<List<int>>.broadcast();
  StreamSubscription<List<int>>? _sub;

  @override
  Stream<List<int>> get notifications {
    _sub ??= super.notifications.listen((n) {
      if (!_corrupted && n[0] == msgTransferData) {
        _corrupted = true;
        n = List.of(n)..[n.length - 1] ^= 0x5A;
      }
      _out.add(n);
    });
    return _out.stream;
  }
}
