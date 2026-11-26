import 'dart:async';

import 'package:flutter_blue_plus/flutter_blue_plus.dart';

import 'protocol.dart';
import 'transport.dart';

/// Finds and connects to Main Modules over Bluetooth LE (flutter_blue_plus).
///
/// flutter_blue_plus is used under its non-profit / educational licence
/// ([License.nonprofit]); commercial use needs its paid licence.
class BleFinder implements ModuleFinder {
  static final Guid _service = Guid(mainServiceUuid);
  static final Guid _toMain = Guid(phoneToMainUuid);
  static final Guid _toPhone = Guid(mainToPhoneUuid);

  @override
  Future<bool> ensureOn() async {
    if (!await FlutterBluePlus.isSupported) return false;
    var state = FlutterBluePlus.adapterStateNow;
    if (state == BluetoothAdapterState.unknown) {
      state = await FlutterBluePlus.adapterState
          .firstWhere((s) => s != BluetoothAdapterState.unknown)
          .timeout(const Duration(seconds: 3), onTimeout: () => BluetoothAdapterState.unknown);
    }
    if (state == BluetoothAdapterState.on) return true;
    try {
      await FlutterBluePlus.turnOn(); // Android asks the user
    } catch (_) {
      return false;
    }
    return FlutterBluePlus.adapterStateNow == BluetoothAdapterState.on;
  }

  /// A Main Module advertises the service UUID (usually in the advertising
  /// packet) and the name "BikeNav Main" (usually in the scan response).
  static bool _isMainModule(ScanResult r) =>
      r.advertisementData.serviceUuids.contains(_service) ||
      r.advertisementData.advName.startsWith('BikeNav') ||
      r.device.platformName.startsWith('BikeNav');

  @override
  Stream<List<FoundModule>> scan({Duration timeout = const Duration(seconds: 15)}) {
    late StreamController<List<FoundModule>> out;
    StreamSubscription<List<ScanResult>>? results;
    StreamSubscription<bool>? scanning;
    out = StreamController<List<FoundModule>>(
      onListen: () async {
        results = FlutterBluePlus.onScanResults.listen((list) {
          out.add([
            for (final r in list)
              if (_isMainModule(r))
                FoundModule(
                  r.device.remoteId.str,
                  r.advertisementData.advName.isNotEmpty
                      ? r.advertisementData.advName
                      : (r.device.platformName.isNotEmpty ? r.device.platformName : mainModuleName),
                  r.rssi,
                ),
          ]);
        });
        try {
          await FlutterBluePlus.startScan(timeout: timeout);
        } catch (e) {
          out.addError(e);
        }
        scanning = FlutterBluePlus.isScanning.where((s) => !s).listen((_) => out.close());
      },
      onCancel: () async {
        await results?.cancel();
        await scanning?.cancel();
        await FlutterBluePlus.stopScan();
      },
    );
    return out.stream;
  }

  @override
  Future<void> stopScan() => FlutterBluePlus.stopScan();

  @override
  Future<LinkTransport> open(String id, {String? name}) async {
    final device = BluetoothDevice.fromId(id);
    await device.connect(license: License.nonprofit, timeout: const Duration(seconds: 12), mtu: 512);
    try {
      final services = await device.discoverServices();
      final service = services.firstWhere((s) => s.uuid == _service,
          orElse: () => throw StateError('This device has no BikeNav service'));
      final toMain = service.characteristics.firstWhere((c) => c.uuid == _toMain);
      final toPhone = service.characteristics.firstWhere((c) => c.uuid == _toPhone);
      final transport = _BleTransport(device, toMain, toPhone, name ?? device.platformName);
      await transport._start();
      return transport;
    } catch (_) {
      await device.disconnect();
      rethrow;
    }
  }
}

class _BleTransport implements LinkTransport {
  final BluetoothDevice device;
  final BluetoothCharacteristic toMain;
  final BluetoothCharacteristic toPhone;
  @override
  final String name;
  final _notifications = StreamController<List<int>>.broadcast();
  final _done = Completer<void>();
  StreamSubscription<List<int>>? _values;
  StreamSubscription<BluetoothConnectionState>? _conn;

  _BleTransport(this.device, this.toMain, this.toPhone, String name)
      : name = name.isEmpty ? mainModuleName : name;

  Future<void> _start() async {
    _values = toPhone.onValueReceived.listen(_notifications.add);
    device.cancelWhenDisconnected(_values!);
    await toPhone.setNotifyValue(true);
    _conn = device.connectionState.listen((s) {
      if (s == BluetoothConnectionState.disconnected) _finish();
    });
  }

  void _finish() {
    if (!_done.isCompleted) _done.complete();
    _conn?.cancel();
    _values?.cancel();
  }

  @override
  Stream<List<int>> get notifications => _notifications.stream;

  @override
  Future<void> get done => _done.future;

  @override
  int get maxWrite => device.mtuNow - 3;

  @override
  bool get isDemo => false;

  /// Android's reason for the last disconnect, in plain words.
  @override
  String? get dropReason {
    final r = device.disconnectReason;
    if (r == null) return null;
    final text = switch (r.code) {
      8 => 'link timed out (radio lost the module)',
      19 => 'the module closed the connection',
      22 => 'the phone closed the connection',
      34 => 'the module stopped answering',
      62 => 'the connection failed to start',
      133 => 'Bluetooth error 133 (Android)',
      _ => r.description ?? 'unknown',
    };
    return '$text (code ${r.code})';
  }

  @override
  Future<void> write(List<int> data) =>
      toMain.write(data, withoutResponse: toMain.properties.writeWithoutResponse);

  @override
  Future<void> close() async {
    await device.disconnect();
    _finish();
  }
}
