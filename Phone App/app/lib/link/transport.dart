/// A byte pipe to the Main Module: Bluetooth on the phone, the built-in demo
/// module, or a fake in tests.
abstract class LinkTransport {
  /// Shown in the app, e.g. "BikeNav Main" or "Demo module".
  String get name;

  /// Notifications from the Main Module, one message each.
  Stream<List<int>> get notifications;

  /// Fires once when the link drops.
  Future<void> get done;

  /// Most bytes one write may carry (BLE MTU minus 3).
  int get maxWrite;

  bool get isDemo => false;

  /// Why the link dropped, in plain words (null if unknown or still up).
  String? get dropReason => null;

  Future<void> write(List<int> data);

  Future<void> close();
}

/// A Main Module found by scanning.
class FoundModule {
  final String id;
  final String name;
  final int rssi;
  const FoundModule(this.id, this.name, this.rssi);
}

/// Finds and opens Main Modules (Bluetooth in the app, fakes in tests).
abstract class ModuleFinder {
  /// Current scan results; the stream ends when scanning stops.
  Stream<List<FoundModule>> scan({Duration timeout = const Duration(seconds: 15)});
  Future<void> stopScan();
  Future<LinkTransport> open(String id, {String? name});

  /// Whether Bluetooth is switched on (asks to switch it on where possible).
  Future<bool> ensureOn();
}
