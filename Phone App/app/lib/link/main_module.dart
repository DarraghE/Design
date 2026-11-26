import 'dart:async';
import 'dart:math' as math;

import 'package:bike_navigation/bike_navigation.dart';
import 'package:flutter/foundation.dart';

import 'close_pass_package.dart';
import 'protocol.dart';
import 'transport.dart';

enum LinkState { disconnected, connecting, connected }

/// The phone's connection to the Main Module: sends handlebar frames, and
/// turns notifications into close-pass alerts and close-pass packages.
///
/// "Pairing" is simply choosing the module once: its id is remembered and
/// the app reconnects to it automatically whenever it is in range.
class MainModule extends ChangeNotifier {
  final ModuleFinder? finder;
  final Duration reconnectDelay;

  /// Called with the id/name of a module the user picked, to remember it.
  final void Function(String id, String name)? onRemember;

  MainModule({this.finder, this.onRemember, this.reconnectDelay = const Duration(seconds: 4)});

  LinkState _state = LinkState.disconnected;
  LinkState get state => _state;
  bool get connected => _state == LinkState.connected;

  LinkTransport? _transport;
  bool get isDemo => _transport?.isDemo ?? false;
  String? get name => _transport?.name ?? _wantedName;

  ModuleStatus? _status;
  ModuleStatus? get status => _status;

  String? _error;
  String? get error => _error;

  /// How often the link has dropped since the app started, and why it last
  /// did, to diagnose an unreliable connection.
  int drops = 0;
  String? lastDrop;
  DateTime? lastDropAt;

  /// 0-1 while a close-pass package is arriving.
  double? _progress;
  double? get transferProgress => _progress;

  final _alerts = StreamController<ClosePassAlert>.broadcast();
  final _packages = StreamController<ClosePassPackage>.broadcast();
  Stream<ClosePassAlert> get alerts => _alerts.stream;
  Stream<ClosePassPackage> get packages => _packages.stream;

  StreamSubscription<List<int>>? _sub;
  final _receiver = TransferReceiver();
  String? _wantedId, _wantedName;
  Timer? _retry;
  bool _disposed = false;

  // ---- connecting ---------------------------------------------------------------

  /// Connects to a module found by scanning and remembers it.
  Future<void> connectTo(FoundModule m) async {
    onRemember?.call(m.id, m.name);
    await connectToId(m.id, name: m.name);
  }

  /// Connects to a known module id, retrying until [disconnect] is called.
  Future<void> connectToId(String id, {String? name}) async {
    await _drop();
    _wantedId = id;
    _wantedName = name;
    await _tryConnect();
  }

  Future<void> _tryConnect() async {
    final id = _wantedId, f = finder;
    if (id == null || f == null || _disposed) return;
    _retry?.cancel();
    _set(LinkState.connecting);
    try {
      final t = await f.open(id, name: _wantedName);
      if (_wantedId != id || _disposed) {
        await t.close();
        return;
      }
      _error = null;
      await attach(t);
    } catch (e) {
      _error = _describe(e);
      _set(LinkState.connecting); // still trying
      _retry = Timer(reconnectDelay, _tryConnect);
    }
  }

  /// Uses [t] as the link (the demo module, or a transport opened elsewhere).
  Future<void> attach(LinkTransport t) async {
    await _sub?.cancel();
    _transport = t;
    _status = null;
    _sub = t.notifications.listen(_onNotification);
    t.done.then((_) => _onDropped(t));
    _set(LinkState.connected);
    try {
      await t.write(encodeHello());
    } catch (_) {}
    _resendLast();
  }

  void _onDropped(LinkTransport t) {
    if (_transport != t) return;
    if (_wantedId != null && !_disposed) {
      drops++;
      lastDrop = t.dropReason ?? 'unknown reason';
      lastDropAt = DateTime.now();
      _error = 'Connection lost: $lastDrop. Reconnecting…';
    }
    _transport = null;
    _sub?.cancel();
    _sub = null;
    _progress = null;
    if (_wantedId != null && !_disposed) {
      _set(LinkState.connecting);
      _retry = Timer(reconnectDelay, _tryConnect);
    } else {
      _set(LinkState.disconnected);
    }
  }

  /// Disconnects and stops reconnecting (the module stays remembered).
  Future<void> disconnect() async {
    _wantedId = null;
    _wantedName = null;
    await _drop();
    _set(LinkState.disconnected);
  }

  Future<void> _drop() async {
    _retry?.cancel();
    final t = _transport;
    _transport = null;
    await _sub?.cancel();
    _sub = null;
    _progress = null;
    _status = null;
    if (t != null) await t.close();
  }

  // ---- handlebar frames ------------------------------------------------------------

  int _frameNumber = 0;
  (OledFrame, bool)? _last;
  Future<void> _sending = Future.value();
  bool _queued = false;

  /// Frames sent so far (for the app's diagnostics).
  int framesSent = 0;
  int lastFrameBytes = 0;

  /// Sends [frame] to the Main Module, which forwards it to the Front Module.
  /// Frames are sent in order; if several arrive while one is being sent,
  /// only the newest is sent next.
  void sendFrame(OledFrame frame, {bool flash = false}) {
    _last = (frame, flash);
    if (!connected || _queued) return;
    _queued = true;
    _sending = _sending.then((_) async {
      _queued = false;
      final t = _transport, last = _last;
      if (t == null || last == null) return;
      final chunk = math.max(1, math.min(maxChunkData, t.maxWrite - displayHeaderBytes));
      final encoded = encodeDisplayPackets(last.$1, frameNumber: _frameNumber, flash: last.$2, chunkData: chunk);
      _frameNumber = (_frameNumber + 1) & 0xFF;
      try {
        for (final p in encoded.packets) {
          await t.write(p);
        }
        framesSent++;
        lastFrameBytes = encoded.compressedBytes;
      } catch (e) {
        _error = _describe(e);
      }
    });
  }

  void _resendLast() {
    final last = _last;
    if (last != null) sendFrame(last.$1, flash: last.$2);
  }

  /// Waits until every frame handed to [sendFrame] has been written.
  Future<void> flush() => _sending;

  // ---- notifications --------------------------------------------------------------

  void _onNotification(List<int> bytes) {
    final m = decodeMainMessage(bytes);
    switch (m) {
      case ClosePassAlert():
        _alerts.add(m);
      case ModuleStatus():
        _status = m;
        notifyListeners();
      case TransferStart() || TransferData() || TransferEnd():
        _onTransfer(m);
      case UnknownMessage():
        break;
    }
  }

  void _onTransfer(MainMessage m) {
    final result = _receiver.handle(m);
    final id = switch (m) {
      TransferStart(:final transferId) => transferId,
      TransferData(:final transferId) => transferId,
      TransferEnd(:final transferId) => transferId,
      _ => -1,
    };
    final p = _receiver.progress(id);
    if (p != null && (_progress == null || (p - _progress!).abs() >= 0.05 || m is TransferStart)) {
      _progress = p;
      notifyListeners();
    }
    switch (result) {
      case TransferComplete(:final transferId, :final kind, :final payload):
        _progress = null;
        notifyListeners();
        if (kind != transferClosePass) {
          _ack(transferId, ackOk);
          return;
        }
        try {
          final pkg = ClosePassPackage.parse(payload);
          _ack(transferId, ackOk);
          _packages.add(pkg);
        } catch (e) {
          _error = 'Bad close-pass package: $e';
          _ack(transferId, ackResend);
        }
      case TransferFailed(:final transferId, :final reason):
        _progress = null;
        _error = 'Close-pass data incomplete ($reason), asking again';
        notifyListeners();
        _ack(transferId, ackResend);
      case null:
        break;
    }
  }

  void _ack(int transferId, int status) {
    final t = _transport;
    if (t == null) return;
    _sending = _sending.then((_) => t.write(encodeAck(transferId, status)).catchError((_) {}));
  }

  // ---- misc -----------------------------------------------------------------------

  void _set(LinkState s) {
    _state = s;
    if (!_disposed) notifyListeners();
  }

  static String _describe(Object e) {
    final s = e.toString();
    return s.length > 160 ? '${s.substring(0, 160)}…' : s;
  }

  @override
  void dispose() {
    _disposed = true;
    _retry?.cancel();
    _drop();
    _alerts.close();
    _packages.close();
    super.dispose();
  }
}
