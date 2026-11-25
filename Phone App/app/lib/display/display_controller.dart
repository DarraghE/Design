import 'dart:async';

import 'package:bike_navigation/bike_navigation.dart';
import 'package:flutter/foundation.dart';

import '../link/main_module.dart';
import '../link/protocol.dart';
import '../navigation/nav_controller.dart';

/// Why the handlebar shows what it shows.
enum DisplayMode { idle, navigation, closePass }

/// Decides what the handlebar screen shows and sends it to the Main Module:
///
/// - Navigation on: the navigation screen (arrow + distance), nothing else.
///   Close passes are still recorded but don't take over the screen.
/// - Navigation off: "READY", and "CLOSE PASS" in large letters for
///   [alertDuration] after the Main Module reports a close pass.
///
/// The current frame is re-sent every [keepAlive] so a Front Module that
/// restarted catches up.
class DisplayController extends ChangeNotifier {
  final MainModule module;
  final NavController nav;
  final Duration alertDuration;
  final Duration alertFlash;
  final Duration keepAlive;

  DisplayController({
    required this.module,
    required this.nav,
    this.alertDuration = const Duration(seconds: 6),
    this.alertFlash = const Duration(milliseconds: 1500),
    this.keepAlive = const Duration(seconds: 3),
  }) {
    nav.addListener(_update);
    module.addListener(_onModule);
    _keepAlive = Timer.periodic(keepAlive, (_) => _send(force: true));
    _update();
  }

  OledFrame frame = renderIdle();
  bool flash = false;
  DisplayMode mode = DisplayMode.idle;

  ClosePassAlert? _alert;
  bool _alertFlashing = false;
  Timer? _alertTimer, _flashTimer, _keepAlive;
  Uint8List? _sentBytes;
  bool? _sentFlash;
  bool _wasConnected = false;

  /// Shows the close-pass alert, unless navigation is on.
  void showCloseAlert(ClosePassAlert alert) {
    if (nav.active) return;
    _alert = alert;
    _alertFlashing = true;
    _alertTimer?.cancel();
    _flashTimer?.cancel();
    _alertTimer = Timer(alertDuration, () {
      _alert = null;
      _update();
    });
    _flashTimer = Timer(alertFlash, () {
      _alertFlashing = false;
      _update();
    });
    _update();
  }

  void _onModule() {
    if (module.connected && !_wasConnected) _sentBytes = null; // send again after (re)connecting
    _wasConnected = module.connected;
    _send();
  }

  void _update() {
    final s = nav.state;
    if (nav.active) {
      mode = DisplayMode.navigation;
      _alert = null;
      frame = s == null ? renderMessage('ROUTING') : renderOled(s);
      flash = s?.flash ?? false;
    } else if (_alert != null) {
      mode = DisplayMode.closePass;
      frame = renderCloseAlert(distanceM: _alert!.minDistanceM, speedKmh: _alert!.speedKmh);
      flash = _alertFlashing;
    } else {
      mode = DisplayMode.idle;
      frame = renderIdle();
      flash = false;
    }
    _send();
    notifyListeners();
  }

  void _send({bool force = false}) {
    if (!module.connected) return;
    final bytes = frameToRowMajor(frame);
    if (!force && _sentFlash == flash && _sentBytes != null && listEquals(_sentBytes, bytes)) return;
    _sentBytes = bytes;
    _sentFlash = flash;
    module.sendFrame(frame, flash: flash);
  }

  @override
  void dispose() {
    nav.removeListener(_update);
    module.removeListener(_onModule);
    _alertTimer?.cancel();
    _flashTimer?.cancel();
    _keepAlive?.cancel();
    super.dispose();
  }
}
