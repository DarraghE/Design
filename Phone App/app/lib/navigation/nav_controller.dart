import 'dart:async';

import 'package:bike_navigation/bike_navigation.dart';
import 'package:flutter/foundation.dart';

import '../services/place_search.dart';

/// GPS fixes during a ride (the phone's GPS, or a simulated ride).
typedef FixSource = Stream<Fix> Function();

/// Navigation mode: choose a destination, preview the cycling route, then
/// ride. While riding, the phone's GPS drives the existing navigation engine
/// and the only output is [state], which the display controller turns into
/// the handlebar screen.
class NavController extends ChangeNotifier {
  /// Gets a cycling route (OpenRouteService in the app, a stub in tests).
  final RouteProvider Function() routeProvider;
  final FixSource gps;

  /// How long between simulated GPS fixes (real time is 1 s; 4x faster by default).
  final Duration simulationInterval;

  NavController({
    required this.routeProvider,
    required this.gps,
    this.simulationInterval = const Duration(milliseconds: 250),
  });

  // ---- planning ------------------------------------------------------------------
  Place? destination;
  NavRoute? preview;
  bool loadingRoute = false;
  String? planError;

  Future<void> choose(Place place, LatLng from) async {
    destination = place;
    preview = null;
    planError = null;
    loadingRoute = true;
    notifyListeners();
    try {
      preview = await routeProvider()(from, place.position);
    } catch (e) {
      planError = friendlyError(e);
    } finally {
      loadingRoute = false;
      notifyListeners();
    }
  }

  void clearDestination() {
    destination = null;
    preview = null;
    planError = null;
    notifyListeners();
  }

  // ---- riding --------------------------------------------------------------------
  bool active = false;
  bool simulated = false;
  NavState? state;
  LatLng? rider;
  String? rideError;
  NavigationSession? _session;
  StreamSubscription<NavState>? _sub;

  NavRoute? get route => _session?.route ?? preview;
  int get rerouteCount => _session?.rerouteCount ?? 0;
  bool get arrived => state?.arrived ?? false;

  /// Starts navigating to [destination] from [from].
  Future<void> start(LatLng from, {bool simulate = false}) async {
    final dest = destination;
    if (dest == null) return;
    await stop(notify: false);
    active = true;
    simulated = simulate;
    state = null;
    rideError = null;
    rider = from;
    notifyListeners();

    final planned = preview;
    final provider = routeProvider();
    var first = true;
    final session = NavigationSession(
      // Use the previewed route first, so starting is instant.
      routeProvider: (a, b) async {
        if (first && planned != null) {
          first = false;
          return planned;
        }
        first = false;
        return provider(a, b);
      },
      destination: dest.position,
      config: const NavConfig(mode: TriggerMode.always),
    );
    _session = session;
    try {
      await session.start(from);
    } catch (e) {
      if (_session != session) return;
      rideError = friendlyError(e);
      active = false;
      notifyListeners();
      return;
    }
    if (_session != session) return;
    final fixes = simulate ? _simulated(session.route!) : gps();
    _sub = fixes.asyncMap((f) async {
      rider = f.position;
      return session.onPosition(f.position, accuracyM: f.accuracyM, at: f.time);
    }).listen((s) {
      if (_session != session) return;
      state = s;
      notifyListeners();
      if (s.arrived) _sub?.cancel();
    }, onError: (Object e) {
      rideError = friendlyError(e);
      notifyListeners();
    });
  }

  Stream<Fix> _simulated(NavRoute r) async* {
    for (final fix in simulateRide(r.line.points, noiseM: 4)) {
      await Future<void>.delayed(simulationInterval);
      yield fix;
    }
  }

  Future<void> stop({bool notify = true}) async {
    await _sub?.cancel();
    _sub = null;
    _session = null;
    active = false;
    state = null;
    if (notify) notifyListeners();
  }

  @override
  void dispose() {
    _sub?.cancel();
    super.dispose();
  }
}

/// Short, human error text for network and API problems.
String friendlyError(Object e) {
  final s = e.toString();
  if (s.contains('OpenRouteService error 401') || s.contains('OpenRouteService error 403')) {
    return 'OpenRouteService rejected the key. Check it in Settings.';
  }
  if (s.contains('OpenRouteService error 404') || s.contains('2010') || s.contains('2009')) {
    return 'No cycling route found to there.';
  }
  if (s.contains('SocketException') || s.contains('ClientException') || s.contains('Failed host lookup')) {
    return 'No internet connection.';
  }
  if (s.contains('No OpenRouteService key')) return s;
  return s.length > 140 ? '${s.substring(0, 140)}…' : s;
}
