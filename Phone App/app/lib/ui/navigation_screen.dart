import 'dart:async';

import 'package:bike_navigation/bike_navigation.dart';
import 'package:material_ui/material_ui.dart';

import '../app.dart';
import '../services/location.dart';
import '../services/place_search.dart';
import 'theme.dart';
import 'widgets/lcd_preview.dart';
import 'widgets/route_map.dart';

/// Where the map starts when GPS isn't available yet.
const trinityCollege = LatLng(53.3438, -6.2546);

/// Navigation mode: pick a destination, check the route, start. While
/// riding, the screen mirrors the handlebar and shows the next instruction.
class NavigationScreen extends StatefulWidget {
  /// The map needs the network; tests turn the tiles off.
  final bool mapTiles;
  const NavigationScreen({super.key, this.mapTiles = true});

  @override
  State<NavigationScreen> createState() => _NavigationScreenState();
}

class _NavigationScreenState extends State<NavigationScreen> {
  final _query = TextEditingController();
  final _focus = FocusNode();
  Timer? _debounce;
  List<Place> _results = const [];
  LatLng _here = trinityCollege;
  bool _haveGps = false;
  String? _searchError;
  bool _located = false;

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    if (!_located) {
      _located = true;
      _locate();
    }
  }

  Future<void> _locate() async {
    try {
      final p = await currentLocation();
      if (!mounted || p == null) return;
      setState(() {
        _here = p;
        _haveGps = true;
      });
    } catch (_) {}
  }

  void _onQuery(String text) {
    _debounce?.cancel();
    final app = AppScope.of(context);
    _debounce = Timer(const Duration(milliseconds: 350), () async {
      if (!app.settings.hasOrsKey) {
        setState(() => _searchError = 'Add your OpenRouteService key in Settings to search.');
        return;
      }
      try {
        final r = await PlaceSearch(app.settings.orsKey).search(text, near: _here);
        if (mounted) {
          setState(() {
            _results = r;
            _searchError = null;
          });
        }
      } catch (e) {
        if (mounted) setState(() => _searchError = 'Search failed. Check the internet connection.');
      }
    });
  }

  Future<void> _choose(Place p) async {
    _focus.unfocus();
    setState(() {
      _results = const [];
      _query.text = p.label;
    });
    await AppScope.of(context).nav.choose(p, _here);
  }

  @override
  void dispose() {
    _debounce?.cancel();
    _query.dispose();
    _focus.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final app = AppScope.of(context);
    return ListenableBuilder(
      listenable: Listenable.merge([app.nav, app.display, app.module]),
      builder: (context, _) => app.nav.active ? _riding(app) : _planning(app),
    );
  }

  // ---- planning -------------------------------------------------------------------

  Widget _planning(AppServices app) {
    final nav = app.nav;
    return Scaffold(
      body: Stack(children: [
        Positioned.fill(
          child: RouteMap(
            center: _here,
            route: nav.preview,
            destination: nav.destination?.position,
            rider: _haveGps ? _here : null,
            tiles: widget.mapTiles,
            onLongPress: (p) => _choose(Place('Dropped pin', p)),
          ),
        ),
        SafeArea(
          child: Padding(
            padding: const EdgeInsets.all(14),
            child: Column(children: [
              Row(children: [
                IconButton.filledTonal(onPressed: () => Navigator.of(context).pop(), icon: const Icon(Icons.arrow_back)),
                const SizedBox(width: 10),
                Expanded(
                  child: TextField(
                    controller: _query,
                    focusNode: _focus,
                    onChanged: _onQuery,
                    textInputAction: TextInputAction.search,
                    decoration: InputDecoration(
                      hintText: 'Where to?',
                      prefixIcon: const Icon(Icons.search),
                      suffixIcon: _query.text.isEmpty
                          ? null
                          : IconButton(
                              icon: const Icon(Icons.close),
                              onPressed: () {
                                _query.clear();
                                setState(() => _results = const []);
                                nav.clearDestination();
                              },
                            ),
                    ),
                  ),
                ),
              ]),
              if (_results.isNotEmpty || _searchError != null)
                Card(
                  margin: const EdgeInsets.only(top: 8),
                  child: Column(children: [
                    if (_searchError != null)
                      ListTile(leading: const Icon(Icons.info_outline, color: Palette.warning), title: Text(_searchError!)),
                    for (final p in _results.take(5))
                      ListTile(
                        leading: const Icon(Icons.place_outlined, color: Palette.accent),
                        title: Text(p.label, maxLines: 2, overflow: TextOverflow.ellipsis),
                        onTap: () => _choose(p),
                      ),
                  ]),
                ),
            ]),
          ),
        ),
        if (nav.destination != null)
          Align(
            alignment: Alignment.bottomCenter,
            child: SafeArea(child: Padding(padding: const EdgeInsets.all(14), child: _routeCard(app))),
          )
        else
          const Align(
            alignment: Alignment.bottomCenter,
            child: SafeArea(
              child: Padding(
                padding: EdgeInsets.all(20),
                child: Text('Search for a place, or long-press the map.', style: TextStyle(color: Palette.muted)),
              ),
            ),
          ),
      ]),
    );
  }

  Widget _routeCard(AppServices app) {
    final nav = app.nav;
    final r = nav.preview;
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(18),
        child: Column(mainAxisSize: MainAxisSize.min, crossAxisAlignment: CrossAxisAlignment.stretch, children: [
          Text(nav.destination!.label, maxLines: 2, overflow: TextOverflow.ellipsis, style: const TextStyle(fontSize: 17, fontWeight: FontWeight.w700)),
          const SizedBox(height: 10),
          if (nav.loadingRoute)
            const Row(children: [
              SizedBox(width: 18, height: 18, child: CircularProgressIndicator(strokeWidth: 2)),
              SizedBox(width: 12),
              Text('Finding a cycling route…', style: TextStyle(color: Palette.muted)),
            ])
          else if (nav.planError != null)
            Text(nav.planError!, style: const TextStyle(color: Palette.danger))
          else if (r != null)
            Row(children: [
              _Stat(formatKm(r.distanceM), 'distance'),
              _Stat('${(r.durationS / 60).round()} min', 'by bike'),
              _Stat('${r.manoeuvres.where((m) => m.kind != ManoeuvreKind.arrive).length}', 'turns'),
            ]),
          const SizedBox(height: 14),
          FilledButton.icon(
            onPressed: r == null ? null : () => nav.start(_here),
            icon: const Icon(Icons.navigation_rounded),
            label: const Text('Start navigation'),
          ),
          TextButton(
            onPressed: r == null ? null : () => nav.start(r.line.points.first, simulate: true),
            child: const Text('Simulate the ride (demo)'),
          ),
        ]),
      ),
    );
  }

  // ---- riding ---------------------------------------------------------------------

  Widget _riding(AppServices app) {
    final nav = app.nav;
    final s = nav.state;
    final instruction = s == null
        ? (nav.rideError ?? 'Getting the route…')
        : s.arrived
            ? 'You have arrived'
            : s.offRoute
                ? 'Off route, finding a new one…'
                : (s.next?.instruction.isNotEmpty ?? false)
                    ? s.next!.instruction
                    : 'Continue';
    return Scaffold(
      backgroundColor: Palette.background,
      body: SafeArea(
        child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
          Padding(
            padding: const EdgeInsets.fromLTRB(8, 4, 8, 0),
            child: Row(children: [
              IconButton(onPressed: () => Navigator.of(context).pop(), icon: const Icon(Icons.arrow_back)),
              Expanded(
                child: Text(nav.destination?.label ?? 'Navigation',
                    maxLines: 1, overflow: TextOverflow.ellipsis, style: const TextStyle(fontSize: 17, fontWeight: FontWeight.w700)),
              ),
              if (nav.simulated)
                const Padding(padding: EdgeInsets.only(right: 8), child: Chip(label: Text('Simulated'))),
            ]),
          ),
          Padding(
            padding: const EdgeInsets.fromLTRB(20, 8, 20, 0),
            child: LcdPreview(frame: app.display.frame, flash: app.display.flash),
          ),
          Padding(
            padding: const EdgeInsets.fromLTRB(22, 16, 22, 4),
            child: Text(instruction, maxLines: 2, overflow: TextOverflow.ellipsis, style: const TextStyle(fontSize: 22, fontWeight: FontWeight.w700, height: 1.2)),
          ),
          if (s?.then != null)
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 22),
              child: Text('then ${s!.then!.instruction}', style: const TextStyle(color: Palette.muted, fontSize: 15)),
            ),
          Padding(
            padding: const EdgeInsets.fromLTRB(22, 6, 22, 10),
            child: Text(
              app.module.connected ? 'Sending to the handlebar' : 'Main Module not connected: preview only',
              style: TextStyle(color: app.module.connected ? Palette.ok : Palette.warning, fontSize: 13),
            ),
          ),
          Expanded(
            child: Padding(
              padding: const EdgeInsets.symmetric(horizontal: 14),
              child: ClipRRect(
                borderRadius: BorderRadius.circular(22),
                child: RouteMap(
                  center: nav.rider ?? _here,
                  route: nav.route,
                  destination: nav.destination?.position,
                  rider: nav.rider,
                  follow: true,
                  tiles: widget.mapTiles,
                ),
              ),
            ),
          ),
          Padding(
            padding: const EdgeInsets.all(14),
            child: FilledButton(
              style: FilledButton.styleFrom(backgroundColor: Palette.surfaceHigh, foregroundColor: Palette.text),
              onPressed: () => nav.stop(),
              child: Text(nav.arrived ? 'Done' : 'End navigation'),
            ),
          ),
        ]),
      ),
    );
  }
}

class _Stat extends StatelessWidget {
  final String value;
  final String label;
  const _Stat(this.value, this.label);

  @override
  Widget build(BuildContext context) => Expanded(
        child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
          Text(value, style: const TextStyle(fontSize: 22, fontWeight: FontWeight.w800)),
          Text(label, style: const TextStyle(color: Palette.muted, fontSize: 12)),
        ]),
      );
}

String formatKm(double m) => m < 1000 ? '${m.round()} m' : '${(m / 1000).toStringAsFixed(1)} km';
