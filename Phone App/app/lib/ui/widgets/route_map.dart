import 'package:bike_navigation/bike_navigation.dart' as nav;
import 'package:flutter_map/flutter_map.dart';
import 'package:latlong2/latlong.dart' as ll;
import 'package:material_ui/material_ui.dart';

import '../theme.dart';

ll.LatLng _ll(nav.LatLng p) => ll.LatLng(p.lat, p.lng);

/// Dark OpenStreetMap map with the route, destination and rider. No API key
/// needed. Set [tiles] false in tests (no network).
class RouteMap extends StatefulWidget {
  final nav.LatLng center;
  final nav.NavRoute? route;
  final nav.LatLng? destination;
  final nav.LatLng? rider;
  final bool follow;
  final bool tiles;
  final void Function(nav.LatLng)? onLongPress;

  const RouteMap({
    super.key,
    required this.center,
    this.route,
    this.destination,
    this.rider,
    this.follow = false,
    this.tiles = true,
    this.onLongPress,
  });

  @override
  State<RouteMap> createState() => _RouteMapState();
}

class _RouteMapState extends State<RouteMap> {
  final _controller = MapController();
  bool _ready = false;

  @override
  void didUpdateWidget(RouteMap old) {
    super.didUpdateWidget(old);
    if (!_ready) return;
    if (widget.follow && widget.rider != null && widget.rider != old.rider) {
      _controller.move(_ll(widget.rider!), _controller.camera.zoom < 15 ? 16.5 : _controller.camera.zoom);
    } else if (widget.route != old.route && widget.route != null) {
      _fit();
    } else if (widget.route == null && widget.center != old.center) {
      _controller.move(_ll(widget.center), 14);
    }
  }

  void _fit() {
    final pts = widget.route!.line.points.map(_ll).toList();
    if (pts.length < 2) return;
    _controller.fitCamera(CameraFit.coordinates(coordinates: pts, padding: const EdgeInsets.fromLTRB(40, 140, 40, 220)));
  }

  @override
  Widget build(BuildContext context) {
    final route = widget.route;
    return FlutterMap(
      mapController: _controller,
      options: MapOptions(
        initialCenter: _ll(widget.center),
        initialZoom: 14,
        backgroundColor: const Color(0xFF0E141B),
        onMapReady: () {
          _ready = true;
          if (route != null) _fit();
        },
        onLongPress: widget.onLongPress == null ? null : (_, p) => widget.onLongPress!(nav.LatLng(p.latitude, p.longitude)),
      ),
      children: [
        if (widget.tiles)
          // Standard OpenStreetMap tiles, darkened on the phone. No key.
          // (CARTO's dark tiles needed a key from late 2026.) Light use like
          // this is within the OSM tile policy as long as the app is named
          // in the user agent and the attribution is shown.
          TileLayer(
            urlTemplate: 'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
            userAgentPackageName: 'ie.tcd.design.bike_app',
            tileBuilder: darkModeTileBuilder,
            maxZoom: 19,
          ),
        if (route != null)
          PolylineLayer(polylines: [
            Polyline(points: route.line.points.map(_ll).toList(), strokeWidth: 9, color: Palette.accent.withValues(alpha: 0.25)),
            Polyline(points: route.line.points.map(_ll).toList(), strokeWidth: 4.5, color: Palette.accent),
          ]),
        MarkerLayer(markers: [
          if (widget.destination != null)
            Marker(
              point: _ll(widget.destination!),
              width: 36,
              height: 36,
              alignment: Alignment.topCenter,
              child: const Icon(Icons.location_on, color: Palette.danger, size: 36),
            ),
          if (widget.rider != null)
            Marker(
              point: _ll(widget.rider!),
              width: 26,
              height: 26,
              child: Container(
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: Palette.accent,
                  border: Border.all(color: Colors.white, width: 3),
                  boxShadow: [BoxShadow(color: Palette.accent.withValues(alpha: 0.6), blurRadius: 14)],
                ),
              ),
            ),
        ]),
        if (widget.tiles)
          const Align(
            alignment: Alignment.bottomRight,
            child: Padding(
              padding: EdgeInsets.all(6),
              child: Text('© OpenStreetMap contributors', style: TextStyle(fontSize: 10, color: Palette.muted)),
            ),
          ),
      ],
    );
  }
}
