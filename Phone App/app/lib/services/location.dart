import 'dart:io';

import 'package:bike_navigation/bike_navigation.dart';
import 'package:geolocator/geolocator.dart';

import '../closepass/close_pass_service.dart';

/// Asks for location permission and returns the current position, or null
/// if location is off or permission is refused.
Future<LatLng?> currentLocation() async {
  if (!await Geolocator.isLocationServiceEnabled()) return null;
  var permission = await Geolocator.checkPermission();
  if (permission == LocationPermission.denied) {
    permission = await Geolocator.requestPermission();
  }
  if (permission == LocationPermission.denied || permission == LocationPermission.deniedForever) {
    return null;
  }
  final p = await Geolocator.getCurrentPosition(
    locationSettings: const LocationSettings(accuracy: LocationAccuracy.best, timeLimit: Duration(seconds: 10)),
  );
  return LatLng(p.latitude, p.longitude);
}

/// GPS fixes during a ride, every couple of metres. On Android this runs as
/// a foreground service (with a small "navigating" notification) so
/// navigation keeps going with the screen off and the phone in a pocket.
Stream<Fix> gpsFixes() {
  final LocationSettings settings = Platform.isAndroid
      ? AndroidSettings(
          accuracy: LocationAccuracy.bestForNavigation,
          distanceFilter: 2,
          intervalDuration: const Duration(seconds: 1),
          foregroundNotificationConfig: const ForegroundNotificationConfig(
            notificationTitle: 'Bike Nav is navigating',
            notificationText: 'Directions are being sent to the handlebar.',
            notificationChannelName: 'Navigation',
            enableWakeLock: true,
            setOngoing: true,
          ),
        )
      : const LocationSettings(accuracy: LocationAccuracy.bestForNavigation, distanceFilter: 2);
  return Geolocator.getPositionStream(locationSettings: settings)
      .map((p) => Fix(LatLng(p.latitude, p.longitude), p.timestamp, accuracyM: p.accuracy));
}

/// Asks once for location permission (used by navigation and to record
/// where each close pass happened). Never throws.
Future<void> askForLocation() async {
  try {
    if (await Geolocator.checkPermission() == LocationPermission.denied) {
      await Geolocator.requestPermission();
    }
  } catch (_) {}
}

/// Where the phone is right now, for a close-pass record. Never asks for
/// permission (that would pop up mid-ride); gives up after a few seconds.
Future<Where?> quickWhere() async {
  final permission = await Geolocator.checkPermission();
  if (permission != LocationPermission.always && permission != LocationPermission.whileInUse) return null;
  if (!await Geolocator.isLocationServiceEnabled()) return null;
  try {
    final p = await Geolocator.getCurrentPosition(
      locationSettings: const LocationSettings(accuracy: LocationAccuracy.high, timeLimit: Duration(seconds: 6)),
    );
    return Where(p.latitude, p.longitude, p.accuracy);
  } catch (_) {
    final p = await Geolocator.getLastKnownPosition();
    return p == null ? null : Where(p.latitude, p.longitude, p.accuracy);
  }
}
