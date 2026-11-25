# Bike Nav (Android app)

The phone app for the bike: **Navigation** (turn-by-turn screens sent to the handlebar) and **Close Pass** (alerts, photos, LiDAR replay, number plates, website upload).

**Everything about it (what it does, the Bluetooth protocol, install steps, tests and what the ESP32s and website still need) is in [`../SPECIFICATION.md`](../SPECIFICATION.md).**

## Quick start

- **Install on a phone:** `../release/BikeNav.apk` (see the spec, section 6).
- **Keys:** none are built in. Enter the OpenRouteService key, OpenAI key and website address in the app's **Settings**.
- **Build:** Flutter 3.50.1, then
  ```bash
  flutter pub get
  flutter test
  flutter build apk --release --target-platform android-arm64
  ```
  The APK is written to `build/app/outputs/flutter-apk/app-release.apk`.
- **No hardware?** In the app: Home → Main Module → **Use the demo module**. Everything then works with a simulated Main Module.
