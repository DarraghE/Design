# Phone App: Specification

| | |
|---|---|
| **Module** | Phone App |
| **Runs on** | Android phone |
| **Language** | Dart / Flutter 3.50+ |
| **Status** | Working: destination search, route preview, turn-by-turn navigation, rerouting, handlebar-screen preview, simulated rides. **Not yet done:** Bluetooth link to the modules. |
| **Source** | Copied from `spidermannn3000/Design-group`, commit `7fd6266` (22 Nov 2026) |

## 1. What this module does

The phone is the brain of the bike. The rider picks a destination. The phone gets a cycling route from OpenRouteService and follows the rider by GPS. After every GPS fix it works out:

- which turn comes next,
- how far away that turn is, measured **along the route**,
- whether to show the turn yet,
- whether the handlebar screen should flash (turn 5 m away or less),
- whether the rider has gone off the route, in which case it gets a new route.

It turns that into a few bytes (**symbol IDs + distance + flags**) for the Front Module's 128x64 OLED. Until the Bluetooth link is built, the app shows a pixel-exact preview of the handlebar screen on the phone instead.

```
 OpenRouteService ──route──▶  Phone App  ──Bluetooth LE (not built yet)──▶  Front Module
 Google Maps ──────map─────▶  (GPS + navigation)                            arrow + distance
```

## 2. Folder layout

```
Phone App/
├── SPECIFICATION.md        ← this file
├── app/                    ← Flutter Android app (screens, GPS, map)
│   ├── lib/
│   │   ├── main.dart                     App entry point
│   │   ├── config.dart                   ORS key (build-time), fallback start point, simulation speed
│   │   ├── screens/destination_screen.dart   Search, route preview, Start / Simulate buttons
│   │   ├── screens/ride_screen.dart          Navigation during a ride
│   │   ├── services/front_module.dart        Interface to the handlebar module (Bluetooth goes here)
│   │   ├── services/location.dart            Location permission + GPS stream
│   │   ├── services/place_search.dart        Place search (OpenRouteService, Ireland only)
│   │   ├── widgets/oled_display.dart         Pixel-exact preview of the handlebar OLED
│   │   └── widgets/route_map.dart            Google map with route, destination and rider
│   ├── android/                          Android project (permissions, Maps key hook-up)
│   └── test/app_test.dart                Place-search parsing + a full simulated Dublin ride
├── navigation/             ← Plain Dart package "bike_navigation" (no Flutter, testable on a laptop)
│   ├── lib/src/
│   │   ├── ors_client.dart    Requests a cycling route, converts ORS steps to manoeuvres
│   │   ├── route.dart         Data types: ManoeuvreKind, Manoeuvre, NavRoute, RouteProvider
│   │   ├── geo.dart           LatLng, distances, snapping a GPS fix onto the route line
│   │   ├── engine.dart        NavigationEngine: GPS fix → NavState (the core logic)
│   │   ├── session.dart       NavigationSession: one ride, with automatic rerouting
│   │   ├── junctions.dart     Side-road junctions from OpenStreetMap (Overpass API), Level 3
│   │   ├── symbols.dart       Manoeuvre → display symbol ID table
│   │   ├── oled.dart          Draws every 128x64 handlebar screen (renderOled)
│   │   ├── protocol.dart      DRAFT Bluetooth packet format (encodeNavPacket)
│   │   └── replay.dart        Replays a GPX track or a simulated noisy ride
│   ├── bin/replay.dart        Command-line replay tool
│   ├── tool/                  Export the OLED screens as images
│   └── test/                  Unit tests + saved ORS routes (incl. Trinity → St Patrick's)
└── docs/
    ├── navigation.md          Full design notes: provider comparison, levels, screen design
    └── images/oled_screens.png
```

The app uses the navigation package through `path: ../navigation` in `app/pubspec.yaml`, so keep the two folders side by side.

## 3. Features, by task ID

| ID | Feature | Status | Where |
|----|---------|--------|-------|
| N1 | Routing provider chosen: OpenRouteService, profile `cycling-regular` | Done | `ors_client.dart`, `docs/navigation.md` |
| N2 | Level 1: always show the next turn | Done (**this is what the app uses now**) | `engine.dart` `TriggerMode.always` |
| N3 | Level 2: show the turn from 110 m before it (100 m + 10 m GPS margin) | Done in the package, not selected in the app | `TriggerMode.distance` |
| N4 | Level 3: show the turn once the rider passes the last side road before it | Done in the package, not wired into the app | `TriggerMode.junction`, `junctions.dart` |
| N5 | Off-route detection and automatic rerouting | Done | `engine.dart`, `session.dart` |
| N6 | Manoeuvre → display symbol ID table | Done | `symbols.dart` |
| N7 | Replay a recorded or simulated ride for testing | Done | `replay.dart`, `bin/replay.dart` |
| F2 | Handlebar screen drawing (shared with Front Module) | Done on the phone side | `oled.dart`, `oled_display.dart` |
| P3 | Destination entry: search or long-press the map, see route, distance, time | Done | `destination_screen.dart` |
| P4/P5 | Bluetooth LE: find, connect to and send to the Front Module | **Not started.** The interface and packet format are ready | `front_module.dart`, `protocol.dart` |
| P6 | Keep navigating with the phone screen off | **Not started** | |
| P2 | Designed wireframes | **Not started.** The current layout works but isn't designed | |

## 4. How it behaves

### 4.1 Screens

1. **Destination screen.** A full-screen Google map with a "Where to?" search box. Typing at least 3 letters searches places in Ireland, ranked by how close they are to the rider (400 ms debounce). Picking a result or long-pressing the map requests a route and shows **distance (km) · time (min) · number of turns**. Two buttons: **Simulate** and **Start ride**.
   - With location off or refused, the start point is **Trinity College, Dublin** (53.3438, -6.2546).
   - With no ORS key, a card says to run with `--dart-define=ORS_API_KEY=...`.
2. **Ride screen.** At the top is the OLED preview, twice real size and blue, with the current instruction text (and "then …" for a close second turn) below it. "Off route, finding a new route…" appears when off route. The map follows the rider. At the bottom: a reroute count and an **End ride** / **Done** button.
   - **Simulate** rides the route automatically at 4x speed with 5 m of GPS noise, for testing indoors.

### 4.2 Navigation rules (all distances measured along the route)

| Rule | Value | Config field (`NavConfig`) |
|------|-------|----------------------------|
| Default trigger level (package) | Level 2 (distance) | `mode` |
| Trigger level used by the app | Level 1 (always) | set in `destination_screen.dart` |
| Show turn from | 100 m + 10 m GPS margin | `triggerDistanceM`, `gpsMarginM` |
| Once shown, a turn stays shown | Even if GPS jitters backwards | |
| Move on to the next turn | 10 m past the turn, or halfway to the next turn if that is sooner | `passMarginM` |
| Close turns shown as a pair | Next turn 40 m or less after this one | `closeTurnM` |
| Flash the screen | Turn is 5 m or less away (not for the destination) | `flashDistanceM` |
| Off route | More than 30 m (+ GPS accuracy, capped at 20 m) from the route for 3 fixes in a row | `offRouteM`, `offRouteFixes` |
| Reroute rate limit | At most once every 15 s | `NavigationSession.rerouteCooldown` |
| Arrived | Within 20 m (2 × GPS margin) of the destination | |
| Snap search window | 400 m ahead of the last position | `searchAheadM` |
| Level 3 lead | Show 5 m before the last junction (+ GPS margin). Falls back to Level 2 if there's no junction or no Overpass data | `junctionLeadM` |

### 4.3 Key data types

```dart
enum ManoeuvreKind { straight, left, right, slightLeft, slightRight, sharpLeft, sharpRight, uTurn, roundabout, arrive }

class NavState {            // returned after every GPS fix
  Manoeuvre? next;          // the coming turn (null once arrived)
  Manoeuvre? then;          // a second turn within 40 m of `next`
  double distanceToNextM;   // along the route
  bool showTurn;            // false = show "straight" for now
  List<int> symbols;        // display symbol IDs to send, in order
  bool offRoute, arrived, flash;
  double alongM, offsetM;   // progress along the route, distance from it
}
```

## 5. Interface to the Front Module (draft, being replaced)

> **Planned change (team decision, 25 Nov 2026):** the phone will draw each navigation screen as an image and send it over Bluetooth to a "main module", which passes it straight on to the Front Module. The Front Module then just shows images. This replaces the symbol-ID design below. **The phone code still implements the design below**, and no Bluetooth code exists yet. The open questions (which board is the main module, the link, the frame bit order) are listed in `Front Module/SPECIFICATION.md` section 1.2. `renderOled` already draws every screen as a 128 × 64 frame, and `OledFrame.toSsd1306()` gives the 1024 bytes in the SSD1306's page layout. The Front Module's current image receiver expects row-by-row bytes instead, so one side will need converting.

The rest of this section is the earlier draft, kept because it is what `symbols.dart`, `protocol.dart` and `oled.dart` implement today.

### 5.1 Display symbol IDs

| ID | Screen | ID | Screen |
|----|--------|----|--------|
| 0 | Blank | 8 | U-turn |
| 1 | Straight on | 9 | Roundabout (followed by exit ID) |
| 2 | Left (90°) | 10 | Arrived |
| 3 | Right (90°) | 11 | Rerouting |
| 4 | Slight left (45°) / keep left | 12 | Bluetooth lost (module sets this itself) |
| 5 | Slight right (45°) / keep right | 13 | Low battery (module sets this itself) |
| 6 | Sharp left (135°) | 21–29 | Roundabout exit 1–9 |
| 7 | Sharp right (135°) | | |

Examples: roundabout, 2nd exit = `[9, 22]`. Left then right within 40 m = `[2, 3]`. The OLED shows only the first turn of a pair; the app shows both.

### 5.2 Bluetooth LE packet (DRAFT v1, `protocol.dart`)

- The Front Module is a BLE **peripheral** advertising service `6e3b0001-8f7a-4c1e-9b52-2d1c0b7a1f10`.
- The phone **writes** each update to characteristic `6e3b0002-8f7a-4c1e-9b52-2d1c0b7a1f10`.

| Byte | Meaning |
|------|---------|
| 0 | Protocol version = `1` |
| 1 | Flags: bit 0 = flash, bit 1 = arrived |
| 2–3 | Distance to turn in metres, little-endian. `0xFFFF` = none (off route, arrived or no turn) |
| 4 | Number of symbol IDs that follow, 0–4 (0 = blank screen) |
| 5… | Symbol IDs |

Clearing the screen sends `[1, 0, 0xFF, 0xFF, 0]`.

### 5.3 Screen layout the module must draw

- 0.96" **128x64 SSD1306** OLED, I2C address `0x3C`. Buy the **Blue or White** version, not Yellow/Blue. The team's latest stated wiring is SDA GPIO15 / SCL GPIO22. Design-group's firmware used SDA GPIO21. See `Front Module/SPECIFICATION.md` section 4.
- **Left 57x64 area:** the arrow, scaled to fill the area. Its angle shows how sharp the turn is: straight 0°, slight 45°, normal 90°, sharp 135°. U-turns go round to the right. The roundabout is a clockwise circle with the exit number inside.
- **Right:** distance, rounded like a bike computer: `87 m`, `120 m` (to 10 m above 100 m), `2.3 km`.
- No text on turn screens. Only the arrived, rerouting, no-phone and battery screens have words.
- **Flash:** invert the whole screen 4 times a second while the flash flag is set (SSD1306 invert command).
- `navigation/lib/src/oled.dart` (`renderOled`) is the reference drawing. The firmware should draw exactly the same pixels. Design-group also has a C++ port of this drawing code (`front-module/src/screen.cpp`), which can be reused when the Front Module is written.

## 6. Build, run and test

**Needs:** Flutter 3.50+, an Android phone with USB debugging (or the emulator), and two keys. **The keys are never committed.**

| Key | Used for | Where it goes |
|-----|----------|---------------|
| OpenRouteService (free at openrouteservice.org) | Routes and place search | `--dart-define=ORS_API_KEY=...` at build time |
| Google Maps, with Maps SDK for Android enabled | Drawing the map | `Phone App/app/android/local.properties` → `MAPS_API_KEY=...` (git ignores this file) |

```powershell
cd "Phone App/app"
flutter pub get
flutter run --dart-define=ORS_API_KEY=your_key
```

```powershell
flutter build apk --dart-define=ORS_API_KEY=your_key   # → build/app/outputs/flutter-apk/app-release.apk
```

Tests:

```powershell
cd "Phone App/navigation"; dart test          # navigation + OLED + packet tests
cd "Phone App/app"; flutter test              # place search + simulated Dublin ride
cd "Phone App/navigation"; dart run bin/replay.dart --route test/fixtures/dublin_trinity_to_st_patricks.json
```

Android permissions: `INTERNET`, `ACCESS_FINE_LOCATION`, `ACCESS_COARSE_LOCATION`. App ID: `ie.tcd.design.bike_app`.

## 7. Known gaps and next steps

1. **Bluetooth (P4/P5):** implement `FrontModule` in `app/lib/services/front_module.dart` with a BLE plugin and send `encodeNavPacket(state)` on every update. The app currently uses `PreviewFrontModule`, which only stores the last state for the on-screen preview.
2. **Bluetooth permissions:** `BLUETOOTH_SCAN` and `BLUETOOTH_CONNECT` need adding to `AndroidManifest.xml` with P4.
3. **Background navigation (P6):** navigation currently stops being reliable with the screen off.
4. **Trigger level:** the app uses Level 1. To use Level 2 or 3, change `NavConfig(mode: ...)` in `destination_screen.dart`. Level 3 also needs `prepareRoute: (r) => addJunctions(r)`.
5. **Untested against live services:** a route with a roundabout (to check `exit_number`), and the live Overpass junction query.
6. **Tuning:** record a real ride as GPX and replay it at all three levels to tune the 100 m, 10 m and 30 m values.
7. **Rear and Camera Modules:** both now have firmware (LiDAR detection, warning lights, close-pass photos), but they talk to a Mac over USB, not to the phone. There's no phone-side code or protocol for them yet. If the Rear Module becomes the "main module", the phone link would also carry the navigation images.
8. **Image-based navigation (planned):** see the note at the top of section 5.

## 8. Notes for AI assistants

- The core logic is `NavigationEngine.update()` in `navigation/lib/src/engine.dart`. Read that first.
- `navigation/` must stay free of Flutter imports so `dart test` runs on a laptop.
- Symbol IDs, packet bytes and the screen layout are a **contract with the Front Module**. If you change one, update `symbols.dart`, `protocol.dart`, `oled.dart`, section 5 of this file and the Front Module's code together.
- Never put API keys in source files.
