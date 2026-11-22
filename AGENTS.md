# AGENTS.md: How to work in this repository

These are the rules for any AI agent (or person) adding or changing code here. Read this file first, then read the `SPECIFICATION.md` in the folder you're working in.

## What this project is

Bike Nav: a cycling navigation and safety system. An Android phone works out the route and talks over Bluetooth LE to ESP32 modules mounted on the bike.

## Repository layout (keep it exactly like this)

```
README.md              Overview table of all modules and their status
AGENTS.md              This file
Phone App/             Android app (Flutter) + navigation package (plain Dart)
  SPECIFICATION.md
  app/                 Flutter app
  navigation/          bike_navigation package (no Flutter imports)
  docs/                Design notes and images
Front Module/          ESP32 handlebar display (128x64 SSD1306 OLED)
  SPECIFICATION.md
  front_module.ino
Camera Module/         ESP32 camera
  SPECIFICATION.md
  camera_module.ino
Rear Module/           ESP32 rear unit (lidar, lights)
  SPECIFICATION.md
  rear_module.ino
Archive/               Old repo contents. Read-only
```

- **Don't add new top-level folders** or rename the four module folders without being asked.
- **Don't edit, move or delete anything in `Archive/`.** It is kept only for reference.

## Rule 1: Every code change updates the specification

Whenever you add or change code in a module folder, update that folder's `SPECIFICATION.md` **in the same commit**. The spec must always describe what the code **actually does now**, not what is planned.

Update every section that the change affects:

| Section | What to keep current |
|---------|----------------------|
| Header table | **Status** line (Not started / In progress / Working, plus what's missing) |
| 1. What this module does | Plain-English description of the current behaviour |
| 2. Interface | Bluetooth service/characteristic UUIDs, message formats, anything another module depends on |
| 3. Features | One row per feature: ID (if the team has one), feature, status, where in the code |
| 4. Hardware and wiring | Every part, pin and ESP32 pin. I2C addresses. Which part version to buy |
| 5. Build and upload | Board, libraries with versions, any settings needed |
| 6. Known gaps and next steps | What is unfinished, untested or known to be wrong |
| 7. Notes for AI assistants | Anything an agent must know to change this code safely |

Also update the **status column in the root `README.md`** when a module's status changes.

### Writing style for specs (must be readable by humans and AI)

- Plain English, short sentences, no marketing language.
- Prefer tables for facts (pins, IDs, bytes, parameters, statuses).
- Give exact values: numbers with units (`5 m`, `115200 baud`, `0x3C`), exact file and function names in backticks.
- Say clearly what is **done**, what is **not done** and what is **untested**. Never describe planned work as if it exists.
- Keep the same section numbers and headings in every module's spec so they can be compared side by side.

## Rule 2: One `.ino` file per ESP32 module

The Front, Camera and Rear Modules are flashed straight onto ESP32s, so each must stay **one single Arduino sketch** that can be copied and uploaded from the Arduino IDE:

- All firmware for a module goes in its `.ino` file (`front_module.ino`, `camera_module.ino`, `rear_module.ino`). No extra `.cpp`/`.h` files, no PlatformIO projects, no sub-folders.
- The top of the file has a comment block giving: the module name, a one-line purpose, the board to select in the Arduino IDE, and **every library needed with its version** (as named in the Arduino Library Manager).
- Put pin numbers, I2C addresses, UUIDs and tunable values as named constants near the top of the file. The spec's wiring table must match them.
- It must compile for the ESP32 with only the listed libraries installed.

The Phone App isn't limited to one file. Keep its existing structure (`app/` and `navigation/`).

## Rule 3: Keep the module contracts in sync

Modules depend on each other through shared numbers. If you change one side, change the other side **and both specs** in the same commit.

| Contract | Defined in | Must match |
|----------|-----------|------------|
| Display symbol IDs (0–13, 21–29) | `Phone App/navigation/lib/src/symbols.dart` | `Front Module/front_module.ino` |
| Bluetooth packet (version, flags, distance, symbols) | `Phone App/navigation/lib/src/protocol.dart` | `Front Module/front_module.ino` |
| BLE service / characteristic UUIDs | `protocol.dart` | `front_module.ino` |
| OLED screen layout | `Phone App/navigation/lib/src/oled.dart` (`renderOled`) | Drawing code in `front_module.ino` |

When the Camera or Rear Module gets a Bluetooth interface, add its rows to this table and to section 5 of `Phone App/SPECIFICATION.md`.

## Rule 4: Never commit secrets

- **No API keys, passwords or tokens in any file**, including free-tier keys. This repository is public.
- OpenRouteService key: passed at build time with `--dart-define=ORS_API_KEY=...`.
- Google Maps key: `Phone App/app/android/local.properties` (`MAPS_API_KEY=...`), which git ignores.
- Wi-Fi passwords or cloud keys for the ESP32 modules must not be committed either. Leave a placeholder constant such as `const char* WIFI_PASSWORD = "CHANGE_ME";` and explain in the spec where the real value goes.

## Rule 5: Test what you can, and say what you couldn't

- Phone App: run `dart test` in `Phone App/navigation` and `flutter test` in `Phone App/app` after changing them.
- ESP32 sketches: compile them for the ESP32 if you can.
- If you couldn't run or test something, write that in the module spec's section 6 ("untested") and in your commit message.

## Commits

- One logical change per commit, with a message saying which module and what changed, e.g. `Front Module: draw roundabout exit number`.
- Code change + spec update + README status change go in the same commit.
