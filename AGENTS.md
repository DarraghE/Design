# AGENTS.md: How to work in this repository

These are the rules for any AI agent (or person) adding or changing code here. Read this file first, then read the `SPECIFICATION.md` in the folder you're working in.

## What this project is

Bike Nav: a cycling navigation and safety system. An Android phone works out the route. Three ESP32 modules on the bike:
- **Main Module:** the hub at the back of the bike. It has the LiDAR and the warning lights, triggers the camera, and is the phone's main link.
- **Front Module:** controls the handlebar screen.
- **Camera Module:** the camera ESP32.

The phone talks to the Main Module, which passes navigation images on to the Front Module. The overview and system diagram are in `README.md`.

## Repository layout (keep it exactly like this)

```
README.md              Overview table of all modules, status, system diagram
AGENTS.md              This file
Phone App/             Android app (Flutter) + navigation package (plain Dart)
  SPECIFICATION.md
  app/                 Flutter app
  navigation/          bike_navigation package (no Flutter imports)
  docs/                Design notes and images
Front Module/          ESP32 DevKit V1 handlebar display (128x64 SSD1306 OLED)
  SPECIFICATION.md
  front_module.ino
Camera Module/         M5Stack Timer Camera X
  SPECIFICATION.md
  camera_module.ino
Main Module/           FireBeetle ESP32 hub at the back of the bike: LiDAR detection,
                       LED warning, camera trigger, Bluetooth phone link
  SPECIFICATION.md
  main_module.ino
Tools/                 Occasional computer-side helpers (flash script, LiDAR viewer,
                       OLED image test). Not firmware, not needed to read the modules
Archive/               Read-only history: old repo contents, and the original
                       firmware backup (Firmware_Backup_v2_2026-11-25/)
```

- **Don't add new top-level folders** or rename the module folders without being asked.
- **Don't edit, move or delete anything in `Archive/`.** It is kept only for reference. The firmware backup's `FILE_INDEX.json` SHA-256 hashes must keep matching.
- Computer-side helpers (scripts, viewers, test pages) go in `Tools/`, never in a module folder. Describe each one in `Tools/README.md`.

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

The Front, Camera and Main Modules are flashed straight onto ESP32s, so each must stay **one single Arduino sketch** that can be copied and uploaded from the Arduino IDE:

- All firmware for a module goes in its `.ino` file (`front_module.ino`, `camera_module.ino`, `main_module.ino`). No extra `.cpp`/`.h` files, no PlatformIO projects, no sub-folders.
- The top of the file has a comment block giving: the module name, a one-line purpose, the board to select in the Arduino IDE, and **every library needed with its version** (as named in the Arduino Library Manager).
- Put pin numbers, I2C addresses, UUIDs and tunable values as named constants near the top of the file. The spec's wiring table must match them.
- It must compile for the ESP32 with only the listed libraries installed.
- The Arduino IDE and arduino-cli need a sketch to sit in a folder with the same name (`main_module/main_module.ino`). Don't rename the module folders to fix this: the IDE offers to make the folder, and `Tools/flash_module.command` copies the file into one.
- The existing sketches came from a firmware backup and don't all keep their constants at the top. Instead, their header comment lists the important settings. If you restructure a sketch, keep its behaviour, serial messages and version string unchanged unless you were asked to change them.

The Phone App isn't limited to one file. Keep its existing structure (`app/` and `navigation/`).

## Rule 3: Keep the module contracts in sync

Modules and tools depend on each other through shared numbers and formats. If you change one side, change the other side **and both specs** in the same commit.

| Contract | Defined in | Must match |
|----------|-----------|------------|
| ESP-NOW photo protocol: channel 6, `Request` 8 bytes, `Reply` 236 bytes, 224-byte chunks, 128 KB max | `Main Module/main_module.ino` (`Radio.h` section) | `Camera Module/camera_module.ino` |
| Camera's MAC address (`cameraMac`) | `main_module.ino` | The real camera board. Update it if the camera is replaced |
| Main Module USB protocol (460800 baud: `V`/`C`/`P`/`R`, `0x7E` frames, `READY`/`SCAN`/`LIVE`/`JPEG`/`EVENT` lines) | `main_module.ino` | `Tools/LiDAR Viewer/viewer.py` and `scan_wire.py` |
| Firmware version string `READY 1.13 RADIO` | `main_module.ino` | `Tools/LiDAR Viewer/viewer.py` |
| OLED image packet: `OLED` + 64 + 1024 bytes, row by row, top bit = leftmost pixel | `Front Module/front_module.ino` | `Tools/OLED Image Test/` (dashboard and `sender.py`) |
| Phone ↔ Main Module Bluetooth LE (service/characteristic UUIDs, message types `0x10 0x30 0x40 0xA1 0xB1-B3 0xC1`, CRC-32) | `Phone App/app/lib/link/protocol.dart` | Main Module firmware (to build: `Phone App/SPECIFICATION.md` §8) |
| Close-pass package `CPK1` (photos + LiDAR clip) | `Phone App/app/lib/link/close_pass_package.dart` | Main Module firmware; the website's `lidar` file |
| Handlebar frame format (row-major, row XOR, PackBits, `0x10` chunks) | `Phone App/navigation/lib/src/display_codec.dart` | Front Module firmware (to build: `Phone App/SPECIFICATION.md` §9); Main Module forwards it unchanged |
| Website upload (multipart: `incident` JSON, `image`, `photos`, `lidar`) | `Phone App/app/lib/closepass/uploader.dart` | The website (`Phone App/docs/website_upload.md`) |

The in-app demo Main Module (`Phone App/app/lib/link/demo_main_module.dart`) implements the Main Module side of these contracts. Keep it in step with any change.

## Rule 4: Never commit secrets

- **No API keys, passwords or tokens in any file**, including free-tier keys. This repository is public.
- OpenRouteService key: typed into the app's Settings (or `--dart-define=ORS_API_KEY=...` for a private build only).
- OpenAI key and website token: typed into the app's Settings on the phone, never in code, build files or a committed APK.
- Wi-Fi passwords or cloud keys for the ESP32 modules must not be committed either. Leave a placeholder constant such as `const char* WIFI_PASSWORD = "CHANGE_ME";` and explain in the spec where the real value goes.

## Rule 5: Test what you can, and say what you couldn't

- Phone App: run `dart test` in `Phone App/navigation` and `flutter test` in `Phone App/app` after changing them.
- ESP32 sketches: compile them for the ESP32 if you can, e.g. `bash Tools/flash_module.command main --compile-only`. They were last compile-checked with ESP32 core 3.2.1 on 25 Nov 2026. On Windows the ESP32 toolchain can fail with missing `bits/...` headers when paths are long. Build from a short path (for example a `subst` drive letter).
- If you couldn't run or test something, write that in the module spec's section 6 ("untested") and in your commit message.
- **Compiling is not the same as working.** Only write that something works on hardware when someone has actually seen it working.

## Rule 6: Report known issues, don't quietly fix them

The specs list known mismatches between what was intended and what the code does. For example, a close pass on the Main Module takes photos but doesn't flash the lights, and the Front Module's SDA pin has changed between versions. **Don't change these unless you've been asked to.** Point them out instead. When you describe behaviour, describe what the code actually does, not what was intended, and say which is which.

## Commits

- One logical change per commit, with a message saying which module and what changed, e.g. `Front Module: draw roundabout exit number`.
- Code change + spec update + README status change go in the same commit.
