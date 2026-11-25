# Design: Bike Nav

A cycling navigation and safety system for a bike in Ireland:
- **turn-by-turn navigation** on a small handlebar screen, worked out by a phone;
- a **LiDAR at the back of the bike** that warns of vehicles approaching from behind (red lights) and spots **close passes** within 1.5 m;
- a **camera** that photographs close passes.

| Folder | Runs on | Status | Code |
|--------|---------|--------|------|
| [Phone App](Phone%20App/SPECIFICATION.md) | Android phone (Pixel 9a) | **App complete**: Navigation and Close Pass modes, Bluetooth link, LiDAR replay, OpenAI plate reading, website upload, demo module. 87 tests pass, including the Main Module firmware's Bluetooth bytes. Not yet tried with the real modules. **Install: [`Phone App/release/BikeNav.apk`](Phone%20App/release/BikeNav.apk)** | Flutter app (`app/`) + navigation package (`navigation/`) |
| [Front Module](Front%20Module/SPECIFICATION.md) | ESP32 DevKit V1 + 128x64 OLED | **v1.21 written, to flash:** shows the phone's navigation screens (decoder checked on all 415 app screens) and camera photos when no phone is connected. The board still runs v1.19, which ignores the phone's screens | `front_module.ino` |
| [Camera Module](Camera%20Module/SPECIFICATION.md) | M5Stack Timer Camera X | Working photo server over ESP-NOW (uploaded 22 Nov, per the backup) | `camera_module.ino` |
| [Main Module](Main%20Module/SPECIFICATION.md) | FireBeetle ESP32 at the back of the bike | The hub: LiDAR approach + close-pass detection, LED warning, photo bursts, photos to the Front Module. **v1.26 is installed on the board** (= reconstructed v1.21 + the Bluetooth phone link + a USB image debugger); v1.28 adds error replies to the USB image debugger. The phone link compiles and its bytes match the app, but **hasn't been tried with the phone yet** (see its spec, section 6) | `main_module.ino` |
| [Tools](Tools/README.md) | A computer | Occasional helpers: flash script, LiDAR viewer, OLED image test | |
| [Archive](Archive/ARCHIVE_NOTE.md) | n/a | Old repo contents and the original firmware backup, kept unchanged for reference | |

Each module folder has a `SPECIFICATION.md` saying what its code does, how to build it and what's still to do. Start there. Each ESP32 module is one `.ino` file: open it in the Arduino IDE, choose **ESP32 Dev Module**, and click Upload.

## How the parts connect

**Now (bench prototype):**

```
 STL-19P LiDAR ──USB──▶ Mac (Tools/LiDAR Viewer) ──USB──▶ Main Module ──▶ red LED strip (approach warning)
                                ▲                              │
                                └──── detections, photos ──────┤ ESP-NOW (wireless)
                                                               ▼
                                                         Camera Module (5 photos per close pass)

 Main Module ──ESP-NOW (photos as JPEG)──▶ Front Module (OLED, 80x60 picture)

 Phone App: complete. Works with its built-in demo Main Module. Main Module v1.26 (installed)
 has the Bluetooth side, not yet tried with the phone. Front v1.21 (written, to flash) draws the phone's screens.
```

**Designed (phone side built and tested; ESP32 side to build):**

```
 Phone App ──Bluetooth LE──▶ Main Module (back of bike: LiDAR, lights, hub) ──ESP-NOW──▶ Front Module (screen)
     ▲                              │
     └── close-pass alert,          └── ESP-NOW ──▶ Camera Module (5 photos per close pass)
         photos, LiDAR clip
```

- **Navigation mode:** the phone draws every handlebar screen and sends it compressed (about 200 bytes) to the Main Module, which forwards it unchanged to the Front Module. Nothing comes back.
- **Close Pass mode (always on):** the Main Module alerts the phone, then sends the photos and a LiDAR clip. The phone shows "CLOSE PASS" on the handlebar (unless navigating), replays the pass, reads the number plate with OpenAI and uploads it to the website.
- Exact bytes: `Phone App/SPECIFICATION.md` section 3. Main Module side: built in v1.25, installed as v1.26 (`Main Module/SPECIFICATION.md` sections 1.6, 2.3, 8). Front Module work list: section 9. Website contract: `Phone App/docs/website_upload.md`.

## Before you trust it

None of the ESP32 firmware has been confirmed working on the bike. The detection thresholds are hand-tuned and not validated. This is a prototype, **not a safety device**. Test stationary and in controlled conditions first.

Anyone adding code (people or AI agents) should read [AGENTS.md](AGENTS.md) first.
