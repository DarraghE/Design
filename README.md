# Design: Bike Nav

A cycling navigation and safety system for a bike in Ireland:
- **turn-by-turn navigation** on a small handlebar screen, worked out by a phone;
- a **LiDAR at the back of the bike** that warns of vehicles approaching from behind (red lights) and spots **close passes** within 1.5 m;
- a **camera** that photographs close passes.

| Folder | Runs on | Status | Code |
|--------|---------|--------|------|
| [Phone App](Phone%20App/SPECIFICATION.md) | Android phone | Navigation works (route, GPS, rerouting, screen preview). No Bluetooth yet | Flutter app (`app/`) + navigation package (`navigation/`) |
| [Front Module](Front%20Module/SPECIFICATION.md) | ESP32 DevKit V1 + 128x64 OLED | Test firmware: shows a picture sent over USB. Not confirmed on hardware | `front_module.ino` |
| [Camera Module](Camera%20Module/SPECIFICATION.md) | M5Stack Timer Camera X | Working photo server over ESP-NOW (uploaded 22 Nov, per the backup) | `camera_module.ino` |
| [Main Module](Main%20Module/SPECIFICATION.md) | FireBeetle ESP32 at the back of the bike | The hub: LiDAR approach + close-pass detection, LED warning, photo bursts (v1.13 compiles, not confirmed on hardware; LiDAR goes through a Mac for now). Phone link and forwarding images to the Front Module: planned, not built | `main_module.ino` |
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
                                                         Camera Module (10 photos per close pass)

 Mac (Tools/OLED Image Test) ──USB──▶ Front Module (shows a test picture)

 Phone App: navigation runs on the phone and previews the handlebar screen on the phone. Not linked to any module yet.
```

**Planned:**

```
 Phone App ──Bluetooth (planned)──▶ Main Module (back of bike: LiDAR, lights, hub) ──▶ Front Module (screen)
                                         │
                                         └── ESP-NOW ──▶ Camera Module
```

The **Main Module** is the phone's main link. The phone draws each navigation screen as an image and sends it to the Main Module, which passes it straight on to the Front Module. Still to decide: the phone link details, the Main Module → Front Module link, the image format, and how the LiDAR connects without the Mac. See `Front Module/SPECIFICATION.md` section 1.2.

## Before you trust it

None of the ESP32 firmware has been confirmed working on the bike. The detection thresholds are hand-tuned and not validated. This is a prototype, **not a safety device**. Test stationary and in controlled conditions first.

Anyone adding code (people or AI agents) should read [AGENTS.md](AGENTS.md) first.
