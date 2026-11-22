# Design: Bike Nav

A cycling navigation and safety system for a bike. The phone works out the route and talks to small ESP32 modules mounted on the bike.

| Folder | Runs on | Status | Code |
|--------|---------|--------|------|
| [Phone App](Phone%20App/SPECIFICATION.md) | Android phone | Navigation working. Bluetooth not built yet | Flutter app (`app/`) + navigation package (`navigation/`) |
| [Front Module](Front%20Module/SPECIFICATION.md) | ESP32 (handlebar OLED) | Not started | `front_module.ino` |
| [Camera Module](Camera%20Module/SPECIFICATION.md) | ESP32 | Not started | `camera_module.ino` |
| [Rear Module](Rear%20Module/SPECIFICATION.md) | ESP32 | Not started | `rear_module.ino` |
| [Archive](Archive/) | n/a | Old contents of this repo (up to 22 Nov 2026), kept for reference only | |

Each folder has a `SPECIFICATION.md` saying what its code does, how to build it and what is still to do. Start there.

Each ESP32 module has one `.ino` file: open it in the Arduino IDE, pick the ESP32 board and click Upload.

Anyone adding code (people or AI agents) should read [AGENTS.md](AGENTS.md) first.
