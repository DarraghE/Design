# Tools

Occasional-use helpers that run **on a computer**, not on the bike. You don't need them to understand or change the module firmware. Use them for flashing, bench testing and debugging.

| Tool | For | Runs on | What it does |
|------|-----|---------|--------------|
| [`flash_module.command`](flash_module.command) | All ESP32 modules | macOS / Linux (bash) | Builds one module's `.ino` with arduino-cli and uploads it, or just compiles it with `--compile-only` |
| [`LiDAR Viewer/`](LiDAR%20Viewer/) | Main Module + Camera Module | Mac (Python 3) | Forwards live LiDAR data to the Main Module, or replays a `.lidar` recording. Shows the point cloud and detections in a browser, saves close-pass photo bursts, and has Test Camera / Test Lights buttons |
| [`OLED Image Test/`](OLED%20Image%20Test/) | Front Module | Chrome/Edge or Python 3 | Converts any picture to 128x64 black and white and sends it to the Front Module's image receiver |
| [`Phone Link Test/`](Phone%20Link%20Test/) | Main Module + Phone App | Any computer (Python 3 + a C++ compiler) | Compiles the Main Module's phone-protocol code on the computer, checks it, and writes the exact Bluetooth notifications it sends for one close pass. The phone app's test `firmware_vectors_test.dart` then decodes them with the app's own code |

These came from the firmware backup of 25 Nov 2026 (`Archive/Firmware_Backup_v2_2026-11-25/`). The OLED files are unchanged copies. The viewer has one change: it also accepts `READY 1.20 RADIO` (sent by Main Module v1.20, v1.21 and v1.25), and no longer assumes 10 photos per burst. It is still the v1.13 viewer: no buttons for v1.20's `A`/`M`/`T` commands (type them in a serial monitor instead), and the newer viewer Oscar used isn't in the repo. `Phone Link Test/` is new. `flash_module.command` is new: it replaces the backup's three flashing scripts, which pointed at old folder names. **It has been compile-tested only, never used to upload to a board.**

---

## Flashing a module

**Easiest (any computer):** open the module's `.ino` in the Arduino IDE (it will offer to put it in a folder of the same name: click OK). Choose **ESP32 Dev Module**, set upload speed **115200**, pick the port, and click **Upload**. Install the libraries listed at the top of the `.ino` first. **Main Module only:** also set Tools → Partition Scheme → **Huge APP (3MB No OTA/1MB SPIFFS)** (it includes Bluetooth).

**Script (macOS/Linux, needs arduino-cli):**

```bash
bash Tools/flash_module.command front  /dev/cu.usbserial-XXXX
bash Tools/flash_module.command camera /dev/cu.usbserial-XXXX
bash Tools/flash_module.command main   /dev/cu.wchusbserialXXXX
bash Tools/flash_module.command main   --compile-only      # check it builds, upload nothing
```

- Find the port by unplugging and replugging the board, then running `ls /dev/cu.*`. Don't trust old port names. The OLED board and the LiDAR have both appeared as `/dev/cu.usbserial-0001` at different times.
- Close any serial monitor or viewer using the port first.
- **Flashing replaces whatever is on the board.** Don't flash just to look at a board.
- If the build fails with `Failed to extract libcrypto.1.1.dylib`, nothing was uploaded. Check free disk space with `df -h "$HOME" "${TMPDIR:-/tmp}"`.

## LiDAR Viewer (Main Module)

Needs Python 3 with `pyserial` (`pip install pyserial`). Run from the top folder of the repo:

```bash
# Live: STL-19P LiDAR plugged into the Mac, Main Module on another USB port
python3 "Tools/LiDAR Viewer/viewer.py" MAIN_PORT --lidar LIDAR_PORT
# Replay: no LiDAR needed, pick a .lidar recording in the web page
python3 "Tools/LiDAR Viewer/viewer.py" MAIN_PORT
```

It opens a local web page (127.0.0.1, random port) with:
- the point cloud: blue for normal returns, pink for close-pass returns, red for an approaching vehicle, and the approach zone in yellow;
- the detection state and speed;
- **Test Camera**, which takes one photo shown for 1 s and doesn't save it;
- **Test Lights**, which sends one red pulse;
- a debug log.

Close-pass bursts are saved as `~/Downloads/LiDAR_<time>_Burst_<n>/photo_01.jpg`…`photo_10.jpg`.

| File | Role |
|------|------|
| `viewer.py` | Serial link to the Main Module (460800 baud) and the LiDAR (230400), replay, photo saving, local web server |
| `scan_wire.py` | Packs scans and raw LiDAR bytes into the Main Module's USB frame format, and decodes its replies |
| `lidar_core.py` | STL-19P decoder and `.lidar` recording loader/saver (a `.lidar` file is a ZIP with `recording.json`, `scans.jsonl`, optional `serial.bin`) |
| `index.html` | The web page |

No recordings are included. The USB protocol is in `Main Module/SPECIFICATION.md`, section 2.

## OLED Image Test (Front Module)

The Front Module must be running `front_module.ino` (the image receiver).

- **Browser:** serve the folder locally and open it in Chrome or Edge (Web Serial needs localhost):
  ```bash
  cd "Tools/OLED Image Test" && python3 -m http.server 8000 --bind 127.0.0.1
  ```
  Open `http://127.0.0.1:8000/OLED_Dashboard_v2.html`, click **Connect ESP32**, choose a picture, adjust the threshold or invert, then **Send image to OLED**. The log shows the board's replies.
- **Python:** `pip install pyserial pillow`, then `python3 "Tools/OLED Image Test/sender.py" PORT picture.png`. It writes `preview.png` and prints the board's reply. It reads only one reply line, so a start-up message can make it look like a failure. The browser version keeps reading.

`DISPLAYED` means the board *drew* the picture. It doesn't prove the screen is wired correctly: check the physical screen.

## Phone Link Test (Main Module + Phone App)

Checks that the Main Module firmware builds exactly the Bluetooth bytes the phone app expects, without any hardware.

```bash
python "Tools/Phone Link Test/run_test.py"           # needs a C++ compiler, or: pip install ziglang
cd "Phone App/app" && flutter test test/firmware_vectors_test.dart
```

1. `run_test.py` copies the block between `// BEGIN PHONE WIRE` and `// END PHONE WIRE` out of `Main Module/main_module.ino` into `phone_wire.inc` (not committed) and compiles `host_test.cpp` with it.
2. `host_test.cpp` checks the block (CRC-32, picking LiDAR points, joining buffers), then builds one close pass the same way the firmware does (3 photos, a 5-scan LiDAR clip) and writes every notification for MTU 23, 185, 247 and 517 to `firmware_notifications.txt`.
3. The phone app's `firmware_vectors_test.dart` decodes that file with the app's real decoder and checks every value.

Run it after changing the `phone` or `wire` code in `main_module.ino`, and commit `firmware_notifications.txt` if it changed. It tests the bytes, not the radio: Bluetooth timing, pairing and ESP-NOW still need the real boards.

