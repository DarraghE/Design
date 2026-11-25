# Tools

Occasional-use helpers that run **on a computer**, not on the bike. You don't need them to understand or change the module firmware. Use them for flashing, bench testing and debugging.

| Tool | For | Runs on | What it does |
|------|-----|---------|--------------|
| [`flash_module.command`](flash_module.command) | All ESP32 modules | macOS / Linux (bash) | Builds one module's `.ino` with arduino-cli and uploads it, or just compiles it with `--compile-only` |
| [`LiDAR Viewer/`](LiDAR%20Viewer/) | Rear Module + Camera Module | Mac (Python 3) | Forwards live LiDAR data to the Rear Module, or replays a `.lidar` recording. Shows the point cloud and detections in a browser, saves close-pass photo bursts, and has Test Camera / Test Lights buttons |
| [`OLED Image Test/`](OLED%20Image%20Test/) | Front Module | Chrome/Edge or Python 3 | Converts any picture to 128x64 black and white and sends it to the Front Module's image receiver |

These came from the firmware backup of 25 Nov 2026 (`Archive/Firmware_Backup_v2_2026-11-25/`). The viewer and OLED files are unchanged copies. `flash_module.command` is new: it replaces the backup's three flashing scripts, which pointed at old folder names. **It has been compile-tested only, never used to upload to a board.**

---

## Flashing a module

**Easiest (any computer):** open the module's `.ino` in the Arduino IDE (it will offer to put it in a folder of the same name: click OK). Choose **ESP32 Dev Module**, set upload speed **115200**, pick the port, and click **Upload**. Install the libraries listed at the top of the `.ino` first.

**Script (macOS/Linux, needs arduino-cli):**

```bash
bash Tools/flash_module.command front  /dev/cu.usbserial-XXXX
bash Tools/flash_module.command camera /dev/cu.usbserial-XXXX
bash Tools/flash_module.command rear   /dev/cu.wchusbserialXXXX
bash Tools/flash_module.command rear   --compile-only      # check it builds, upload nothing
```

- Find the port by unplugging and replugging the board, then running `ls /dev/cu.*`. Don't trust old port names. The OLED board and the LiDAR have both appeared as `/dev/cu.usbserial-0001` at different times.
- Close any serial monitor or viewer using the port first.
- **Flashing replaces whatever is on the board.** Don't flash just to look at a board.
- If the build fails with `Failed to extract libcrypto.1.1.dylib`, nothing was uploaded. Check free disk space with `df -h "$HOME" "${TMPDIR:-/tmp}"`.

## LiDAR Viewer (Rear Module)

Needs Python 3 with `pyserial` (`pip install pyserial`). Run from the top folder of the repo:

```bash
# Live: STL-19P LiDAR plugged into the Mac, Rear Module on another USB port
python3 "Tools/LiDAR Viewer/viewer.py" REAR_PORT --lidar LIDAR_PORT
# Replay: no LiDAR needed, pick a .lidar recording in the web page
python3 "Tools/LiDAR Viewer/viewer.py" REAR_PORT
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
| `viewer.py` | Serial link to the Rear Module (460800 baud) and the LiDAR (230400), replay, photo saving, local web server |
| `scan_wire.py` | Packs scans and raw LiDAR bytes into the Rear Module's USB frame format, and decodes its replies |
| `lidar_core.py` | STL-19P decoder and `.lidar` recording loader/saver (a `.lidar` file is a ZIP with `recording.json`, `scans.jsonl`, optional `serial.bin`) |
| `index.html` | The web page |

No recordings are included. The USB protocol is in `Rear Module/SPECIFICATION.md`, section 2.

## OLED Image Test (Front Module)

The Front Module must be running `front_module.ino` (the image receiver).

- **Browser:** serve the folder locally and open it in Chrome or Edge (Web Serial needs localhost):
  ```bash
  cd "Tools/OLED Image Test" && python3 -m http.server 8000 --bind 127.0.0.1
  ```
  Open `http://127.0.0.1:8000/OLED_Dashboard_v2.html`, click **Connect ESP32**, choose a picture, adjust the threshold or invert, then **Send image to OLED**. The log shows the board's replies.
- **Python:** `pip install pyserial pillow`, then `python3 "Tools/OLED Image Test/sender.py" PORT picture.png`. It writes `preview.png` and prints the board's reply. It reads only one reply line, so a start-up message can make it look like a failure. The browser version keeps reading.

`DISPLAYED` means the board *drew* the picture. It doesn't prove the screen is wired correctly: check the physical screen.
