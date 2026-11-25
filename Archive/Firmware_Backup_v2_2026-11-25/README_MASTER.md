# Bicycle close-pass detection project — firmware handover v2

Snapshot date: 2026-11-25. Documentation revision only; firmware bytes unchanged from backup v1.

## 1. Read this first
This archive is a source-code backup, not a flash-memory dump. It contains three physically distinct ESP32 devices: an OLED display receiver, an M5Stack Timer Camera X, and a FireBeetle relay controlling lights and processing LiDAR. The Mac is currently the USB bridge for the sensor. Do not assume that these devices share a serial port, pin mapping, or firmware. Use `hardware_manifest.json` for structured facts, `FILE_INDEX.json` for paths, sizes and SHA-256 hashes, and the existing per-package READMEs for historical instructions. Source code takes precedence over historical intentions. No compile, upload, or physical validation was performed for this documentation revision.

## 2. Architecture and data flow
```text
STL-19P LiDAR -- USB serial 230400 --> Mac viewer
Mac viewer -- framed raw bytes / replay points, USB 460800 --> FireBeetle relay
FireBeetle relay -- GPIO2 --> WS2813 warning light strip
FireBeetle relay <-- ESP-NOW channel 6, JPEG chunks --> Timer Camera X
FireBeetle relay -- scan results + JPEGs, USB 460800 --> Mac viewer --> browser
Separate OLED dashboard -- USB 115200 --> ESP32 DevKit V1 -- I2C --> OLED
```
The OLED is currently a standalone image test subsystem. There is no implemented relay-to-OLED or camera-to-OLED automatic link in this archive. A future integration must explicitly define this interface; do not infer that the display already shows detection or camera data.

## 3. Version selection / provenance
| Device | Preferred source | Evidence and limitation |
|---|---|---|
| OLED ESP32 | `OLED_GPIO15_22/Receiver/Receiver.ino` | Matches user-reported latest wiring SDA GPIO15, SCL GPIO22. Not verified flashed. |
| OLED prior build | `OLED_Test_v1/Receiver/Receiver.ino` | SDA21/SCL22; compile patch applied; upload log successful on Nov 25, display not confirmed. |
| Camera | `TimerCam_Camera_Only/Camera/Camera.ino` | Previous backup associates this with successful Nov 22 upload; not independently read back. |
| Relay | `LiDAR_v1.13_Complete_Setup/ESP32_Relay/Relay/Relay.ino` | Latest recovered v1.13, actual installation not confirmed. Nov 22 log showed libcrypto extraction failure before upload. |
| Camera alternates | `LiDAR_v1.13_Complete_Setup/TimerCam/Camera/Camera.ino`, `TimerCam/Recovery/Camera/Camera.ino` under same package | Historical alternatives, not preferred. Main nested camera differs from preferred in min() expression; recovery is a separate older implementation. |

## 4. Wiring and hardware
All GPIO numbers mean actual chip GPIO, not an arbitrary board silkscreen D-number. Confirm board pinout before changing wiring. Never connect 5 V logic directly to an ESP32 input. Disconnect power before rewiring.

### 4.1 OLED ESP32 DevKit V1
| OLED label | ESP32 connection (latest stated) |
|---|---|
| SDA | GPIO15 / D15 |
| SCK or SCL | GPIO22 / D22 |
| VDD | 3V3 |
| GND | GND |
Firmware assumes SSD1306 128x64 monochrome I2C display, address 0x3C, no dedicated reset pin (-1). Actual controller and address have not been independently verified. Earlier user wiring used SCK D1, then changed to D22. GPIO15 is a boot strapping pin; if boot issues appear, inspect the external circuitry. Historical I2C NACK errors indicate a bus/device acknowledgement problem; a successful upload alone does not establish that display data arrived. An old image remaining on the screen does not establish the new firmware is functioning. `DISPLAYED` is a software acknowledgement, not physical readback.

### 4.2 Relay and LED strip
FireBeetle ESP32 is the target identified in v1.13 README. Relay code sets actual GPIO2 (comment: FireBeetle D9) to LED data, 30 pixels, GRB, 800 kHz, brightness 20/255. User previously described a WS2813 strip with DI/DO/BI/GND/+5V. Connect data to DI, external regulated 5 V to strip +5V, and common ground to strip and ESP32. DO is downstream output, not input. BI wiring is not verified; consult exact strip datasheet. A compatible logic level shifter may be needed for reliable 3.3V-to-5V signalling. Power strip from appropriately rated supply, not ESP32 GPIO. Exact strip length, current rating, level-shifter presence and enclosure wiring are unknown. Do not assume Arduino D2 from earlier prototype equals FireBeetle D9 label.

### 4.3 LiDAR
DTOF STL-19P (decoder also described as D500 compatible), serial 230400 8N1. Current implementation expects Mac USB bridge, not direct GPIO wiring. Optional direct UART2 mode is compiled only when `LIDAR_RX_PIN` is explicitly defined. No RX GPIO chosen or verified. Connect sensor TX to chosen RX only after voltage and board pinout verification, with common ground; UART TX disabled (-1). Use one raw input at a time. Sensor power and connector pinout are not verified here.

### 4.4 Camera internal pin map
M5Stack Timer Camera X OV3660. These are internal module connections, not instructions to rewire the camera. GPIO33 HIGH holds battery power. Data D0..D7 = 32,35,34,5,39,18,36,19. XCLK27, PCLK21, VSYNC22, HREF26, SCCB SDA25/SCL23, RESET15, PWDN=-1. XCLK 20 MHz. JPEG QVGA 320x240, quality 12, one DRAM framebuffer, grab when empty. USB serial 115200 diagnostics only; photo retrieval via ESP-NOW.

## 5. Host environment and install
MacBook/macOS, Arduino CLI expected at `$HOME/.esp-photo/arduino-cli` for relay/camera scripts. Historical ESP32 Arduino cores 3.2.0 and 3.2.1; exact installed version may differ. Firmware uses ESP-NOW callback API of modern Arduino ESP32. Upload FQBN `esp32:esp32:esp32:UploadSpeed=115200`. OLED dependencies Adafruit GFX, SSD1306, BusIO; observed library versions GFX 1.13.0, SSD1306 2.5.17, BusIO 1.17.4. Relay uses Adafruit NeoPixel (observed 1.15.5). Python dependencies pyserial and Pillow for OLED sender; relay viewer uses pyserial and Python standard library. Browser OLED dashboard requires Web Serial (Chrome/Edge, preferably localhost).

Historical ports: OLED and LiDAR have both appeared as `/dev/cu.usbserial-0001` at different times; relay `/dev/cu.wchusbserial14110`; camera `/dev/cu.usbserial-2952066438` (other earlier ports existed). Do not use historical port names blindly. Identify by disconnecting/reconnecting one device and running `ls /dev/cu.*`. Close serial monitor and viewers before uploading. Flashing replaces existing application; do not flash merely to inspect a device.

From extracted archive root:
```bash
# OLED: only after matching GPIO15/22 wiring; script uploads immediately
bash OLED_GPIO15_22/test.sh
# Preferred camera source: replace PORT with verified camera port
"$HOME/.esp-photo/arduino-cli" compile --upload --fqbn esp32:esp32:esp32:UploadSpeed=115200 --port PORT TimerCam_Camera_Only/Camera
# Relay: replace RELAY_PORT with verified relay port
bash LiDAR_v1.13_Complete_Setup/ESP32_Relay/Flash_Relay.command RELAY_PORT
# Live Mac viewer (install pyserial in chosen Python environment first)
"$HOME/.venv/bin/python" LiDAR_v1.13_Complete_Setup/Mac_Viewer/viewer.py RELAY_PORT --lidar LIDAR_PORT
# Replay mode: omit --lidar
"$HOME/.venv/bin/python" LiDAR_v1.13_Complete_Setup/Mac_Viewer/viewer.py RELAY_PORT
# Serve OLED dashboard without flashing
cd OLED_GPIO15_22 && python3 -m http.server 8000 --bind 127.0.0.1
# Open http://127.0.0.1:8000/OLED_Dashboard_v2.html in Chrome
```
If libcrypto extraction fails, check free disk space (`df -h "$HOME" "${TMPDIR:-/tmp}"`); build failure is not upload success. OLED script attempts Homebrew installation if CLI absent; installs libraries, flashes, creates virtual environment, opens file picker, sends image and opens preview. Python sender reads only one response line and may mistake boot messages for failure; dashboard reads ongoing logs.

## 6. Protocols
### OLED serial
115200 baud, command packet exactly 1029 bytes: ASCII OLED (4), height byte 64 (1), bitmap 1024 bytes (128x64, row-major, MSB first, 1=white). No checksum or length beyond fixed size. Receiver scans for magic, checks height, reads fixed buffer with 3s serial timeout, clears display and draws bitmap. Replies READY, DISPLAY_ERROR, HEIGHT_ERROR, TRANSFER_ERROR, DISPLAYED. Browser fits image to canvas with letterboxing, threshold and inversion; Python sender uses Pillow monochrome conversion. No image persistence across reset. Two OLED folders contain equivalent dashboard/sender, different I2C pins.

### Radio (camera/relay)
ESP-NOW unencrypted, station mode, channel 6, sleep off. Relay hardcodes camera MAC in Radio.h; see source for address. Camera preferred sketch accepts compatible requesters dynamically. Request 8 bytes: uint32 token, uint32 offset; Reply 236 bytes: same request, uint32 total JPEG size, 224-byte chunk. Native little-endian ESP32 fields. Token nonzero; offsets multiples of 224; max JPEG 131072 bytes. Camera returns UINT32_MAX for failed capture. Offset zero starts capture; stale buffered frame discarded, fresh frame retained for retries. One active transfer, released after 2000ms idle. Relay retries each chunk up to four times, initial wait 600ms, subsequent 50ms, overall transfer budget 8000ms after first response. Relay filters sender MAC and matches token/offset. No encryption/authentication beyond protocol checks; do not treat as secure or suitable for adversarial environments.

### Relay USB
460800 baud. Single-byte commands V=version, C=light test, P=camera test, R=reset detector. Framed payload marker 0x7e; escape 0x7e and 0x7d using 0x7d followed by byte XOR 0x20. Header little-endian uint32 scan ID, uint32 elapsed ms, uint16 count. Each replay point int16 x mm, int16 y mm, uint16 angle centidegrees. Max 1024 points. Count 65535 means raw LiDAR bytes following header instead of points. Partial frames discarded after >250ms input gap. Output: `READY 1.13 RADIO OK/ERROR`, `OK LED PULSE GPIO2`, `EVENT <close_count> <approach_count>`, `SCAN <id> <close> <incoming> <km/h> [+close_indices] [-approach_indices]`, `LIVE <stamp> <close> <incoming> <km/h> <count>` followed by 5 bytes/point (int16 x mm, int16 y mm, uint8 class 0 ordinary / 1 close / 2 approach), `JPEG <size> <burst> <shot>` followed by exact raw JPEG bytes; ERR/INFO diagnostics. Binary data must be read by explicit length, not line splitting.

### STL-19P raw sensor decoder
47-byte packets: 0x54,0x2c header; 12 beams; CRC8 polynomial 0x4d over first 46 bytes. Speed, start/end centidegree angles, distances in mm, device time used for sweep assembly. Reject corrupt/incomplete/discontinuous sweeps. Sensor crop raw 55–215 degrees, distance 30–12000 mm. Coordinates x=r*sin(raw angle), y=r*cos(raw angle). Raw sensor angle differs from geometric atan2 angle. Geometric 0° is +x right; 90° +y front; 180° left; 270° rear (-y). Earlier user notes sometimes used inconsistent rear labels; code and physical orientation calibration take precedence.

## 7. Detection algorithm (actual v1.13 source)
No machine learning. `Detection.h` is authoritative. Close detector: raw sensor angle [55,180), radius 0.15–1.5 m; sort by y; group consecutive y gaps <=0.55m; minimum 12 points, y span >=0.75m; require two consecutive qualifying scans. Close state remains for 2 seconds from last qualifying scan; newClose only on rising edge. Scan gap >0.65s resets close continuity. This is a heuristic, not proven vehicle classification or legal passing-distance measurement.
Approach detector: radius >1.5m and <=12m; geometric atan2 angle >=-125° and <0° (235°–360°); x<=5m. Horizontal bands step 0.2m, height 0.75m, x gaps <=0.85m, minimum 3 points, width >=0.25m and width >=0.85 times y spread. Deduplicate candidates within 0.35m y and 0.65m center x; max 30. Match prior shapes in history <=0.85s, elapsed >=0.04s, x overlap tolerance 0.7m; shape overlaps x interval [-1.5,1.5]; y<0; velocity in +y 0.6–35m/s; ETA to y=-1.5 in [-0.2,8]s. Score width times proximity weight (1.35 down to 0.70 as distance from y-axis grows to 5m). Evidence window 0.65s, >=2 samples, >=0.08s span, weight sum >=2. Median speed then exponential smoothing (0.65 old +0.35 new). Warning renews for 2s; displayed speed converted to km/h. Earlier desired persistence >=1s is NOT what this source implements. Earlier discussion about 225° boundary is also not the actual code. Live and replay use same ESP32 detector.

## 8. Lights, camera triggers, viewer behavior
Approach evidence renews LED warning for 2 seconds, red pulses 350ms on/350ms off; current on pulse finishes. Manual C command tests light. **Important implementation mismatch:** `processScan` starts photo burst on `newClose`, but calls `extendWarning(false)` only on `renewWarning` from approach detector. Close detection alone does not explicitly trigger lights in recovered v1.13, despite original requirement. Do not claim otherwise or silently change this behavior.
Close event triggers 10 photo requests, first due 500ms after event and following spaced 500ms apart; actual rate depends on capture/radio/USB latency. Active burst not restarted by additional close event. Camera test requests single image, queued after active burst. Separate FreeRTOS capture/send tasks and serial mutex avoid concurrent output corruption. Successful close burst images saved under `~/Downloads/LiDAR_<timestamp>_Burst_<number>`; debug photo preview only. Close image preview five seconds after latest photo, manual one second. Browser server binds localhost on dynamically chosen port. GET /, /state, /latest?<id>; POST /play, /stop, /photo, /lights requires X-Lidar-Test: 1. Replay file <=64MB. `lidar_core.py` contains recording loader and geometry helpers; `scan_wire.py` encodes frames; `viewer.py` handles serial, replay, bridge, HTTP; `index.html` renders UI. No recording dataset included in backup.

## 9. Original project objectives versus delivered implementation
Goal: detect approaching vehicles from behind while cycling on left side of Irish roads, provide early warning, identify close passes within 1.5m and take photos. User requested noisy line/shape heuristic rather than ML, 12m view, right boundary x=5m, 2s warning memory, speed estimate, consistent live/replay, debug light/camera controls. Original intent includes flashing on close pass; mismatch above. OLED standalone dashboard added later for photo upload testing. Future desired integration is not specified. Do not change geometry based solely on conversational quadrant labels; calibrate physical sensor mount. Do not rely on this prototype as a sole road-safety device, nor claim validated performance, plate recognition, or legal evidence accuracy.

## 10. Suggested test sequence (not executed)
1. Verify ports and board identities; preserve current flash if needed before changing anything.
2. OLED: check supply, common ground, actual GPIO15/22, I2C address; observe READY and physical checkerboard/image. If NACK persists, run I2C scanner separately only after preserving firmware.
3. Camera: power independently, inspect startup logs, use relay camera test; verify JPEG decode and MAC/channel agreement.
4. Relay: query V, verify 1.13 and radio OK, test C and observe full red pulse; test P and verify preview.
5. LiDAR: connect distinct USB ports, verify raw packet/sweep flow, orientation with stationary target, live and recorded scene behavior, approaching speed, close boundary, multiple close events and burst save path.
6. Validate intended versus actual detection thresholds and close-only LED mismatch before field use. Start stationary and controlled; avoid unsafe road testing.

## 11. Instructions for another LLM
Read this master README, hardware_manifest.json and FILE_INDEX.json first. Then inspect the exact relevant source. Preserve all versioned alternatives, avoid replacing camera with recovery firmware accidentally, avoid assuming installed firmware equals recovered source, and distinguish actual code values from intended behavior. Ask for physical confirmation only when required (board, ports, voltage, pinout, flash status). For any code changes, bump release version, record changelog, provide reproducible compile/upload steps, and test live/replay parity. Do not silently resolve discrepancies by modifying firmware. The current release is a documentation-only backup; original per-package READMEs remain intact.
