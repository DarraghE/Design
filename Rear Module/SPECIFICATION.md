# Rear Module: Specification

| | |
|---|---|
| **Module** | Rear Module: LiDAR detection, warning lights and camera trigger (called the "relay" in the backup) |
| **Runs on** | FireBeetle ESP32 |
| **Code file** | `rear_module.ino` (Arduino sketch, firmware version **1.13**) |
| **Status** | **Firmware written; compiles (ESP32 core 3.2.1); not confirmed working on hardware.** v1.13 has never been confirmed flashed: the last recorded build (22 Nov 2026) failed on the Mac before upload. The LiDAR currently has to go through a computer (see section 4.3). |
| **Source** | v1.13 relay from the firmware backup of 25 Nov 2026: `Archive/Firmware_Backup_v2_2026-11-25/LiDAR_v1.13_Complete_Setup/ESP32_Relay/Relay/`. Its five files were merged into one, with no logic changes. |

## 1. What this module does

The Rear Module watches the road **behind and to the right** of the bike with a 2D LiDAR (STL-19P). On every full scan it runs two detectors:

| Detector | Looks for | Result |
|----------|-----------|--------|
| **Approach** | A vehicle-sized shape 1.5–12 m away, behind the bike, getting closer at 0.6–35 m/s (2–126 km/h) | Red light pulses for 2 s (renewed while the vehicle keeps coming), plus a speed estimate in km/h |
| **Close pass** | A long object within 1.5 m on the bike's right side, seen in 2 scans in a row | Asks the Camera Module for **10 photos**, 0.5 s apart, and sends them to the computer |

It also answers test commands from a computer: test the lights, take one photo, report the version, reset the detectors.

```
 STL-19P LiDAR ──USB 230400──▶ Mac (Tools/LiDAR Viewer) ──USB 460800──▶ Rear Module (FireBeetle ESP32)
                                     ▲                                      │  │
                                     └──── scans, detections, JPEGs ────────┘  ├─ GPIO2 ──▶ WS2813 LED strip (30 LEDs)
                                                                               └─ ESP-NOW ch 6 ◀──▶ Camera Module
```

The detection runs **on the ESP32**. The Mac only forwards bytes and displays results. Live data and replayed recordings go through the same detector code.

### 1.1 Coordinates

Metres, seen from above, with the LiDAR at (0, 0): **x = to the right, y = forward** (negative y = behind).
- **Raw sensor angle** (the LiDAR's own angle, used by the close detector): x = r·sin(angle), y = r·cos(angle). So 90° is straight right and 180° is straight behind.
- **Geometric angle** (`atan2(y, x)`, used by the approach detector): 0° is right, 90° is forward, −90° (270°) is behind.

This assumes the LiDAR is mounted the way it was when the code was written. **Calibrate against the real mount** with a stationary object before trusting left and right.

### 1.2 Close-pass detector (`Detector::detectClosePass`)

1. Keep points with **raw angle 55° to <180°** (front-right, round through the right side, to behind) and **distance 0.15–1.5 m**.
2. Sort them front to back (by y). Split into groups wherever two neighbours are more than **0.55 m** apart.
3. A group counts if it has **at least 12 points** and is **at least 0.75 m long**. The longest such group is the "close object".
4. A close object must be seen in **2 scans in a row**. Then the close state stays on for **2 s** after the last such scan.
5. Photos are triggered on the **start** of a close state only (`newClose`), not on every scan.
6. If scans stop for more than **0.65 s**, the 2-in-a-row count restarts.

### 1.3 Approach detector (`Detector::detectApproach`)

1. Keep points **1.5–12 m away**, with **geometric angle −125° to 0°** (behind and to the right), and **x ≤ 5 m**. At least 3 points are needed.
2. Cut the points into horizontal bands **0.75 m tall**, stepping **0.2 m**. In each band, group points that are no more than **0.85 m** apart across (in x). A group is a candidate shape if it has **at least 3 points**, is **at least 0.25 m wide**, and its width is at least **0.85×** its front-to-back depth. Shapes are de-duplicated (within 0.35 m in y and 0.65 m in centre x), up to 30.
3. A shape must be **behind the bike** (y < 0) and overlap the strip **1.5 m either side** of the bike's line (x from −1.5 to +1.5).
4. Match it with shapes from scans in the last **0.85 s** (at least 0.04 s old, x ranges overlapping within 0.7 m). Speed = change in y ÷ time. Keep matches with speed **0.6–35 m/s** that would reach 1.5 m behind the bike within **−0.2 to 8 s**.
5. Best match = largest width × weight. The weight is 1.35 for shapes on the bike's line, falling to 0.70 at 5 m to the side.
6. **Warning:** over the last **0.65 s**, at least **2** matches, at least **0.08 s** apart, with weights adding up to **≥ 2**. Then the speed = median of those speeds, smoothed (0.65 × old + 0.35 × new), and the warning is renewed for **2 s**.
7. When no warning is active, the speed resets to 0.

These are hand-tuned rules, not machine learning. **They haven't been validated on the road.** They aren't proof of a vehicle or a legal passing distance.

### 1.4 Lights (`updateLights`)

- While a warning is active (the 2 s since the last renewal), the strip shows **red pulses: 350 ms on, 350 ms off**. All 30 LEDs are red (255, 0, 0) at brightness **20/255**. A pulse that has started always finishes.
- The `C` test command starts the same 2 s of pulses and reports `OK LED PULSE GPIO2` after the first pulse.
- **Only the approach detector turns the lights on.** A close pass alone takes photos but doesn't flash the lights (see section 6).

### 1.5 Photos (`captureBurst`, `capture`)

- A close pass starts one **burst of 10 photos**. The first request is due 0.5 s after the close pass, then every 0.5 s. Slow transfers delay later shots.
- A close pass during a running burst doesn't start another burst. It's counted and reported as `INFO … during an active photo burst`.
- The `P` test command takes one photo, after any running burst.
- Each photo is fetched from the Camera Module over ESP-NOW (section 2.2) and sent to the computer as `JPEG …` (section 2.1).
- Two FreeRTOS tasks (capture and send) plus a serial lock stop text and photo bytes getting mixed up on USB.

## 2. Interfaces

### 2.1 USB serial to the computer (460800 baud). Contract with `Tools/LiDAR Viewer/`

**Into the module:**

| Input | Meaning |
|-------|---------|
| `V` | Report version → `READY 1.13 RADIO OK` (or `… RADIO ERROR` if ESP-NOW failed to start) |
| `C` | Test lights |
| `P` | Test camera (one photo) |
| `R` | Reset the detectors |
| Framed data | Starts and ends with `0x7E`. Inside, `0x7E` and `0x7D` are sent as `0x7D` followed by (byte XOR `0x20`) |

Frame contents, little-endian:

| Bytes | Field |
|-------|-------|
| 0–3 | uint32 scan ID |
| 4–7 | uint32 time since the start of the recording, in ms |
| 8–9 | uint16 point count, max 1024. **65535 = raw LiDAR bytes follow instead of points** |
| 10… | Per point: int16 x (mm), int16 y (mm), uint16 raw angle (hundredths of a degree; 65535 = unknown) |

A half-received frame is dropped if no byte arrives for 250 ms.

**Out of the module.** Text lines, some followed by binary data. Read the binary by its stated length, never by lines.

| Line | Meaning |
|------|---------|
| `READY 1.13 RADIO OK` / `READY 1.13 RADIO ERROR` | Reply to `V` |
| `OK LED PULSE GPIO2` | First light pulse after `C` has finished |
| `EVENT <close> <approach>` | Number of new close passes / new approach warnings since the last report |
| `SCAN <id> <close 0/1> <incoming 0/1> <km/h> [+i …] [-j …]` | Result for a replayed scan. `+i` = point i is part of a close pass, `-j` = point j is part of an approaching vehicle |
| `LIVE <ms> <close> <incoming> <km/h> <count>` + count × 5 bytes | Result for a live scan. Per point: int16 x mm, int16 y mm, uint8 class (0 normal, 1 close, 2 approach) |
| `JPEG <size> <burst> <shot>` + size bytes | A photo. Burst 0 = test photo |
| `ERR …` / `INFO …` | Problems and notes (e.g. `ERR Burst 3 photo 4/10: Camera did not answer ESP-NOW`) |

### 2.2 ESP-NOW to the Camera Module. Contract with `Camera Module/camera_module.ino`

| | |
|---|---|
| Radio | ESP-NOW, Wi-Fi station mode, **channel 6**, Wi-Fi sleep off, **no encryption** |
| Camera address | `cameraMac = 3C:8A:1F:D7:B1:24` (hard-coded; **change it if the camera board is replaced**). Replies from other addresses are ignored |
| Request (8 bytes) | uint32 token (random, never 0), uint32 offset (multiple of 224) |
| Reply (236 bytes) | the request echoed, uint32 total JPEG size (`0xFFFFFFFF` = capture failed), 224 bytes of JPEG |
| Timing | Offset 0 (which starts the capture): wait up to 600 ms, 4 tries. Later chunks: 50 ms, 4 tries. Whole photo within 8 s. Max photo size 128 KB |

`relayMac = D4:8C:49:FA:75:48` is also defined but unused by this sketch. The old recovery camera firmware in the archive only answers that address.

### 2.3 Phone App

**None yet.** The Rear Module has no Bluetooth link to the phone. See section 6 for the planned "main module" role.

## 3. Features

| Feature | Status | Where in `rear_module.ino` |
|---------|--------|----------------------------|
| STL-19P decoding (CRC check, sweep assembly, 55–215° and 30–12000 mm crop) | Written, not hardware-tested in this form | `class Stl19p` |
| Replay of recorded scans from the computer | Written | `class ScanInput`, `receiveInput` |
| Close-pass detection | Written, thresholds not validated | `Detector::detectClosePass` |
| Approach detection + speed | Written, thresholds not validated | `Detector::detectApproach` |
| Warning lights on approach | Written | `extendWarning`, `updateLights` |
| Warning lights on close pass | **Not implemented** (was intended) | — |
| 10-photo burst on close pass via the Camera Module | Written | `requestPhotoBurst`, `captureBurst`, `capture` |
| Test commands V / C / P / R | Written | `receiveInput` |
| Direct LiDAR wiring to the ESP32 (UART2) | Written, **off by default**, no pin chosen | `#ifdef LIDAR_RX_PIN` |
| Link to the Phone App | Not started | — |

## 4. Hardware and wiring

All pin numbers are the **chip's GPIO numbers**, not board labels. Check the board's pinout before wiring. Disconnect power before rewiring. Never connect 5 V signals straight to an ESP32 pin.

### 4.1 Board

| | |
|---|---|
| Board | FireBeetle ESP32 (built as generic "ESP32 Dev Module") |
| USB | Serial at **460800 baud** to the computer; uploads at 115200. Last seen as `/dev/cu.wchusbserial14110` on the Mac (check, don't assume) |
| Power | Not documented |

### 4.2 LED strip

| Strip pin | Connect to |
|-----------|-----------|
| DI (data in) | **GPIO2** (FireBeetle label **D9**), ideally through a 3.3 V → 5 V logic level shifter |
| +5V | **External regulated 5 V supply**, not the ESP32 |
| GND | Supply ground **and** ESP32 GND (common ground) |
| DO | Nothing (it's the output to a next strip) |
| BI | **Unknown.** Check the strip's datasheet |

Strip: WS2813 (as reported by the team), **30 LEDs**, GRB, 800 kHz. The strip's real length, supply rating and level shifter aren't documented. GPIO2 is a boot-mode pin on the ESP32: if uploads fail with the strip connected, unplug the data wire while uploading.

### 4.3 LiDAR

| | |
|---|---|
| Sensor | STL-19P (D500-compatible protocol), 2D, **230400 baud, 8N1** |
| Packets | 47 bytes: header `0x54 0x2C`, speed, start angle, 12 × (distance mm, intensity), end angle, timestamp, CRC8 (polynomial `0x4D`) |
| **Current connection** | **Plugged into the Mac by USB**. The Mac forwards the raw bytes to the Rear Module (`Tools/LiDAR Viewer/viewer.py --lidar`) |
| Optional direct connection | Build with `LIDAR_RX_PIN=<gpio>`. Sensor TX → that GPIO (UART2 RX), common ground, matching voltage levels. ESP32 TX is not used. **No pin has been chosen or tested.** Use only one LiDAR input at a time |
| Sensor power and connector pinout | Not documented |

### 4.4 Camera

Wireless (ESP-NOW), no wires. See `Camera Module/SPECIFICATION.md`.

## 5. Build and upload

| | |
|---|---|
| Board setting | **ESP32 Dev Module** (`esp32:esp32:esp32`), upload speed **115200** |
| ESP32 core | **3.x** (compile-checked with **3.2.1**; earlier builds used 3.2.0/3.2.1). 2.x won't work (ESP-NOW callback API) |
| Libraries | **Adafruit NeoPixel** (1.15.5) |

- **Arduino IDE:** open `rear_module.ino` (click OK when it offers to make a `rear_module` folder), set the board and port, then Upload.
- **Mac script:** `bash Tools/flash_module.command rear PORT`. For direct LiDAR wiring: `LIDAR_RX_PIN=16 bash Tools/flash_module.command rear PORT` (16 is only an example).
- **Check it works:** open the LiDAR Viewer (or a serial monitor at 460800) and send `V`. You should see `READY 1.13 RADIO OK`. Then `C` → the strip pulses red and the module reports `OK LED PULSE GPIO2`. Then `P` → a photo arrives (needs the Camera Module powered).
- If a Mac build fails with `Failed to extract libcrypto.1.1.dylib`, nothing was uploaded. Check free disk space with `df -h "$HOME" "${TMPDIR:-/tmp}"`.

## 6. Known gaps and next steps

1. **Close pass doesn't flash the lights.** The original goal included flashing on a close pass, but the code only lights up on approach warnings. It hasn't been changed on purpose, so the decision is the team's. The fix would be to also call `extendWarning(false)` when `newClose` is set in `processScan`.
2. **Warning persistence:** the team earlier asked for warnings that last at least 1 s. The code renews a 2 s warning on fresh evidence, which is not the same rule. Confirm which is wanted.
3. **Never confirmed on hardware:** v1.13's flash, the lights, the camera link and live LiDAR are all untested. The last recorded build failed on the Mac (libcrypto extraction) before uploading.
4. **Needs a computer:** the LiDAR currently goes through a Mac. For riding, the LiDAR must be wired to the ESP32 (`LIDAR_RX_PIN`) and photos must go somewhere other than the Mac's Downloads folder.
5. **Thresholds not validated:** test stationary and in controlled conditions first (suggested order: ports → version → lights → camera → LiDAR orientation → replay → live). Don't do unsafe road tests. **This must not be the only safety device on a bike.**
6. **ESP-NOW is unencrypted and unauthenticated.** That's fine for a prototype, not for anything adversarial.
7. **Planned role as the "main module" (not decided):** the team's plan is for the phone to send ready-drawn navigation images over Bluetooth to a main module, which passes them straight on to the Front Module. This board already runs ESP-NOW, so it's a candidate, but which board is the main module hasn't been decided.
8. Power supply, enclosure, LED strip length and BI wiring, and the LiDAR connector pinout are all undocumented.

## 7. Notes for AI assistants

- Detection logic is the `lidar::Detector` class. Every number in sections 1.2–1.4 comes from that code. If you change a number, update this file.
- **Don't silently "fix" the known issues in section 6** (close-pass lights, persistence). They are decisions for the team. Report them instead.
- The version string `READY 1.13 RADIO` is checked by `Tools/LiDAR Viewer/viewer.py`. If you bump the firmware version, update the viewer in the same commit.
- The USB protocol (section 2.1) must match `Tools/LiDAR Viewer/scan_wire.py` and `viewer.py`. The ESP-NOW packets (section 2.2) must match `Camera Module/camera_module.ino` (`Request` 8 bytes, `Reply` 236 bytes, channel 6).
- Keep live and replay going through the same detector.
- The sketch must stay one file. The original five-file version is in `Archive/Firmware_Backup_v2_2026-11-25/`.
