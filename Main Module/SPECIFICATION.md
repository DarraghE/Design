# Main Module: Specification

| | |
|---|---|
| **Module** | Main Module: the hub at the back of the bike. It handles LiDAR detection, the warning lights and the camera trigger, and is the phone's main link (called the "relay" / "rear" in older notes) |
| **Runs on** | FireBeetle ESP32, mounted at the back of the bike |
| **Code file** | `main_module.ino` (Arduino sketch, firmware **v1.28**) |
| **Status** | **v1.28 written, not yet flashed; the board runs v1.26.** v1.28 = v1.26 + clear error replies from the USB image debugger (section 2.1); nothing else changed, so reflashing is only needed for that. v1.26 is installed (25 Nov 2026, from the team's `BikeNav_Installed_Firmware_2026-11-25` archive): the board answered with the v1.26 handshake over USB. A USB image transfer then timed out at byte zero (section 6, item 2). **The Bluetooth link to the phone hasn't been tried yet.** Compiles (ESP32 core 3.3.8, NimBLE-Arduino 2.5.2); its Bluetooth bytes are checked against the phone app's decoder on a computer (`Tools/Phone Link Test/`). |
| **Source** | `Rear_v1.21_reconstructed/Relay/` from the team's `BikeNav_PreBluetooth_Source` bundle (25 Nov 2026): `Relay.ino` and `TestImage.h`, plus `Detection.h`, `ScanInput.h`, `Stl19p.h` and `Radio.h`, which are byte-identical to v1.13's (`Archive/Firmware_Backup_v2_2026-11-25/LiDAR_v1.13_Complete_Setup/ESP32_Relay/Relay/`). Merged into one file; every v1.25 change is marked `v1.25:`; the phone link is new code in the last two sections. **v1.26** (Oscar, 25 Nov 2026) = v1.25 + a USB image debugger (`readImageByte`, section 2.1) and `#define LIDAR_RX_PIN 16` as the first line. v1.28 changes to it are marked `v1.28:` (v1.27 is the number of Oscar's Chrome page, so it was skipped). The version numbers 1.22–1.24 were used for phone experiments on Oscar's Mac that aren't in this repo; v1.25 is unrelated to them. |

## 1. What this module does

The Main Module is the centre of the bike system. It sits at the back of the bike with the LiDAR and the warning lights. It is also the module the **phone talks to**:

| Role | Status |
|------|--------|
| LiDAR detection, warning lights, close-pass photos (below) | Firmware written (v1.21 logic) |
| Commands the Camera Module over ESP-NOW | Firmware written |
| Sends each camera photo to the Front Module as a JPEG (v1.20/v1.21 feature) | Firmware written |
| **Main link to the Phone App** (Bluetooth LE, section 1.6) | **Written (v1.25), not hardware-tested** |
| **Forwards navigation screens from the phone straight to the Front Module** | **Written (v1.25), not hardware-tested** |

### What the current firmware does

The Main Module watches the road **behind and to the right** of the bike with a 2D LiDAR (STL-19P). On every full scan it runs two detectors:

| Detector | Looks for | Result (with automatic actions on) |
|----------|-----------|--------|
| **Approach** | A vehicle-sized shape 1.5–12 m away, behind the bike, getting closer at 0.6–35 m/s (2–126 km/h) | Red light flashes for 2 s (renewed while the vehicle keeps coming), plus a speed estimate in km/h |
| **Close pass** | A long object within 1.5 m on the bike's right side, seen in 2 scans in a row | Alerts the phone at once. Red light flashes while the close state lasts. Asks the Camera Module for **5 photos**, 0.5 s apart; sends them to the computer, to the Front Module (JPEG) and, with a LiDAR clip, to the phone |

**Automatic actions** (lights and photos) are **on from power-up** since v1.25 (`automaticActions = true`). In v1.20/v1.21 they were off until a computer sent `A`. `M` still switches them off. Detection, the `EVENT` counts and the phone alert happen either way.

At power-up the strip flashes red 3 times (startup light test). The module also answers test commands from a computer (section 2.1).

```
 STL-19P LiDAR ──UART2 GPIO16 (LIDAR_RX_PIN=16)──▶ Main Module (FireBeetle ESP32)
   (or via a Mac: Tools/LiDAR Viewer, USB 460800)    │  │  │  │
                                                     │  │  │  └─ GPIO2 ──▶ WS2813 LED strip (30 LEDs)
                                                     │  │  └─ ESP-NOW ch 6 ◀──▶ Camera Module (photos)
                                                     │  └─ ESP-NOW ch 6 ───▶ Front Module (phone screens, unicast;
 Phone (Bike Nav app) ◀═══ Bluetooth LE ═════════════┘                         camera JPEGs, broadcast)
                          "BikeNav Main"
```

The detection runs **on the ESP32**. Live data and replayed recordings go through the same detector code.

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
5. Photos and the phone alert are triggered on the **start** of a close state only (`newClose`), not on every scan.
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

### 1.4 Lights (`extendWarning`, `serviceLed`, `directLedShow`)

- With automatic actions on, every scan with a renewed approach warning **or** an active close state starts/extends a **2 s** warning. While it lasts, all 30 LEDs go red (255, 0, 0) at brightness **80/255** and off again, **500 ms on, 500 ms off**.
- `C` (and power-up) runs **3 on/off cycles** without blocking the loop, then reports `OK LED PULSE GPIO2`.
- The strip is driven by `directLedShow()`: the pixel bytes are bit-banged on GPIO2 with the CPU cycle counter (1.25 µs per bit), interrupts on that core held off for about 1 ms per frame. This replaced the NeoPixel library's `show()` in v1.21. The NeoPixel library is still used to hold and scale the pixel colours.
- `manualLedTest()` (blocking, with `DEBUG`-style lines) is still in the file but is no longer called.

### 1.5 Photos (`captureBurst`, `capture`, `forwardDisplay`)

- A close pass (automatic actions on) starts one **burst of 5 photos** (10 before v1.25). The first request is due 0.5 s after the close pass, then every 0.5 s. Slow transfers delay later shots.
- A close pass during a running burst doesn't start another burst. It's counted and reported as `INFO … during an active photo burst`.
- `P` takes one test photo, after any running burst. `T` sends the built-in test JPEG (`debugJpeg`, `TestImage.h`) to the Front Module.
- Each photo is fetched from the Camera Module over ESP-NOW (section 2.2), then **broadcast to the Front Module** as 236-byte JPEG packets (`forwardDisplay`, about 4 ms apart, no delivery check), then sent to the computer as `JPEG …`. Since v1.25, burst photos are then kept in memory for the phone package (section 1.6); test photos are freed.
- FreeRTOS tasks: `capturePhotos` and `sendPhotos` (8 KB stacks), `diagWatch` (prints `DEBUG WATCH` every 2 s). A serial lock stops text and photo bytes getting mixed up on USB.

### 1.6 Phone link (new in v1.25, `namespace phone`)

The phone is the Bluetooth central; the Main Module is the peripheral. The byte layouts are the contract in `Phone App/SPECIFICATION.md` section 3 (summary in section 2.3 below).

| What | How the firmware does it |
|------|---------------------------|
| Advertising | Name **`BikeNav Main`** (in the scan response), service UUID in the advertisement. Restarts by itself when the phone disconnects. No PIN, no bonding. One phone at a time |
| Screens | Every phone write starting with `0x10` is sent **unchanged** to the Front Module (`phone::frontMac`, unicast) by ESP-NOW, straight from the Bluetooth callback. Nothing is decoded or stored |
| Hello / status | `0x40` from the phone → `0xC1` status. Then a status every **10 s** while connected (table in section 2.3) |
| Close-pass alert | At `newClose`, `0xA1` is sent at once (if connected), **even with automatic actions off**: event number (+1 per close pass since power-on), the closest close-pass point in mm, and the approach speed if a vehicle was being tracked in the last **5 s** (else `0xFFFF`) |
| LiDAR clip | Every processed scan (live or replayed) goes into a ring of **40 scans** (~4 s at 10 scans/s), at most **200 points each**: every close-pass/approach point, then evenly spaced others. **1.5 s** after the close pass, the scans from **2 s before to 1.5 s after** are copied into the package |
| Photos | The 5 burst photos (after their USB output) go through a queue to the package. Failed shots are counted but not included. The package waits for the 5th shot, or **20 s** at most. With automatic actions off there is no burst, so the package carries the clip only |
| Package | Built in the `CPK1` layout and sent as `0xB1` start, `0xB2` parts (MTU − 10 bytes each; up to 16 parts per `loop()`, retrying when the Bluetooth queue is full), `0xB3` end with the CRC-32. Photos and clip are sent from their own buffers, never copied into one block |
| Confirmation | `0x30` status 0 → memory freed. Status 1, or no answer within **5 s** → the whole package again, at most **2 more times**, then dropped (`ERR` on USB) |
| Phone not connected | Only after the phone has connected and sent hello is a package sent. A package that can't be delivered waits; a newer close pass replaces it (only one is kept). A close pass while another is still being collected gets an alert only |
| Simulated close pass | USB command `X`: alert + photo burst + package with no distance (`0xFFFF`), for bench tests without LiDAR |

## 2. Interfaces

### 2.1 USB serial to the computer (460800 baud)

**Into the module:**

| Input | Meaning |
|-------|---------|
| `V` | Report versions: `READY 1.20 RADIO OK`/`ERROR`, `INFO DISPLAY RELAY READY`/`ERROR`, `INFO PHONE LINK 1.28 BLUETOOTH READY`/`ERROR`, `USB IMAGE READY 1.26` (unchanged in v1.28: Oscar's tools check it). The first line still says 1.20 (as v1.21 did) so existing viewers recognise the board |
| `C` | Test lights (3 flashes) |
| `A` / `M` | Automatic actions (lights + photos) on / off. On at power-up since v1.25 |
| `P` | Test camera (one photo; also shown on the Front Module) |
| `T` | Send the built-in test JPEG to the Front Module |
| `R` | Reset the detectors |
| `X` | **v1.25:** fake a close pass (phone alert, 5-photo burst, package to the phone) to test the phone link |
| `@` + 472 hex digits + `\n` | **v1.26 USB image debugger:** one 236-byte JPEG packet (camera `Reply` layout: token, offset, total, 224 bytes) as **lowercase** hex. If valid (token ≠ 0, total ≤ 128 KB, offset < total and a multiple of 224) it is broadcast to the Front Module and the module prints `USBFRAME <token> <offset> <esp_err>`. **v1.28:** `\r` is ignored (CRLF line endings work), and a rejected line gets `ERR USB IMAGE <reason> (<n> hex digits)`: bad character (uppercase hex isn't accepted, because a stray `A` or `C` would be a command), wrong length, bad packet, or line timed out (2 s). In v1.26 all of these were dropped silently. Used by Oscar's v1.27 Chrome page on the Mac (not in this repo) |
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
| `READY 1.20 RADIO OK` / `… ERROR` + two `INFO` lines | Reply to `V` (above) |
| `OK LED PULSE GPIO2` | The 3 test flashes after `C` (or power-up) have finished |
| `EVENT <close> <approach>` | Number of new close passes / new approach warnings since the last report |
| `SCAN <id> <close 0/1> <incoming 0/1> <km/h> [+i …] [-j …]` | Result for a replayed scan. `+i` = point i is part of a close pass, `-j` = point j is part of an approaching vehicle |
| `LIVE <ms> <close> <incoming> <km/h> <count>` + count × 5 bytes | Result for a live scan, **at most every 200 ms**. Per point: int16 x mm, int16 y mm, uint8 class (0 normal, 1 close, 2 approach) |
| `JPEG <size> <burst> <shot>` + size bytes | A photo. Burst 0 = test photo |
| `DEBUG HEALTH <lidarBytes> <scans> <freeHeap> <burstActive> <radio> <displayRadio>` | Every second |
| `DEBUG WATCH <loopCount> <loopStage>` | Every 2 s from a separate task: shows a stalled loop |
| `DEBUG CAMERA_…`, `DEBUG DISPLAY_BEGIN/END …`, `DEBUG DISPLAY_ACK …` | Photo and Front Module transfer progress |
| `USB IMAGE READY 1.26` | After `V`: the USB image debugger is present |
| `USBFRAME <token> <offset> <result>` | One `@` image line was forwarded to the Front Module (`result` 0 = queued by ESP-NOW) |
| `ERR …` / `INFO …` | Problems and notes (e.g. `ERR Burst 3 photo 4/5: Camera did not answer ESP-NOW`, `INFO AUTOMATIC ACTIONS ON`). Phone link: `INFO Phone connected` / `disconnected`, `INFO Close pass N ready for the phone: P photos, B bytes`, `INFO Close pass N delivered to the phone`, `INFO … sending again`, `ERR Close pass N dropped: …` |

`Tools/LiDAR Viewer/` in this repo is the **v1.13 viewer**. It recognises `READY 1.20 RADIO`, ignores `DEBUG` lines and can't send `A`, `M` or `T`. The newer viewer Oscar used with v1.20/v1.21 isn't in the repo (section 6).

### 2.2 ESP-NOW (channel 6, Wi-Fi station mode, no encryption)

**Camera Module** (contract with `Camera Module/camera_module.ino`):

| | |
|---|---|
| Camera address | `cameraMac = 3C:8A:1F:D7:B1:24` (hard-coded; **change it if the camera board is replaced**) |
| Request (8 bytes) | uint32 token (random, never 0), uint32 offset (multiple of 224) |
| Reply (236 bytes) | the request echoed, uint32 total JPEG size (`0xFFFFFFFF` = capture failed), 224 bytes of JPEG |
| Timing | Offset 0 (which starts the capture): wait up to 600 ms, 4 tries. Later chunks: 50 ms, 4 tries. Whole photo within 8 s. Max photo size 128 KB |

**Front Module:**

| Traffic | Format | Since |
|---------|--------|-------|
| Camera photos (Main → Front) | Same 236-byte layout as the camera reply (token, offset, total, 224 bytes), **broadcast** to `FF:FF:FF:FF:FF:FF` | v1.20 |
| Display acks (Front → Main) | 16 bytes: magic `0x44495350` ("DISP"), token, size, status; printed as `DEBUG DISPLAY_ACK` | v1.20 |
| Phone screens (Main → Front) | Each phone `0x10` packet, unchanged (≤ 185 bytes), **unicast** to `phone::frontMac = 8C:94:DF:6C:BC:80` (**change it if the front board is replaced**). An ESP-NOW send callback records delivery for the status message | v1.25 |

The Front Module's current receiver (v1.19/v1.20, in the team's source bundle) only accepts 236-byte packets from the Main Module, so it ignores the phone screens until its phone-frame code is written (`Phone App/SPECIFICATION.md` section 9).

`relayMac = D4:8C:49:FA:75:48` is this board's own address (used by the Front and the old recovery camera firmware to recognise it).

### 2.3 Bluetooth LE to the Phone App. Contract with `Phone App/SPECIFICATION.md` section 3

Implemented in v1.25 (`namespace phone`; byte builders in `namespace wire`). All numbers little-endian. Protocol version 1.

| | UUID / value | |
|---|---|---|
| Service | `6e3b2000-8f7a-4c1e-9b52-2d1c0b7a1f10` | advertised |
| Phone → Main | `6e3b2001-8f7a-4c1e-9b52-2d1c0b7a1f10` | Write, Write Without Response |
| Main → Phone | `6e3b2002-8f7a-4c1e-9b52-2d1c0b7a1f10` | Notify |
| Name | `BikeNav Main` | scan response |
| MTU | up to 517 offered; the phone asks for 512 | requests a 7.5–15 ms connection interval |

| Message | Bytes | Firmware behaviour |
|---------|-------|--------------------|
| `0x10` screen part (in) | `[0x10][frame#][index][count][flags][data…]` | Forwarded unchanged to the Front Module |
| `0x40` hello (in) | `[0x40][version 1]` | Replies with `0xC1`; enables package sending on this connection |
| `0x30` ack (in) | `[0x30][transferId u16][status]` | 0 = delivered (free it), 1 = send again |
| `0xA1` alert (out) | `[0xA1][eventId u16][minDistanceMm u16][speedKmh×10 u16]` | Sent at once at a close pass |
| `0xC1` status (out) | `[0xC1][flags][battery 255][1][25]` | flags: bit 0 **LiDAR OK** = a scan in the last 1 s; bit 1 **camera OK** = ESP-NOW started and the last photo request didn't fail (OK before the first photo); bit 2 **Front OK** = the last phone screen sent to the Front Module was delivered, within the last 15 s |
| `0xB1`/`0xB2`/`0xB3` (out) | start / parts / end + CRC-32 | One close-pass package (`CPK1`, section 1.6) |

## 3. Features

| Feature | Status | Where in `main_module.ino` |
|---------|--------|----------------------------|
| STL-19P decoding (CRC check, sweep assembly, 55–215° and 30–12000 mm crop) | Written, not hardware-tested in this form | `class Stl19p` |
| Replay of recorded scans from the computer | Written | `class ScanInput`, `receiveInput` |
| Close-pass detection | Written, thresholds not validated | `Detector::detectClosePass` |
| Approach detection + speed | Written, thresholds not validated | `Detector::detectApproach` |
| Warning lights on approach **and close pass** (automatic actions) | Written (v1.21) | `processScan`, `extendWarning`, `serviceLed` |
| Bit-banged WS2813 driver on GPIO2 | Written (v1.21 reconstruction) | `directLedShow` |
| Startup light test (3 flashes) | Written (v1.21 reconstruction) | `loop` (`startupTest`) |
| Automatic actions on/off (`A`/`M`), **on at power-up** | Written (v1.25 default) | `automaticActions` |
| 5-photo burst on close pass via the Camera Module (10 before v1.25) | Written | `requestPhotoBurst`, `captureBurst`, `capture` |
| Photos and test JPEG to the Front Module (broadcast) | Written (v1.20) | `forwardDisplay`, `debugJpeg` |
| USB diagnostics (`DEBUG HEALTH/WATCH/CAMERA/DISPLAY`) | Written (v1.20) | `reportStatus`, `diagnosticWatch`, `debugLine` |
| Test commands V / C / A / M / P / T / R, and X (fake close pass, v1.25) | Written | `receiveInput` |
| USB image debugger (`@` hex lines → Front Module) | Written (v1.26), installed; first transfer timed out. Error replies added in v1.28 | `readImageByte`, `imageNibble`, `imageLineError` |
| Direct LiDAR wiring to the ESP32 (UART2) | Written, needs `LIDAR_RX_PIN=16` at build time (section 4.3) | `#ifdef LIDAR_RX_PIN` |
| Bluetooth LE server for the phone, hello/status | Written (v1.25), not hardware-tested | `phone::begin`, `phone::service`, `phone::sendStatus` |
| Forward phone screens to the Front Module | Written (v1.25), not hardware-tested | `phone::PhoneWrites::onWrite` |
| Close-pass alert to the phone | Written (v1.25), not hardware-tested | `phone::onClosePass` |
| LiDAR clip (2 s before to 1.5 s after) | Written (v1.25), bytes host-tested | `phone::onScan`, `phone::buildClip` |
| Close-pass package to the phone, CRC, resend | Written (v1.25), bytes host-tested | `phone::collect`, `phone::finalize`, `phone::transfer` |

## 4. Hardware and wiring

All pin numbers are the **chip's GPIO numbers**, not board labels. Check the board's pinout before wiring. Disconnect power before rewiring. Never connect 5 V signals straight to an ESP32 pin.

### 4.1 Board

| | |
|---|---|
| Board | FireBeetle ESP32 (built as generic "ESP32 Dev Module"). Station MAC `D4:8C:49:FA:75:48` |
| USB | Serial at **460800 baud** to the computer; uploads at 115200. Seen as `/dev/cu.wchusbserial1410` on Oscar's Mac (check, don't assume) |
| Power | Own supply; not documented further |

### 4.2 LED strip

| Strip pin | Connect to |
|-----------|-----------|
| DI (data in) | **GPIO2** (FireBeetle label **D9**), ideally through a 3.3 V → 5 V logic level shifter |
| +5V | **External regulated 5 V supply**, not the ESP32 |
| GND | Supply ground **and** ESP32 GND (common ground) |
| DO | Nothing (it's the output to a next strip) |
| BI | **Unknown.** Check the strip's datasheet |

Strip: WS2813 (as reported by the team), **30 LEDs**, GRB, 800 kHz, brightness 80/255 (about 0.2 A with all LEDs red). GPIO2 is a boot-mode pin on the ESP32: if uploads fail with the strip connected, unplug the data wire while uploading.

### 4.3 LiDAR

| | |
|---|---|
| Sensor | STL-19P (D500-compatible protocol), 2D, **230400 baud, 8N1** |
| Packets | 47 bytes: header `0x54 0x2C`, speed, start angle, 12 × (distance mm, intensity), end angle, timestamp, CRC8 (polynomial `0x4D`) |
| Wiring on the bike (team notes, 25 Nov 2026) | Sensor TX/DATA → **GPIO16** (UART2 RX), P5V → 5 V supply, GND → common ground. ESP32 TX not used. **Build with `LIDAR_RX_PIN=16`**, otherwise the sketch ignores GPIO16 |
| Bench alternative | LiDAR on a Mac by USB; the viewer forwards the raw bytes (`Tools/LiDAR Viewer/viewer.py --lidar`). Use only one LiDAR input at a time |
| Sensor connector pinout | Not documented |

### 4.4 Camera

Wireless (ESP-NOW), no wires. See `Camera Module/SPECIFICATION.md`.

## 5. Build and upload

| | |
|---|---|
| Board setting | **ESP32 Dev Module** (`esp32:esp32:esp32`), upload speed **115200**, Partition Scheme **Huge APP (3MB No OTA/1MB SPIFFS)** (`PartitionScheme=huge_app`) |
| LiDAR input | The file starts with `#define LIDAR_RX_PIN 16` (LiDAR wired to GPIO16). Delete that line to take LiDAR data from a computer over USB instead |
| ESP32 core | **3.x**. v1.25 compile-checked with **3.3.8** (Oscar's Mac had 3.2.0). 2.x won't work (ESP-NOW callback API). The ESP-NOW send callback changed in 3.3; the code handles both |
| Libraries | **Adafruit NeoPixel** (1.15.5), **NimBLE-Arduino** (2.5.2, by h2zero) |
| Size (v1.26) | 1,213,885 bytes of flash (38% of Huge APP; 93% of the default 1.3 MB), 111,992 bytes of static RAM (40 KB of it is the LiDAR clip ring) |

- **Arduino IDE:** open `main_module.ino` (click OK when it offers to make a `main_module` folder), set the board, partition scheme and port, then Upload.
- **Mac script:** `bash Tools/flash_module.command main PORT` (sets the partition scheme and installs both libraries).
- **Check it works:** serial monitor at 460800. At power-up the strip flashes 3 times. Send `V` → `READY 1.20 RADIO OK`, `INFO DISPLAY RELAY READY`, `INFO PHONE LINK 1.25 BLUETOOTH READY`, `USB IMAGE READY 1.26`. `DEBUG HEALTH` lines should show `lidarBytes` and `scans` rising with the LiDAR connected. `P` → a photo arrives (Camera Module powered).
- **Check the phone link:** section 8.
- If a Mac build fails with `Failed to extract libcrypto.1.1.dylib`, nothing was uploaded. Check free disk space with `df -h "$HOME" "${TMPDIR:-/tmp}"`.

## 6. Known gaps and next steps

1. **v1.21 is a reconstruction.** The team's bundle says `Rear_v1.21_reconstructed` was rebuilt from the documented v1.21 changes (GPIO2 driver, non-blocking light test, startup test) on the recovered v1.20 sketch. The original Mac source and its final upload were never verified, and the v1.20 sketch itself was recovered from a terminal listing with v1.19 headers. If Oscar's original v1.21 turns up, compare it with the `From Relay.ino` section and port the `v1.25:` changes (section 8.2).
2. **USB image transfer timed out at byte zero** (Oscar's test on v1.26, 25 Nov 2026). In v1.26 an uppercase hex digit, a `\r`, a wrong length or an invalid packet (e.g. token 0, or big-endian numbers) dropped the line **without any reply**, which looks exactly like a timeout. v1.28 accepts `\r` and replies `ERR USB IMAGE <reason>` to everything else, so the next try will say what's wrong. The Chrome page (not in the repo) must send lowercase hex of the little-endian 236-byte packet and wait for `USBFRAME` after each line. Also, a command letter sent while an `@` line is open ends that line and is lost.
3. **Phone link not run on hardware.** Compiles, and its bytes are checked against the app on a computer, but Bluetooth + ESP-NOW together, photo transfer speed with Bluetooth connected, memory with 5 photos held, and notification pacing are all untested on the board.
4. **Bit-banged LED driver + Bluetooth (untested):** `directLedShow` holds off interrupts on its core for about 1 ms per strip update. Wi-Fi and Bluetooth run on the other core, so this should be fine, but check that the strip colours are stable and the phone link doesn't drop while the lights flash.
5. **Camera photos to the Front Module:** every photo is still broadcast to the Front Module (v1.20 behaviour). Front v1.21 ignores them while the phone's screens are showing, so they no longer cover the directions; the broadcast still takes air time during a burst and reaches any ESP-NOW device on channel 6.
6. **Viewer out of date:** `Tools/LiDAR Viewer/` is the v1.13 viewer. It works with v1.25's USB lines but has no buttons for `A`/`M`/`T`, and expects bursts to be saved per photo. Add Oscar's newer viewer to `Tools/` if it is still needed.
7. **Warning persistence:** the team earlier asked for warnings that last at least 1 s. The code renews a 2 s warning on fresh evidence, which is not the same rule. Confirm which is wanted.
8. **Thresholds not validated:** test stationary and in controlled conditions first (suggested order: ports → version → lights → camera → LiDAR orientation → replay → live). Don't do unsafe road tests. **This must not be the only safety device on a bike.**
9. **ESP-NOW and Bluetooth are unencrypted and unauthenticated.** Any phone running the app can connect. That's fine for a prototype, not for anything adversarial.
10. **Memory:** about 216 KB of RAM is free at start-up before Wi-Fi and Bluetooth take their share. A package holds up to 5 JPEGs (QVGA, roughly 5-20 KB each) plus a clip of up to ~35 KB. Watch the free heap in the `DEBUG HEALTH` lines. If memory runs out, the clip is left out (`ERR … no memory for the LiDAR clip`) or the camera reports `Relay ran out of memory`.
11. **3.3 V data into a 5 V strip:** GPIO2 drives the WS2813 data line directly. That often works but can flicker or fail; a 3.3 V → 5 V level shifter fixes it.
12. **Front Module "NO PHONE":** the Main Module doesn't tell the Front Module when the phone disconnects; the Front Module must time out by itself (`Phone App/SPECIFICATION.md` section 9).
13. Oscar's v1.27 Chrome page (the USB image sender) and his newer LiDAR viewer aren't in the repo; add them to `Tools/` if they are kept.
14. Power supply, enclosure, LED strip length and BI wiring, and the LiDAR connector pinout are all undocumented.

## 7. Notes for AI assistants

- Detection logic is the `lidar::Detector` class. Every number in sections 1.2–1.4 comes from the code. If you change a number, update this file.
- **Don't silently "fix" the known issues in section 6.** They are decisions for the team. Report them instead.
- `READY 1.20 RADIO` is deliberately unchanged (viewers look for it). The other version lines are `INFO PHONE LINK 1.28` (firmware version, matches `phone::firmwareMajor/Minor` sent to the phone) and `USB IMAGE READY 1.26` (Oscar's USB tools check it). Change a version line only together with the tool that reads it.
- The board runs v1.26; this file is v1.28. Keep the Status line saying which version is installed.
- Bluetooth memory: on ESP32 core 3.3+, the core frees the Bluetooth memory at boot unless the `esp32-hal-bt-mem.h` flag is set. NimBLE-Arduino 2.5.2 sets it (through `esp32-hal-bt.h`); checked in the built firmware (`_setBtLibraryInUse`). If you change the Bluetooth library, check this again.
- **The `wire` namespace between `// BEGIN PHONE WIRE` and `// END PHONE WIRE` must stay plain C++** (no Arduino calls): `Tools/Phone Link Test/run_test.py` compiles it on a computer. After changing it or the package logic, run that test and `flutter test test/firmware_vectors_test.dart` in `Phone App/app`, and commit the regenerated `firmware_notifications.txt`.
- NimBLE callbacks run on the Bluetooth task: keep them short (forward screens, set atomics). Everything else happens in `phone::service()` from `loop()`.
- Don't set `WiFi.setSleep(false)` again: with Bluetooth on, ESP-IDF aborts unless Wi-Fi modem sleep is enabled.
- `forwardDisplay` has an explicit declaration after `Photo capture();`. Keep it: without it, the Arduino IDE generates a prototype above `struct Photo` and the one-file sketch doesn't compile.
- The ESP-NOW packets must match `Camera Module/camera_module.ino` (`Request` 8 bytes, `Reply` 236 bytes, channel 6) and the Front Module's receiver.
- Keep live and replay going through the same detector.
- The sketch must stay one file. The original multi-file versions: v1.13 in `Archive/Firmware_Backup_v2_2026-11-25/`; v1.20/v1.21 in the team's `BikeNav_PreBluetooth_Source` bundle (not in the repo).
- The phone link follows `Phone App/SPECIFICATION.md` section 3 exactly; the phone app's tests define the expected bytes. Change both sides in the same commit.

## 8. Phone link: bench test and porting

### 8.1 Bench test (no LiDAR needed)

1. v1.26 is already on the board (or flash it, section 5). Power the Camera Module.
2. Serial monitor at 460800: the strip flashes 3 times; send `V` → `INFO PHONE LINK 1.28 BLUETOOTH READY` (`1.25` on v1.26).
3. Phone: Bike Nav app → Main Module card → **BikeNav Main**. USB shows `INFO Phone connected`; the app shows firmware **1.28** (1.25 on v1.26) and ✓/✗ for LiDAR, camera and Front Module. The Front ✓ needs Front v1.21 (it reports delivery of the phone's screens).
4. Send `X`: the app shows the close-pass alert at once; after ~4-5 s USB shows `INFO Close pass N ready for the phone …` then `… delivered to the phone`, and the app's **Recordings** has the photos (and the LiDAR replay if scans are coming in).
5. With the LiDAR on GPIO16, a real close pass should do the same with a distance, and the strip should flash.
6. With Front v1.21 flashed: the handlebar shows the app's screen within 3 s of connecting; start a navigation (or **Simulate the ride**) and it follows each turn.

### 8.2 Moving the phone link onto other relay source

The phone link is self-contained. To put it on another version (e.g. Oscar's original v1.21):

1. Copy the last two sections of `main_module.ino` (`namespace wire` and `namespace phone`) to the end of the other sketch, plus `#include <cstdarg>`, `#include <NimBLEDevice.h>` and `#include <esp_idf_version.h>`.
2. Apply every change marked `v1.25:` (search for it): the `phone::` declarations after `Photo capture();`, `takenAt` in `Photo` and in `captureBurst`/`capturePhotos`, `burstShots = 5`, `automaticActions = true`, `phone::cameraResult` and `phone::keepPhoto` in `sendPhotos` (instead of `free`), `requestPhotoBurst` returning `bool`, `phone::onScan`/`phone::onClosePass` in `processScan`, the `X` command, the `INFO PHONE LINK` line, `phone::begin()` at the end of `setup()`, `phone::service()` in `loop()`, and `WiFi.setSleep(true)` in `startRadio`.
3. Use the Huge APP partition and install NimBLE-Arduino. Run the Phone Link Test (section 7).
