# Camera Module: Specification

| | |
|---|---|
| **Module** | Camera Module: photographs close passes on request |
| **Runs on** | M5Stack Timer Camera X (ESP32 with an OV3660 camera) |
| **Code file** | `camera_module.ino` (Arduino sketch) |
| **Status** | **Working firmware, probably on the camera already.** The backup links this exact code to a successful upload on 22 Nov 2026, but it was never read back from the board. It compiles (ESP32 core 3.2.1). |
| **Source** | `TimerCam_Camera_Only/Camera/Camera.ino` from the firmware backup of 25 Nov 2026, unchanged apart from an added header comment |

## 1. What this module does

The camera is a wireless photo server. It does nothing until another ESP32 asks it for a photo over **ESP-NOW**. Right now that's the Main Module, which asks for a burst of 10 photos after each close pass. With the phone link (`Phone App/SPECIFICATION.md` section 8), this becomes **5 photos**, which the Main Module sends on to the phone for number-plate reading. The camera firmware doesn't need to change for that. The camera then:

1. throws away the old buffered frame and takes a **fresh** photo: JPEG, **320 × 240**, quality 12;
2. sends it back in **224-byte chunks**, one chunk per request;
3. keeps the photo so a repeated request for the same chunk gets the same bytes;
4. frees it after the transfer finishes and the next photo is asked for, or after **2 s** with no requests.

It serves **one requester at a time** and answers **any** ESP32 that uses the right packet format. It has no fixed partner address.

USB is only for power, uploading and start-up messages. **Photos are never sent over USB.**

## 2. Interfaces

### 2.1 ESP-NOW photo protocol. Contract with `Main Module/main_module.ino`

| | |
|---|---|
| Radio | ESP-NOW, Wi-Fi station mode, **channel 6**, Wi-Fi sleep off, **no encryption or authentication** |
| Request (8 bytes, little-endian) | uint32 `token` (any non-zero number, the same for every chunk of one photo), uint32 `offset` (byte position: 0, 224, 448, …; below 128 KB) |
| Reply (236 bytes) | the request echoed (8 bytes), uint32 `total` = JPEG size in bytes (`0xFFFFFFFF` if the capture failed), 224 bytes of JPEG starting at `offset` (zero-padded at the end) |
| Start a photo | Request with offset 0 and a new token. Ignored while another requester's transfer is unfinished |
| Get the rest | Same token, offsets 224, 448, … until `total` is reached |
| Limits | Max photo size **128 KB** (131072 bytes); transfer released after **2000 ms** idle |

Requests with token 0, an offset that isn't a multiple of 224, or an offset of 128 KB or more are ignored.

The Main Module only accepts replies from this camera's address, `3C:8A:1F:D7:B1:24` (`cameraMac` in `main_module.ino`). **If this camera board is replaced, update that address.**

### 2.2 USB serial (115200 baud), messages only

| Message | Meaning |
|---------|---------|
| `ESP-NOW camera ready on channel 6` | Started fine |
| `ESP-NOW initialization failed` | Radio didn't start: no photos possible |
| `Camera initialization failed` | Camera didn't start. Requests are still answered, with `total = 0xFFFFFFFF` |

### 2.3 Phone App

**None yet.**

## 3. Features

| Feature | Status | Where in `camera_module.ino` |
|---------|--------|------------------------------|
| Camera start-up (OV3660, JPEG QVGA, quality 12) | Working (uploaded 22 Nov, per backup) | `setup()` |
| Fresh photo on each new request | Written | `serveRequest()` |
| Chunked transfer over ESP-NOW with repeatable chunks | Written | `serveRequest()` |
| 2 s idle release | Written | `loop()`, `releaseTransfer()` |
| Battery power hold | Written | `setup()` (GPIO33 HIGH) |
| Number-plate recognition / cloud upload | **Not started** (planned) | — |
| Link to the Phone App | **Not started** | — |

## 4. Hardware and wiring

There's no external wiring: the camera is built into the M5Stack board. The pin numbers below are the board's **internal** connections, listed so the code can be checked. **Don't rewire anything.**

| Signal | GPIO | Signal | GPIO |
|--------|------|--------|------|
| D0 | 32 | XCLK (20 MHz) | 27 |
| D1 | 35 | PCLK | 21 |
| D2 | 34 | VSYNC | 22 |
| D3 | 5 | HREF | 26 |
| D4 | 39 | SCCB SDA | 25 |
| D5 | 18 | SCCB SCL | 23 |
| D6 | 36 | RESET | 15 |
| D7 | 19 | PWDN | not used (−1) |
| **Battery power hold** | **33** (set HIGH at start so the camera stays on when running from its battery) | | |

- Frame buffer: 1, in internal RAM (DRAM), grabbed when empty.
- Power: USB or the board's own battery. For the battery test, unplug the USB data connection and leave the camera powered.
- **Where the camera is mounted on the bike, and which way it faces, isn't documented.** It should see vehicles passing on the right.
- USB port last seen as `/dev/cu.usbserial-2952066438` on the Mac (check, don't assume).

## 5. Build and upload

| | |
|---|---|
| Board setting | **ESP32 Dev Module** (`esp32:esp32:esp32`), upload speed **115200** |
| ESP32 core | **3.x** (compile-checked with **3.2.1**) |
| Libraries | None beyond the ESP32 core |

- **Arduino IDE:** open `camera_module.ino` (click OK when it offers to make a `camera_module` folder), set the board and port, then Upload. This replaces M5Stack's own camera firmware.
- **Mac script:** `bash Tools/flash_module.command camera PORT`
- **Check it works:** open a serial monitor at 115200 and reset the board. You should see `ESP-NOW camera ready on channel 6`. Then, with the Main Module running, click **Test Camera** in `Tools/LiDAR Viewer`: a 320 × 240 photo should appear.

## 6. Known gaps and next steps

1. **Not read back from the board:** the backup says this code was uploaded on 22 Nov 2026, but nobody has confirmed the camera is running it.
2. **Anyone nearby can request photos.** ESP-NOW is unencrypted and the camera answers any sender. That's fine for a prototype, not for real use.
3. **Number-plate recognition** (planned via a cloud service) isn't started. Photos currently end up on the Mac (`~/Downloads/LiDAR_<time>_Burst_<n>/`), through the Main Module and the LiDAR Viewer.
4. **No phone link.** The Main Module will be the phone's main link, so photos would most naturally reach the phone through it. That isn't designed yet.
5. Mounting position and direction aren't documented.
6. Older camera firmware is kept in `Archive/Firmware_Backup_v2_2026-11-25/LiDAR_v1.13_Complete_Setup/TimerCam/`. **Don't flash it by mistake:**
   - `Camera/Camera.ino` behaves the same as this sketch; only one line is written differently.
   - `Recovery/Camera/Camera.ino` is an older design. It only answers the Main Module's old address (`D4:8C:49:FA:75:48`), has a USB `P` photo command, and frees photos after 8 s.

## 7. Notes for AI assistants

- The `Request`/`Reply` structs (8 and 236 bytes), channel 6 and the 224-byte chunk size are a **contract with the Main Module**. The sketch enforces the sizes with a `static_assert`. Change both sketches and both specs together.
- The camera pin map is for the Timer Camera X with the OV3660 only. A different camera board needs a different map.
- Keep this sketch as one file.
