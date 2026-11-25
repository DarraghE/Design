# Front Module: Specification

| | |
|---|---|
| **Module** | Front Module: the handlebar display |
| **Runs on** | ESP32 DevKit V1 with a 0.96" 128 × 64 SSD1306 OLED |
| **Code file** | `front_module.ino` (Arduino sketch, "BikeNav OLED v1.19": wireless JPEG receiver) |
| **Status** | **Installed on the front board.** On 25 Nov 2026 the board (MAC `8C:94:DF:6C:BC:80`) was flashed with this source and the flash hash was verified (team's `BikeNav_Installed_Firmware_2026-11-25` archive). It shows JPEG photos sent by the Main Module. Compiles (ESP32 core 3.3.8). **It doesn't show the phone's navigation screens yet** (section 8). |
| **Source** | `Front/Receiver/Receiver.ino` from that archive, unchanged apart from an added header comment. The archive notes the build was labelled v1.20 at upload; whether the v1.19 listing and the v1.20 build are identical is unconfirmed. The old USB-only test sketch is in `Archive/Firmware_Backup_v2_2026-11-25/OLED_GPIO15_22/` |

## 1. What this module does

### 1.1 Now: wireless JPEG receiver

1. On start: `Wire.begin(15, 22)`, starts the OLED, shows "Wireless receiver", joins ESP-NOW on channel 6 with the Main Module (`relayMac = D4:8C:49:FA:75:48`) as its only peer, and prints `READY display=<0/1> radio=<0/1> MAC=<address>` on USB (115200 baud).
2. It accepts only **236-byte** ESP-NOW packets from the Main Module's address. Each is one piece of a JPEG: `{uint32 token, uint32 offset, uint32 total, 224 bytes}`, the same layout as the camera's replies. The Main Module broadcasts every camera photo (and the `T` test image, and Oscar's USB image debugger lines) in this form.
3. Pieces are collected per token (duplicates ignored). A new token drops an unfinished JPEG (ack status 2). Nothing for **1.8 s** also drops it (status 2).
4. When every byte is in and it starts `FF D8` and ends `FF D9`, the JPEG is decoded at **1/4 scale** (TJpg_Decoder), turned black and white (brightness threshold) and drawn at **x + 24, y + 2** (so a 320 × 240 photo becomes 80 × 60 in the middle of the screen).
5. It sends an **ack** back to the Main Module: 16 bytes `{0x44495350 ("DISP"), token, size, status}`. The Main Module prints it as `DEBUG DISPLAY_ACK`.

| Ack status | Meaning |
|-----------|---------|
| 1 | Drawn |
| 2 | Dropped: a new image started, or 1.8 s with no pieces |
| 3 | Out of memory for the JPEG |
| 4 | Complete but not a valid JPEG, or the decoder failed |
| 5 | Screen not found |

It also still accepts the **USB test picture** (`OLED` + 64 + 1024 pixel bytes, section 2.1) from `Tools/OLED Image Test/`.

### 1.2 Next: navigation display (team decision, 25 Nov 2026)

The phone works out the turn-by-turn directions and **draws each screen as an image**. It sends the image to the **Main Module** (the hub at the back of the bike, and the phone's main link), which **immediately passes it on** to the Front Module. The Front Module just shows each image. It doesn't talk to the phone directly.

```
 Phone App ──Bluetooth LE──▶ Main Module (v1.26) ──ESP-NOW ch 6──▶ Front Module (OLED)
 draws 128x64 frames                 forwards frames at once            shows frames
```

**Decided and built on the phone side (25 Nov 2026):** the Phone App sends every screen as a compressed 128 × 64 frame in `0x10` display packets: row-by-row bytes (leftmost pixel in the top bit), each row XORed with the row above, then PackBits, in chunks of ≤ 180 bytes. The Main Module forwards these packets unchanged over ESP-NOW. The exact format is in `Phone App/SPECIFICATION.md` section 3.5, and the work list for this module is in section 9 there (summarised in section 8 below). The current receiver already listens to the Main Module over ESP-NOW and its USB test path draws the same row-by-row layout, so the change is mostly: accept the `0x10` packets as well, then decompress. Today they are ignored (wrong size), so nothing breaks in the meantime.

## 2. Interfaces

### 2.1 USB serial test picture (115200 baud). Contract with `Tools/OLED Image Test/`

| Bytes | Content |
|-------|---------|
| 0–3 | ASCII `OLED` |
| 4 | Height = `64` |
| 5–1028 | 1024 bytes: 128 × 64 pixels, **row by row from the top**, 8 pixels per byte, **leftmost pixel in the most significant bit**, 1 = lit |

Total **1029 bytes**. There's no checksum.

| Reply | Meaning |
|-------|---------|
| `READY display=<0/1> radio=<0/1> MAC=…` | Started (sent once at power-up) |
| `DISPLAY_ERROR` | Picture received but the screen wasn't found at 0x3C |
| `HEIGHT_ERROR` | Byte after `OLED` wasn't 64 |
| `TRANSFER_ERROR` | Fewer than 1024 pixel bytes arrived within 3 s |
| `DISPLAYED` | Picture drawn. This is a software confirmation; **look at the screen** to be sure |

### 2.2 ESP-NOW from the Main Module (channel 6)

| Direction | Packet | Notes |
|-----------|--------|-------|
| Main → Front | 236 bytes: token, offset (multiple of 224), total (≤ 128 KB), 224 bytes of JPEG | Only from `relayMac = D4:8C:49:FA:75:48`. Sent as a broadcast by the Main Module |
| Front → Main | 16-byte ack: `0x44495350`, token, size, status (table in 1.1) | Also printed on USB as `ACK <token> <size> <status> queue=<err>` |
| Main → Front | Phone screens: `0x10` packets ≤ 185 bytes (Main Module v1.25+) | **Ignored by this firmware** (wrong size). Section 8 |

The Front Module doesn't talk to the phone directly.

## 3. Features

| Feature | Status | Where in `front_module.ino` |
|---------|--------|-----------------------------|
| Start the OLED (SSD1306, I2C 0x3C, SDA 15, SCL 22) | Installed | `setup()` |
| Receive JPEGs from the Main Module over ESP-NOW, decode at 1/4 scale, draw, ack | Installed | `onPacket`, `process`, `drawBlock` |
| Receive and show a 128 × 64 test image over USB | Installed | `loop()` |
| Show the phone's compressed navigation frames (`0x10`) | **Not started** (format fixed, section 8) | — |
| Flash (invert 4×/s) when the frame's flag is set | **Not started** | — |
| "NO PHONE" after 10 s without frames | **Not started** | — |
| Brightness for daylight, battery reading | **Not started** | — |

## 4. Hardware and wiring

| OLED pin | ESP32 DevKit V1 |
|----------|-----------------|
| SDA | **GPIO15** (D15) |
| SCL (may be labelled SCK) | **GPIO22** (D22) |
| VDD / VCC | 3V3 |
| GND | GND |

- Display: SSD1306, 128 × 64, monochrome, I2C address **0x3C**, no reset pin. The controller and address assume a standard module and haven't been checked on the actual one.
- Buy the **Blue or White** OLED, not Yellow/Blue: on that one, the top 16 rows are always yellow.
- The installed wiring (team notes, 25 Nov 2026) is **SDA GPIO15, SCL GPIO22**. Older sketches used SDA GPIO21.
- GPIO15 is a boot-mode pin. If the board won't start or upload with the screen connected, that's a likely cause, and moving SDA to GPIO21 avoids it.
- Earlier tests reported I2C "NACK" errors (the screen didn't answer). A successful upload doesn't prove the screen works, and an old image left on screen doesn't prove the new firmware is running.
- USB port: `/dev/cu.usbserial-0001` on Oscar's Mac. The LiDAR has also used that name, so check.
- Station MAC `8C:94:DF:6C:BC:80` (the Main Module sends phone screens to this address).

## 5. Build and upload

| | |
|---|---|
| Board setting | **ESP32 Dev Module** (`esp32:esp32:esp32`), upload speed **115200** |
| ESP32 core | Compile-checked with **3.3.8** |
| Libraries | **Adafruit SSD1306** (2.5.17), **Adafruit GFX Library** (2.0.0), **Adafruit BusIO** (1.17.4), **TJpg_Decoder** (1.1.0, by Bodmer) |
| Size | 960,204 bytes of flash (73% of the default 1.3 MB) |

- **Arduino IDE:** open `front_module.ino` (click OK when it offers to make a `front_module` folder), set the board and port, then Upload.
- **Mac script:** `bash Tools/flash_module.command front PORT`
- **Check it works:**
  1. Serial monitor at 115200, reset the board: `READY display=1 radio=1 MAC=8C:94:DF:6C:BC:80`, and "Wireless receiver" on the screen.
  2. On the Main Module send `T` (test image) or `P` (camera photo): the picture appears and the Main Module prints `DEBUG DISPLAY_ACK … 1 …`.
  3. Or send a picture with `Tools/OLED Image Test/`: `DISPLAYED`.

## 6. Known gaps and next steps

1. **Navigation screens not shown yet:** build section 8 so the phone's `0x10` frames (forwarded by the Main Module) are drawn. Decide what happens to the camera photos the Main Module still broadcasts: they would replace the navigation screen during a close-pass burst (`Main Module/SPECIFICATION.md` section 6).
2. **v1.19 vs v1.20:** the archive's source is the v1.19 listing; the upload was labelled v1.20. If they differ, the v1.20 source should replace this file.
3. GPIO15 (SDA) is a boot-mode pin. If the board won't start or upload with the screen connected, move SDA to GPIO21 and change `Wire.begin(15,22)`.
4. Photos are tiny (80 × 60, thresholded): fine as a test, not useful to a rider.
5. Brightness/contrast for daylight, battery level and mounting aren't done.

## 7. Notes for AI assistants

- This file is the **installed** source. Keep the repo in step with what is on the board.
- The 236-byte packet and the 16-byte ack must match `Main Module/main_module.ino` (`Reply`, `DisplayAck`, `forwardDisplay`, `readImageByte`). The USB test picture must match `Tools/OLED Image Test/`.
- When adding the navigation frames, keep accepting the 236-byte JPEG packets unless the team decides otherwise, and update this spec, `Phone App/SPECIFICATION.md` section 9 and `Main Module/SPECIFICATION.md` in the same commit.
- The code body is the team's compact style. If you rewrite it for readability, keep the behaviour and the serial replies the same.
- Keep the sketch as one file.

## 8. To add: showing the phone's frames (brief for whoever writes it)

**Not built yet** (the installed v1.19 receiver ignores these packets). Full steps: `Phone App/SPECIFICATION.md` section 9. In short:

1. In `onPacket`, also accept packets from the Main Module that start with `0x10` (≤ 185 bytes), next to the existing 236-byte JPEG packets.
2. Packets starting with `0x10`: `[0x10][frame#][chunk index][chunk count][flags][data]`. Collect every chunk of one frame number; a new frame number starts over.
3. Join the chunks in order and **PackBits-decode** them (header h < 128: copy h+1 bytes; h > 128: repeat the next byte 257−h times). You must get exactly 1024 bytes.
4. **Undo the row XOR:** `for (i = 16; i < 1024; i++) buf[i] ^= buf[i - 16];`
5. `display.clearDisplay(); display.drawBitmap(0, 0, buf, 128, 64, SSD1306_WHITE); display.display();`
6. flags bit 0 set: toggle `display.invertDisplay()` every 125 ms until a frame without it arrives.
7. Nothing for 10 s: show "NO PHONE".
8. Check against the test vectors from `Phone App/navigation`: `dart run tool/export_screens.dart out` writes the exact packets for 415 screens and what each must look like.
