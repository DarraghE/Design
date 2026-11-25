# Front Module: Specification

| | |
|---|---|
| **Module** | Front Module: the handlebar display |
| **Runs on** | ESP32 DevKit V1 with a 0.96" 128 × 64 SSD1306 OLED |
| **Code file** | `front_module.ino` (Arduino sketch, **v1.21**: phone screens + wireless JPEG receiver) |
| **Status** | **v1.21 written, not yet flashed.** It adds the phone's navigation screens to the installed v1.19 receiver. Compiles (ESP32 core 3.3.8); its screen decoder is checked on a computer against all 415 screens the phone app draws, pixel by pixel (`Tools/Phone Link Test/`). **The board still runs v1.19** (flashed 25 Nov 2026, MAC `8C:94:DF:6C:BC:80`, hash verified), which ignores the phone's screens: that is why phone → Main → Front showed nothing. |
| **Source** | The installed v1.19 receiver (`Front/Receiver/Receiver.ino` in the team's `BikeNav_Installed_Firmware_2026-11-25` archive). Every v1.21 change is marked `v1.21:`; the screen decoder is new code between `BEGIN/END SCREEN DECODER`. The old USB-only test sketch is in `Archive/Firmware_Backup_v2_2026-11-25/OLED_GPIO15_22/` |

## 1. What this module does

### 1.1 Wireless JPEG receiver (since v1.19)

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
| 6 | **v1.21:** not drawn, because the phone's screens are showing (a screen arrived in the last 10 s) |

It also still accepts the **USB test picture** (`OLED` + 64 + 1024 pixel bytes, section 2.1) from `Tools/OLED Image Test/`.

### 1.2 Phone navigation screens (new in v1.21)

The phone draws every handlebar screen itself and sends it, compressed, to the Main Module, which forwards each `0x10` packet unchanged to this module (unicast to its MAC):

```
 Phone App ──Bluetooth LE──▶ Main Module ──ESP-NOW ch 6──▶ Front Module (OLED)
 draws 128x64 frames          forwards packets at once      decodes and draws
```

1. `onPacket` accepts packets from the Main Module that start with `0x10` (5–250 bytes, never exactly 236) and queues them (32 deep).
2. `screen::Assembler` collects the parts of one screen in any order: `[0x10][frame#][index][count][flags][data…]`. A different frame number or part count starts over; duplicates are ignored; up to 96 parts.
3. When all parts are in: join them, **PackBits-decode** to exactly **1024 bytes** (anything else is dropped), **undo the row XOR** and draw the whole screen with `drawBitmap`.
4. flags bit 0 = **flash**: the screen inverts every **125 ms** until a screen without the flag arrives.
5. **Phone screens have priority:** while screens are arriving (one in the last **10 s**), camera JPEGs are acked with status 6 and not drawn, so a close-pass photo burst can't cover the directions. The phone shows its own "CLOSE PASS" screen instead.
6. **NO PHONE:** if screens stop for 10 s after the phone was seen, the screen shows "NO PHONE / Open the Bike Nav app" and USB prints `INFO NO PHONE`. The phone re-sends its screen every 3 s, so this only happens when the app stops or the link drops. JPEGs are drawn again after that.

Format: `Phone App/SPECIFICATION.md` section 3.5.

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
| `READY display=<0/1> radio=<0/1> MAC=…` then `INFO PHONE SCREENS 1.21` | Started (sent once at power-up) |
| `INFO NO PHONE` | Phone screens stopped for 10 s |
| `DISPLAY_ERROR` | Picture received but the screen wasn't found at 0x3C |
| `HEIGHT_ERROR` | Byte after `OLED` wasn't 64 |
| `TRANSFER_ERROR` | Fewer than 1024 pixel bytes arrived within 3 s |
| `DISPLAYED` | Picture drawn. This is a software confirmation; **look at the screen** to be sure |

### 2.2 ESP-NOW from the Main Module (channel 6)

| Direction | Packet | Notes |
|-----------|--------|-------|
| Main → Front | 236 bytes: token, offset (multiple of 224), total (≤ 128 KB), 224 bytes of JPEG | Only from `relayMac = D4:8C:49:FA:75:48`. Sent as a broadcast by the Main Module |
| Front → Main | 16-byte ack: `0x44495350`, token, size, status (table in 1.1) | Also printed on USB as `ACK <token> <size> <status> queue=<err>` |
| Main → Front | Phone screens: `0x10` packets ≤ 185 bytes (Main Module v1.25+), unicast | **v1.21:** decoded and drawn (section 1.2). v1.19 ignored them |

The Front Module doesn't talk to the phone directly.

## 3. Features

| Feature | Status | Where in `front_module.ino` |
|---------|--------|-----------------------------|
| Start the OLED (SSD1306, I2C 0x3C, SDA 15, SCL 22) | Installed | `setup()` |
| Receive JPEGs from the Main Module over ESP-NOW, decode at 1/4 scale, draw, ack | Installed | `onPacket`, `process`, `drawBlock` |
| Receive and show a 128 × 64 test image over USB | Installed | `loop()` |
| Show the phone's compressed navigation frames (`0x10`) | Written (v1.21), decoder host-tested on 415 screens, not yet flashed | `screen::Assembler`, `showPhoneFrame` |
| Flash (invert every 125 ms) when the frame's flag is set | Written (v1.21) | `showPhoneFrame`, `servicePhoneScreen` |
| "NO PHONE" after 10 s without frames | Written (v1.21) | `servicePhoneScreen` |
| Phone screens take priority over camera JPEGs | Written (v1.21) | `process` (status 6) |
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
| Size (v1.21) | 961,476 bytes of flash (73% of the default 1.3 MB), 79,016 bytes of static RAM (23 KB of it holds screen parts) |

- **Arduino IDE:** open `front_module.ino` (click OK when it offers to make a `front_module` folder), set the board and port, then Upload.
- **Mac script:** `bash Tools/flash_module.command front PORT`
- **Check it works:**
  1. Serial monitor at 115200, reset the board: `READY display=1 radio=1 MAC=8C:94:DF:6C:BC:80`, `INFO PHONE SCREENS 1.21`, and "Wireless receiver" on the screen.
  2. Connect the phone app to the Main Module: within 3 s the screen shows the app's screen ("READY / CLOSE PASS ON", or the navigation screen). Close the app: "NO PHONE" after 10 s.
  3. With the phone disconnected (after "NO PHONE"), on the Main Module send `T` (test image) or `P` (camera photo): the picture appears and the Main Module prints `DEBUG DISPLAY_ACK … 1 …`. With the phone connected the ack status is 6 (not drawn).
  4. Or send a picture with `Tools/OLED Image Test/`: `DISPLAYED`.

## 6. Known gaps and next steps

1. **Flash v1.21.** The board still runs v1.19, which ignores the phone's screens. Not yet tested on the board: drawing speed, flashing, and "NO PHONE".
2. **v1.19 vs v1.20:** the installed build was labelled v1.20 but only the v1.19 source exists; v1.21 is built on the v1.19 source.
3. **Photos during navigation:** decided in v1.21: not drawn while phone screens arrive (status 6). If the team wants photos on the screen, change `process()`.
4. GPIO15 (SDA) is a boot-mode pin. If the board won't start or upload with the screen connected, move SDA to GPIO21 and change `Wire.begin(15,22)`.
5. Brightness/contrast for daylight, battery level and mounting aren't done.

## 7. Notes for AI assistants

- v1.21 is not yet on the board (v1.19 is). Keep the repo in step with what is on the board, and say in the Status line which version is installed.
- **The `screen` namespace between `// BEGIN SCREEN DECODER` and `// END SCREEN DECODER` must stay plain C++** (no Arduino calls): `Tools/Phone Link Test/run_test.py` compiles it on a computer and checks it against the phone app's 415 screens. Run it after any change.
- The 236-byte packet and the 16-byte ack must match `Main Module/main_module.ino` (`Reply`, `DisplayAck`, `forwardDisplay`, `readImageByte`). The USB test picture must match `Tools/OLED Image Test/`.
- The `0x10` screen format must match `Phone App/navigation/lib/src/display_codec.dart`. Change both in the same commit, with this spec and `Phone App/SPECIFICATION.md` sections 3.5 and 9.
- The code body is the team's compact style. If you rewrite it for readability, keep the behaviour and the serial replies the same.
- Keep the sketch as one file.

## 8. Phone screens: how to check

Built in v1.21 (section 1.2). To check the decoder without hardware: `python "Tools/Phone Link Test/run_test.py"` (needs a C++ compiler and the Dart SDK). It decodes all 415 test screens from `Phone App/navigation/tool/export_screens.dart` four ways (as sent, re-cut into 15-byte parts in reverse order, every part twice, after a broken half-frame) and compares every pixel with the app's own drawing.
