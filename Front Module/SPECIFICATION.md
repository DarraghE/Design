# Front Module: Specification

| | |
|---|---|
| **Module** | Front Module: the handlebar display |
| **Runs on** | ESP32 DevKit V1 with a 0.96" 128 × 64 SSD1306 OLED |
| **Code file** | `front_module.ino` (Arduino sketch) |
| **Status** | **Test firmware only.** It shows a picture sent from a computer over USB, and compiles (ESP32 core 3.2.1). This GPIO15 version has **not** been confirmed flashed or working. There is no Bluetooth and no link to other modules yet. |
| **Source** | `OLED_GPIO15_22/Receiver/Receiver.ino` from the firmware backup of 25 Nov 2026, unchanged apart from an added header comment |

## 1. What this module does

### 1.1 Now: image receiver (test)

The board waits on USB serial for a picture, draws it on the OLED and confirms. It's a bench test of the screen and the picture format:

1. On start: shows "Receiver ready" and sends `READY`. If the screen isn't found, it sends `DISPLAY_ERROR` and stops.
2. It scans incoming bytes for `OLED`, reads the height byte (must be 64) and then exactly 1024 pixel bytes (3 s timeout).
3. It clears the screen, draws the picture and replies `DISPLAYED`.

The picture isn't saved: after a reset the screen shows "Receiver ready" again. Send pictures with `Tools/OLED Image Test/`.

### 1.2 Planned: navigation display (team decision, 25 Nov 2026)

The phone works out the turn-by-turn directions and **draws each screen as an image**. It sends the image to the **Main Module** (the hub at the back of the bike, and the phone's main link), which **immediately passes it on** to the Front Module. The Front Module just shows each image. It doesn't talk to the phone directly.

```
 Phone App ──Bluetooth (planned)──▶ Main Module ──(link to choose)──▶ Front Module (OLED)
 draws 128x64 frames                 forwards frames at once            shows frames
```

Today's image receiver is the starting point for this: it already shows whole 128 × 64 frames. **Still to decide:**

| Question | Notes |
|----------|-------|
| Phone → Main Module link | The team leans towards Bluetooth. Not built on either side |
| Main Module → Front Module link | ESP-NOW fits: the Main Module already uses it for the camera, and a 1024-byte frame is 5 ESP-NOW packets of up to 250 bytes |
| Bit order of the 1024 bytes | The receiver expects **row by row, leftmost pixel in the top bit** (Adafruit `drawBitmap`). The phone's `OledFrame.toSsd1306()` produces the SSD1306's **page layout** (8 rows per byte, top pixel in bit 0). Pick one format and convert on one side |
| Compression | Raw frames are only 1024 bytes. Compression is optional |
| Flashing at 5 m | The phone's current design flashes the screen by inverting it 4 times a second. With images, the phone can send inverted frames, or the protocol can carry a flash flag |

This replaces the earlier draft in `Phone App/SPECIFICATION.md` section 5, where the phone sends symbol numbers and the Front Module draws the arrows itself. The phone code still implements that earlier draft. A Bluetooth firmware for it exists in `spidermannn3000/Design-group` (`front-module/`), but it isn't used here.

## 2. Interfaces

### 2.1 USB serial image packet (115200 baud). Contract with `Tools/OLED Image Test/`

| Bytes | Content |
|-------|---------|
| 0–3 | ASCII `OLED` |
| 4 | Height = `64` |
| 5–1028 | 1024 bytes: 128 × 64 pixels, **row by row from the top**, 8 pixels per byte, **leftmost pixel in the most significant bit**, 1 = lit |

Total **1029 bytes**. There's no checksum.

| Reply | Meaning |
|-------|---------|
| `READY` | Started, screen found |
| `DISPLAY_ERROR` | Screen not found at 0x3C (check wiring/address). Board stops |
| `HEIGHT_ERROR` | Byte after `OLED` wasn't 64 |
| `TRANSFER_ERROR` | Fewer than 1024 pixel bytes arrived within 3 s |
| `DISPLAYED` | Picture drawn. This is a software confirmation; **look at the screen** to be sure |

### 2.2 Other modules and the phone

**None yet.** See section 1.2.

## 3. Features

| Feature | Status | Where in `front_module.ino` |
|---------|--------|-----------------------------|
| Start the OLED (SSD1306, I2C 0x3C, SDA 15, SCL 22) | Written, not confirmed on hardware | `setup()` |
| Receive and show a 128 × 64 image over USB | Written, not confirmed on hardware | `loop()` |
| Receive images from the Main Module | **Not started** (planned) | — |
| Link to the Main Module | **Not started** (ESP-NOW suggested) | — |
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
- **The wiring has changed over time.** First SCL went to D1, then to D22. The older sketch, and Design-group's Bluetooth firmware, use **SDA GPIO21**. This sketch follows the latest stated wiring, **SDA GPIO15**. If the screen stays blank, check which pin SDA is really on and change `Wire.begin(15,22)` to match.
- GPIO15 is a boot-mode pin. If the board won't start or upload with the screen connected, that's a likely cause, and moving SDA to GPIO21 avoids it.
- Earlier tests reported I2C "NACK" errors (the screen didn't answer). A successful upload doesn't prove the screen works, and an old image left on screen doesn't prove the new firmware is running.
- USB port last seen as `/dev/cu.usbserial-0001` on the Mac. The LiDAR has also used that name, so check.

## 5. Build and upload

| | |
|---|---|
| Board setting | **ESP32 Dev Module** (`esp32:esp32:esp32`), upload speed **115200** |
| ESP32 core | Compile-checked with **3.2.1** |
| Libraries | **Adafruit GFX Library** (1.13.0 used before; 2.0.0 also compiles), **Adafruit SSD1306** (2.5.17), **Adafruit BusIO** (1.17.4) |

- **Arduino IDE:** open `front_module.ino` (click OK when it offers to make a `front_module` folder), set the board and port, then Upload.
- **Mac script:** `bash Tools/flash_module.command front PORT`
- **Check it works:**
  1. Open a serial monitor at 115200 and reset the board. You should see `READY`, and "Receiver ready" on the screen.
  2. Send a picture with `Tools/OLED Image Test/`. You should get `DISPLAYED` and see the picture on the screen.
  3. If you get `DISPLAY_ERROR`, check power, ground, SDA/SCL pins and the address.

## 6. Known gaps and next steps

1. **Not confirmed on hardware.** This GPIO15 version was never confirmed flashed. The older GPIO21 version uploaded on 25 Nov 2026, but the screen wasn't confirmed.
2. **Confirm the real SDA pin** (15 or 21) and update `Wire.begin` and section 4.
3. **Build the planned navigation path** (section 1.2): choose the Main Module → Front Module link and the frame format.
4. Brightness/contrast for daylight, battery level and mounting aren't done.
5. Old versions are in `Archive/Firmware_Backup_v2_2026-11-25/` (`OLED_Test_v1/` is the GPIO21 version). The Bluetooth symbol-based firmware is in `spidermannn3000/Design-group`.

## 7. Notes for AI assistants

- The 1029-byte packet (section 2.1) must match `Tools/OLED Image Test/OLED_Dashboard_v2.html` and `sender.py`.
- When building the navigation link, write the chosen frame format and link into section 2 here, into `Phone App/SPECIFICATION.md` section 5, and into `Main Module/SPECIFICATION.md`, all in the same commit.
- The code body is the backup's original compact style. If you rewrite it for readability, keep the behaviour and the serial replies the same.
- Keep the sketch as one file.
