# Front Module: Specification

| | |
|---|---|
| **Module** | Front Module (handlebar display) |
| **Runs on** | ESP32 |
| **Code file** | `front_module.ino` (Arduino sketch) |
| **Status** | **Not started.** The code file is an empty sketch. |

## 1. What this module does

Nothing yet. No firmware has been written in this repository.

## 2. What the Phone App already expects from it

The Phone App has already defined how it will talk to this module. The full contract is in [`../Phone App/SPECIFICATION.md`](../Phone%20App/SPECIFICATION.md), section 5. In short:

- Hardware: 0.96" 128x64 SSD1306 OLED (Blue or White version) over I2C, address `0x3C`.
- Bluetooth LE peripheral, service `6e3b0001-8f7a-4c1e-9b52-2d1c0b7a1f10`. The phone writes to characteristic `6e3b0002-8f7a-4c1e-9b52-2d1c0b7a1f10`.
- Each update carries: version, flags (flash, arrived), distance in metres, and up to 4 display symbol IDs.
- The screen layout is defined by `renderOled` in `Phone App/navigation/lib/src/oled.dart`.

## 3. Features

| ID | Feature | Status |
|----|---------|--------|
| | | |

## 4. Hardware and wiring

| Part | Pin | ESP32 pin |
|------|-----|-----------|
| | | |

## 5. Build and upload

1. Open `front_module.ino` in the Arduino IDE. If it asks to put the file in a folder called `front_module`, click OK.
2. Pick the ESP32 board under **Tools → Board** and its port under **Tools → Port**.
3. Click **Upload**.

## 6. Known gaps and next steps

- Everything. Fill this in as the firmware is written.

## 7. Notes for AI assistants

- Keep all firmware in the single file `front_module.ino` so it can be copied and uploaded directly.
- List every library the sketch needs at the top of the file.
- If you change the symbol IDs or the Bluetooth packet, update the Phone App to match.
