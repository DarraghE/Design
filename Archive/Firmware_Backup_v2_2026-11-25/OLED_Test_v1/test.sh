#!/bin/bash
set -e
cd "$(dirname "$0")"
CLI="$HOME/.esp-photo/arduino-cli"
if [[ ! -x "$CLI" ]]; then CLI=$(command -v arduino-cli || true); fi
if [[ -z "$CLI" ]]; then brew install arduino-cli; CLI=$(command -v arduino-cli); fi
PORT=$(ls /dev/cu.wchusbserial* /dev/cu.SLAB_USBtoUART* /dev/cu.usbserial* /dev/cu.usbmodem* 2>/dev/null | head -1 || true)
if [[ -z "$PORT" ]]; then echo 'No USB serial port found'; exit 1; fi
printf 'Detected port: %s\n' "$PORT"
read -r -p 'Press Enter to use this port, or enter another port: ' P
PORT=${P:-$PORT}
if ! "$CLI" core list | grep -q 'esp32:esp32'; then "$CLI" core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json; "$CLI" core install esp32:esp32 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json; fi
"$CLI" lib install 'Adafruit GFX Library' 'Adafruit SSD1306'
"$CLI" compile --upload --fqbn esp32:esp32:esp32:UploadSpeed=115200 --port "$PORT" Receiver
python3 -m venv .venv
.venv/bin/pip install pillow pyserial
IMAGE=$(osascript -e 'POSIX path of (choose file with prompt "Select an image")')
.venv/bin/python sender.py "$PORT" "$IMAGE"
open preview.png
