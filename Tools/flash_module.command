#!/bin/bash
# Build and upload one module's firmware with arduino-cli (macOS or Linux).
#
#   bash Tools/flash_module.command front  PORT
#   bash Tools/flash_module.command camera PORT
#   bash Tools/flash_module.command rear   PORT
#   bash Tools/flash_module.command rear   --compile-only     (build, no upload)
#
# PORT is the board's USB serial port, e.g. /dev/cu.usbserial-0001. Find it by
# unplugging and replugging the board and running: ls /dev/cu.*
# Rear Module only: LIDAR_RX_PIN=<gpio> bash Tools/flash_module.command rear PORT
# builds the optional direct LiDAR UART input on that GPIO.
#
# Replaces the backup's Flash_Relay.command, Flash_Camera.command and the
# flashing part of the OLED test.sh, which pointed at the old folder names.
set -euo pipefail

module=${1:-}
port=${2:-}
repo=$(cd -- "$(dirname -- "$0")/.." && pwd)
case "$module" in
  front)  folder="Front Module";  sketch=front_module;  libs=("Adafruit GFX Library" "Adafruit SSD1306" "Adafruit BusIO") ;;
  camera) folder="Camera Module"; sketch=camera_module; libs=() ;;
  rear)   folder="Rear Module";   sketch=rear_module;   libs=("Adafruit NeoPixel") ;;
  *) echo "Usage: bash Tools/flash_module.command front|camera|rear PORT|--compile-only"; exit 1 ;;
esac
[ -n "$port" ] || { echo "Give the USB port, or --compile-only."; exit 1; }

cli="$HOME/.esp-photo/arduino-cli"
[ -x "$cli" ] || cli=$(command -v arduino-cli || true)
[ -n "$cli" ] || { echo "arduino-cli not found. Install it (e.g. brew install arduino-cli) and retry."; exit 1; }

index=https://espressif.github.io/arduino-esp32/package_esp32_index.json
if ! "$cli" core list | grep -q '^esp32:esp32'; then
  "$cli" core update-index --additional-urls "$index"
  "$cli" core install esp32:esp32 --additional-urls "$index"
fi
if [ ${#libs[@]} -gt 0 ]; then "$cli" lib install "${libs[@]}"; fi

# arduino-cli needs the sketch in a folder with the same name as the .ino file.
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
mkdir "$build/$sketch"
cp "$repo/$folder/$sketch.ino" "$build/$sketch/"

flags=()
if [ "$module" = rear ] && [ -n "${LIDAR_RX_PIN:-}" ]; then
  [[ "$LIDAR_RX_PIN" =~ ^[0-9]+$ ]] || { echo "LIDAR_RX_PIN must be a GPIO number."; exit 1; }
  flags=(--build-property "compiler.cpp.extra_flags=-DLIDAR_RX_PIN=$LIDAR_RX_PIN")
fi

fqbn=esp32:esp32:esp32:UploadSpeed=115200
if [ "$port" = --compile-only ]; then
  "$cli" compile ${flags[@]+"${flags[@]}"} --fqbn "$fqbn" "$build/$sketch"
  echo "Compiled $folder. Nothing was uploaded."
  exit 0
fi

[ -c "$port" ] || { echo "No serial device at $port."; exit 1; }
if lsof "$port" >/dev/null 2>&1; then
  echo "Something is using $port (serial monitor or viewer). Close it and retry."
  exit 1
fi
echo "Uploading $folder to $port. This replaces the firmware on that board."
"$cli" compile --upload ${flags[@]+"${flags[@]}"} --fqbn "$fqbn" --port "$port" "$build/$sketch"
