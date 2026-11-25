#!/bin/bash
set -e
folder=$(cd -- "$(dirname -- "$0")" && pwd)
cli="$HOME/.esp-photo/arduino-cli"
port=${1:-/dev/cu.wchusbserial14110}
[ -x "$cli" ] || { echo "Missing $cli"; exit 1; }
[ -c "$port" ] || { echo "Relay ESP32 missing at $port"; exit 1; }
if lsof "$port" 2>/dev/null; then
  echo "Stop the viewer or Serial Monitor using this port, then retry."
  exit 1
fi
"$cli" lib install "Adafruit NeoPixel"
build_flags=()
if [ -n "${LIDAR_RX_PIN:-}" ]; then
  [[ "$LIDAR_RX_PIN" =~ ^[0-9]+$ ]] || { echo "LIDAR_RX_PIN must be a GPIO number."; exit 1; }
  build_flags=(--build-property "compiler.cpp.extra_flags=-DLIDAR_RX_PIN=$LIDAR_RX_PIN")
fi
exec "$cli" compile --upload "${build_flags[@]}" --fqbn esp32:esp32:esp32:UploadSpeed=115200 --port "$port" "$folder/Relay"
