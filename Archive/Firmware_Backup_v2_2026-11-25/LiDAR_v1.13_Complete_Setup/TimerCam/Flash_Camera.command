#!/bin/bash
set -e
folder=$(cd -- "$(dirname -- "$0")" && pwd)
cli="$HOME/.esp-photo/arduino-cli"

case "${1:-}" in
  camera) sketch="$folder/Camera"; port=/dev/cu.usbserial-2952066438 ;;
  camera-rollback) sketch="$folder/Recovery/Camera"; port=/dev/cu.usbserial-2952066438 ;;
  *) echo "Usage: bash Flash_Camera.command camera|camera-rollback [USB-port]"; exit 1 ;;
esac

port=${2:-$port}
[ -x "$cli" ] || { echo "Arduino CLI is missing at $cli."; exit 1; }
[ -c "$port" ] || { echo "USB serial port missing: $port"; exit 1; }
if lsof "$port" 2>/dev/null; then
  echo "Close the process using $port and retry."
  exit 1
fi

echo "Building and uploading $1 firmware to $port at 115200 baud."
exec "$cli" compile --upload --fqbn esp32:esp32:esp32:UploadSpeed=115200 \
  --port "$port" "$sketch"
