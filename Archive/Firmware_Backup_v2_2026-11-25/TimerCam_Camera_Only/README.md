# Timer Camera X receiver

This is a camera-only replacement for ESP32_Photo_Relay. Keep the existing
second ESP32 firmware and Mac relay viewer. The camera answers any compatible
ESP32 on ESP-NOW channel 6. There is no fixed requester MAC and no direct USB
photo mode. USB is for power, firmware upload, and startup diagnostics.

The existing relay requests chunk zero using its capture token, then asks
for the remaining 224-byte chunks. The camera keeps one frame for retries
and serves a single active requester. If a transfer stalls for two seconds
the camera releases its buffer and allows another request. The radio
protocol is unchanged and unencrypted. A new ESP32 can use the existing
Relay sketch with this camera's MAC address in its Radio.h.

## Install

Unzip this folder into Downloads. Stop the camera's direct viewer or serial
monitor if it is running. Plug the camera into the Mac through the usual
adapter and run the command below. This replaces the M5Stack camera firmware
only. The camera upload speed is 115200.

```bash
"$HOME/.esp-photo/arduino-cli" compile --upload \
  --fqbn esp32:esp32:esp32:UploadSpeed=115200 \
  --port /dev/cu.usbserial-2952066438 \
  "$HOME/Downloads/TimerCam_Camera_Only/Camera"
```

## Test

Keep the second ESP32 connected to the Mac. Leave the camera powered and
disconnect its USB data connection to the Mac for the battery test.
Use the existing relay viewer and click Take Photo.

```bash
bash "$HOME/Downloads/ESP32_Photo_Relay/Start_Relay.command"
```

For rollback, upload the original Camera sketch from ESP32_Photo_Relay using
the same command with its Camera folder in place of this one.
