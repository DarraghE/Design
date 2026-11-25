# LiDAR v1.13 · Live ESP32 decoding

## Package contents

| Folder | Contents |
| --- | --- |
| `ESP32_Relay/` | FireBeetle relay firmware, detection logic, and upload script |
| `Mac_Viewer/` | Live browser dashboard, serial bridge, and recording playback |
| `TimerCam/` | Previously packaged camera firmware, recovery sketch, and restore script |

Run the commands below from the extracted package folder. Use the relay flashing script for v1.13, and keep the TimerCam files for recovery.

The Mac forwards raw STL-19P/D500 bytes at 230400 baud over the relay USB link at 460800 baud. The ESP32 checks packet CRC, assembles full sweeps, runs both detectors, drives LEDs and requests TimerCam photos. The browser receives the ESP32's point cloud and detection results. **Test Camera** and **Test Lights** are available.

```bash
bash ESP32_Relay/Flash_Relay.command /dev/cu.wchusbserial14110
"$HOME/.venv/bin/python" Mac_Viewer/viewer.py /dev/cu.wchusbserial14110 --lidar /dev/cu.usbserial-0001
```

Use the current USB port names from `ls /dev/cu.*`. Stop previous viewers before flashing. Run the viewer without `--lidar` to replay a saved recording. The camera uses its existing firmware.

## Direct UART later

`ESP32_Relay/Relay/Stl19p.h` handles both the Mac bridge and the optional ESP32 UART2 input. To enable UART2, define `LIDAR_RX_PIN` in the relay build for a free RX-capable GPIO verified against your board pinout. Connect sensor TX to that RX with compatible voltage levels and shared ground. UART settings are 230400, 8N1. Use one raw LiDAR input at a time. Leave the UART option disabled for the Mac bridge.

The decoder uses the same 55°–215° scan crop and coordinate convention as the Mac's previous STL-19P decoder. It discards incomplete sweeps after packet corruption or discontinuities.


## Close-pass photos

The ESP32 requests ten photos per burst. The first request is due 0.5 seconds after detection, with subsequent requests spaced 0.5 seconds apart. Each successful JPEG is sent immediately, displayed in the viewer, and saved in a numbered burst folder in Downloads. Slow camera or USB transfers can delay later shots. The log identifies failed shots. A close pass during an active burst does not interrupt or start a second burst.

**Test Camera** requests one debug photo and shows it for one second. A running burst finishes first. Test photos are preview-only. Close-pass photos stay visible for five seconds after the newest photo appears and are still saved to Downloads. An active close-pass preview takes priority.

## Build error on macOS

If the build reports `Failed to extract libcrypto.1.1.dylib`, the firmware upload has not started. Check the available space for your home and temporary directories:

```bash
df -h "$HOME" "${TMPDIR:-/tmp}"
```

If either location is nearly full, free a few GB before retrying the relay upload. If plenty of space is available and the same error recurs, keep the full build log so the failing ESP32 tool can be identified.
