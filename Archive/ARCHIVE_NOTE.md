# Archive

Read-only history. Nothing here is maintained or used by the current code. Don't edit, move or delete it.

| Folder / files | What it is |
|----------------|------------|
| `README.md`, `pubspec.lock`, `docs/`, `phone/` | This repository's contents up to 22 Nov 2026, moved here unchanged when the repo was split into module folders. The newer navigation code is in [`../Phone App`](../Phone%20App/SPECIFICATION.md) |
| [`Firmware_Backup_v2_2026-11-25/`](Firmware_Backup_v2_2026-11-25/README_MASTER.md) | The team's firmware handover backup (v2, 25 Nov 2026), byte for byte as received. All 29 files match its `FILE_INDEX.json` SHA-256 hashes. Start with its `README_MASTER.md` and `hardware_manifest.json` |

## Where the firmware backup's contents went

The current copies live in the module folders and `Tools/`. The backup keeps the originals and every alternative version.

| In the backup | Now at | Notes |
|---------------|--------|-------|
| `OLED_GPIO15_22/Receiver/Receiver.ino` | `Front Module/front_module.ino` | Same code, header comment added |
| `OLED_GPIO15_22/OLED_Dashboard_v2.html`, `sender.py` | `Tools/OLED Image Test/` | Unchanged |
| `OLED_GPIO15_22/test.sh` | (not copied) | Replaced by `Tools/flash_module.command front`; the image-sending step is in `Tools/README.md` |
| `OLED_Test_v1/` | Archive only | Older OLED version with SDA on GPIO21 |
| `TimerCam_Camera_Only/Camera/Camera.ino` | `Camera Module/camera_module.ino` | Same code, header comment added |
| `LiDAR_v1.13_Complete_Setup/ESP32_Relay/Relay/` (5 files) | `Main Module/main_module.ino` | Merged into one file, no logic changes (same compiled size) |
| `LiDAR_v1.13_Complete_Setup/Mac_Viewer/` | `Tools/LiDAR Viewer/` | Unchanged |
| `…/ESP32_Relay/Flash_Relay.command`, `…/TimerCam/Flash_Camera.command` | (not copied) | Replaced by `Tools/flash_module.command` |
| `LiDAR_v1.13_Complete_Setup/TimerCam/Camera/`, `…/TimerCam/Recovery/` | Archive only | Older camera versions. **Don't flash the Recovery one by mistake** |
| `README_MASTER.md`, `hardware_manifest.json`, other READMEs | Archive only | Their facts are written into each module's `SPECIFICATION.md` |
