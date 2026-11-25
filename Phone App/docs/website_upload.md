# Website upload contract (for whoever builds the website)

The phone app sends each close pass to **one URL** that you choose. Paste that URL into the app (**Settings → Website → Upload address**), optionally with a token, and tap **Send a test upload**. That's all the app needs.

## When the app uploads

- **Automatically**, as soon as a close pass has a confidently read number plate (plate found, no unreadable characters, OpenAI confidence ≥ 0.5).
- **By hand**, for any close pass: "Upload to website" on the incident screen.
- Failed or waiting uploads are retried when the app starts and when Settings are saved. **The same incident can arrive more than once**, so treat `incident.id` as unique and update instead of duplicating.

## The request

```
POST <your upload address>
Content-Type: multipart/form-data; boundary=...
Authorization: Bearer <token>        (only if a token is set in the app)
Accept: application/json
```

| Part (field name) | Type | Always sent? | Content |
|-------------------|------|--------------|---------|
| `incident` | `application/json` (file `incident.json`) | yes | The details below |
| `image` | `image/jpeg` (or `image/png` for demo incidents) | if there are photos | The photo where the plate is clearest |
| `photos` | `image/jpeg`, **repeated** once per photo | if there are photos | Every photo in time order (`photo_1.jpg` …) |
| `lidar` | `application/octet-stream` (file `lidar.bin`) | if available | The LiDAR clip, for a reconstruction on the website (format below) |

The test upload from Settings sends only `incident`, with `"simulated": true` and `"id": "test-…"`.

### `incident` JSON

```json
{
  "schema": "bikenav.closepass.v1",
  "id": "20261125-143012",
  "recordedAt": "2026-11-25T14:30:12.000Z",
  "location": { "lat": 53.3438, "lng": -6.2546, "accuracyM": 6.0 },
  "passDistanceM": 0.82,
  "passSpeedKmh": 47,
  "plate": "241-D-12345",
  "plateCountry": "IE",
  "plateConfidence": 0.93,
  "plateFormatValid": true,
  "vehicle": { "type": "car", "colour": "blue", "make": "Toyota" },
  "plateReaderModel": "gpt-4.1",
  "photoCount": 5,
  "bestPhoto": 2,
  "photoOffsetsMs": [300, 600, 900, 1200, 1500],
  "simulated": false,
  "app": "BikeNav Android 1.0.1"
}
```

| Field | Notes |
|-------|-------|
| `id` | Unique per close pass (`YYYYMMDD-HHMMSS`, with `-2`, `-3`… if two happen in one second). Use it to avoid duplicates |
| `recordedAt` | UTC, when the Main Module reported the pass |
| `location` | The phone's GPS at that moment, or `null` if location was off |
| `passDistanceM` | Closest distance between the bike's LiDAR and the vehicle, metres (`null` if unknown) |
| `passSpeedKmh` | The vehicle's estimated speed as it came up behind, km/h (`null` if unknown) |
| `plate` | Normalised: Irish `241-D-12345`, UK `AB12 CDE`, NI `ABZ 1234`. `null` if not read. May contain `?` when uploaded by hand |
| `plateConfidence` | 0-1, from OpenAI. Automatic uploads are always ≥ 0.5 |
| `plateFormatValid` | Whether the plate has a valid Irish, UK or NI shape |
| `bestPhoto` | 1-based index of the `image` among `photos` |
| `photoOffsetsMs` | Each photo's time after the pass was detected |
| `simulated` | `true` for the app's demo module and test uploads: **don't publish these** |

### `lidar` binary (little-endian)

```
u16 scanCount, u16 reserved
scanCount × {
  i32 offsetMs           time relative to the pass (negative = before)
  u8  flags              bit 0 close pass active, bit 1 vehicle approaching
  u16 speedKmh×10        0xFFFF = none
  u16 pointCount
  pointCount × { i16 x_mm, i16 y_mm, u8 class }   x = right, y = forward (LiDAR at 0,0);
                                                  class 0 scene, 1 close-pass, 2 approaching
}
```

## The response

- Any **2xx** status = success. The app marks the incident "Uploaded".
- Optionally reply with JSON `{"id": "...", "url": "https://…/passes/123"}`: the app shows the link and lets the user copy it.
- Anything else = failure: the app shows `Website replied <status>: <first 120 characters of the body>`, so a short plain-text error message is useful.
- Keep request size limits at **5 MB** or more (a typical upload is 80-250 KB).

## Minimal working receiver (Python, for testing)

Save as `receiver.py`, then run `pip install flask` and `python receiver.py`. In the app, use `http://<your computer's IP>:8000/api/close-passes` (phone and computer on the same Wi-Fi). Use HTTPS for the real website.

```python
import json, os
from flask import Flask, request, jsonify

TOKEN = os.environ.get("BIKENAV_TOKEN", "")     # optional; must match the app's token
SAVE = "uploads"
app = Flask(__name__)

@app.post("/api/close-passes")
def close_pass():
    if TOKEN and request.headers.get("Authorization") != f"Bearer {TOKEN}":
        return "Wrong token", 401
    incident = json.load(request.files["incident"])
    folder = os.path.join(SAVE, incident["id"])
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, "incident.json"), "w") as f:
        json.dump(incident, f, indent=2)
    if "image" in request.files:
        request.files["image"].save(os.path.join(folder, "best." + request.files["image"].filename.split(".")[-1]))
    for n, photo in enumerate(request.files.getlist("photos"), 1):
        photo.save(os.path.join(folder, photo.filename or f"photo_{n}.jpg"))
    if "lidar" in request.files:
        request.files["lidar"].save(os.path.join(folder, "lidar.bin"))
    print("Close pass", incident["id"], incident.get("plate"), incident.get("passDistanceM"), "m")
    return jsonify(id=incident["id"], url=f"{request.host_url}passes/{incident['id']}"), 201

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000)
```

Try it without the phone (this sends what the app's test upload sends):

```bash
curl -F 'incident={"schema":"bikenav.closepass.v1","id":"test-1","recordedAt":"2026-11-25T14:30:12Z","plate":"262-D-12345","passDistanceM":0.85,"passSpeedKmh":46,"simulated":true};type=application/json;filename=incident.json' http://localhost:8000/api/close-passes
```

## Privacy

Uploads contain number plates, photos of vehicles and people, and the rider's location. The website should use HTTPS, require the token, keep `simulated` uploads separate, and follow GDPR for storing and publishing this data.
