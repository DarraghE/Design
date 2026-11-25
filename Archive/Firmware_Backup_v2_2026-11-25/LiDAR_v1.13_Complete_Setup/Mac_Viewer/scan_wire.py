"""Encode a recording scan for the ESP32 and decode its display result."""
import math
import struct

MAX_POINTS = 1024


def encode_scan(scan_id, elapsed, points):
    records = []
    plot = []
    for point in points:
        if len(point) < 2:
            continue
        y, x = map(float, point[:2])
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        if abs(x) > 32 or abs(y) > 32:
            continue
        angle = float(point[3]) if len(point) >= 4 else -1
        encoded_angle = min(35999, int(angle * 100)) if 0 <= angle < 360 else 65535
        records.append(struct.pack('<hhH', round(x * 1000), round(y * 1000), encoded_angle))
        plot.append([x, y, 0])
    if len(records) > MAX_POINTS:
        raise ValueError(f'Scan {scan_id} exceeds the relay limit of {MAX_POINTS} points.')
    payload = struct.pack('<IIH', scan_id, round(elapsed * 1000), len(records)) + b''.join(records)
    return frame_payload(payload), plot


def frame_payload(payload):
    escaped = payload.replace(b'\x7d', b'\x7d\x5d').replace(b'\x7e', b'\x7d\x5e')
    return b'\x7e' + escaped + b'\x7e'


def encode_raw(data):
    return frame_payload(struct.pack('<IIH', 0, 0, 65535) + data)


def decode_live(header, payload):
    stamp, close, incoming, speed = header
    points = [[x / 1000, y / 1000, mark] for x, y, mark in struct.iter_unpack('<hhB', payload)]
    return dict(points=points, close=close == b'1', incoming=incoming == b'1',
                speed=float(speed), time=int(stamp) / 1000)


def apply_result(line, scans):
    scan_id, close, incoming, speed, *marks = line[5:].split()
    scan = scans.pop(int(scan_id), None)
    if scan is None:
        return None
    for mark in marks:
        index = int(mark[1:])
        if 0 <= index < len(scan['points']):
            scan['points'][index][2] = 2 if mark.startswith(b'-') else 1
    return dict(scan, close=close == b'1', incoming=incoming == b'1', speed=float(speed))
