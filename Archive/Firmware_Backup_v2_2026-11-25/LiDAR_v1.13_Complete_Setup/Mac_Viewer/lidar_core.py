"""STL-19P decoding and .lidar recording support."""
from __future__ import annotations
import math
import json
import time
import types
import zipfile
from collections import deque

def _protocol_module():
    """STL-19P / D500 UART decoder: 230400 baud, 8N1, continuous receive.

    Standard-library port of the tested browser decoder. It never sends commands.
    Invalid or interrupted revolutions are discarded, never reported as clear scans.
    """
    import math
    import time

    PACKET_BYTES = 47
    POINTS_PER_PACKET = 12
    TIMESTAMP_PERIOD_MS = 30000


    def crc8(data, end=None):
        crc = 0
        for value in data[:len(data) if end is None else end]:
            crc ^= value
            for _ in range(8):
                crc = ((crc << 1) ^ (0x4D if crc & 128 else 0)) & 255
        return crc


    def decode_packet(data, verify_crc=True):
        if len(data) != PACKET_BYTES or data[0] != 0x54 or data[1] != 0x2C:
            return None
        if verify_crc and crc8(data, 46) != data[46]:
            return None
        def word(i):
            return data[i] | data[i + 1] << 8
        speed, start, end, timestamp = word(2), word(4), word(42), word(44)
        if start >= 36000 or end >= 36000 or timestamp > TIMESTAMP_PERIOD_MS:
            return None
        span = (end - start + 36000) % 36000 / 100
        if not 0 < span <= 60:
            return None
        points = []
        for i in range(POINTS_PER_PACKET):
            mm = word(6 + i * 3)
            points.append({
                "angle_deg": (start / 100 + span * i / 11) % 360,
                "distance_m": mm / 1000 if 30 <= mm <= 12000 else None,
                "intensity": data[8 + i * 3],
            })
        return {"speed_dps": speed, "start_angle": start / 100,
                "end_angle": end / 100, "span_deg": span,
                "timestamp_ms": timestamp % TIMESTAMP_PERIOD_MS,
                "points": points, "crc": data[46]}


    class Parser:
        def __init__(self, on_packet=None):
            self.on_packet = on_packet or (lambda packet: None)
            self._buffer = bytearray(PACKET_BYTES)
            self.reset()

        def reset(self):
            self._count = 0
            self.stats = {"bytes": 0, "packets": 0, "crc_errors": 0,
                          "invalid_packets": 0, "discarded_bytes": 0,
                          "resyncs": 0, "buffered_bytes": 0}

        def _resync(self):
            self.stats["resyncs"] += 1
            for i in range(1, self._count - 1):
                if self._buffer[i] == 0x54 and self._buffer[i + 1] == 0x2C:
                    remaining = self._count - i
                    self._buffer[:remaining] = self._buffer[i:self._count]
                    self._count = remaining
                    self.stats["discarded_bytes"] += i
                    return
            keep = 1 if self._buffer[self._count - 1] == 0x54 else 0
            self.stats["discarded_bytes"] += self._count - keep
            self._count = keep
            if keep:
                self._buffer[0] = 0x54

        def push(self, chunk):
            if not isinstance(chunk, (bytes, bytearray, memoryview)):
                raise TypeError("Serial input must be bytes.")
            self.stats["bytes"] += len(chunk)
            for value in chunk:
                if self._count == 0:
                    if value == 0x54:
                        self._buffer[0] = value
                        self._count = 1
                    else:
                        self.stats["discarded_bytes"] += 1
                    continue
                if self._count == 1 and value != 0x2C:
                    if value == 0x54:
                        self.stats["discarded_bytes"] += 1
                    else:
                        self.stats["discarded_bytes"] += 2
                        self._count = 0
                    continue
                self._buffer[self._count] = value
                self._count += 1
                if self._count != PACKET_BYTES:
                    continue
                if crc8(self._buffer, 46) != self._buffer[46]:
                    self.stats["crc_errors"] += 1
                    self._resync()
                    continue
                packet = decode_packet(self._buffer, False)
                if packet is None:
                    self.stats["invalid_packets"] += 1
                    self._resync()
                    continue
                self._count = 0
                self.stats["packets"] += 1
                self.on_packet(packet)
            self.stats["buffered_bytes"] = self._count


    def _finite(value):
        return isinstance(value, (int, float)) and math.isfinite(value)


    class ScanAssembler:
        def __init__(self, on_scan=None, offset_deg=0, reverse=False,
                     on_discontinuity=None, now=time.monotonic):
            self.on_scan = on_scan or (lambda scan: None)
            self.on_discontinuity = on_discontinuity or (lambda event: None)
            self.offset_deg = offset_deg if _finite(offset_deg) else 0
            self.reverse = bool(reverse)
            self._now = now
            self.reset()

        def reset(self):
            self.stats = {"packets": 0, "scans": 0, "dropped_partial_scans": 0,
                          "discontinuities": 0, "duplicate_packets": 0,
                          "backwards_packets": 0, "timestamp_wraps": 0,
                          "packet_gaps": 0, "invalid_packets": 0,
                          "last_reason": None}
            self._last_packet = None
            self._last_timestamp = None
            self._clock = 0
            self._last_beam_time = None
            self._last_host = None
            self._previous_raw = None
            self._collecting = False
            self._pending_gap = False
            self._gap_reason = None
            self._clear_scan()

        def _clear_scan(self):
            self._points = []
            self._beam_count = 0
            self._sector_beam_count = 0
            self._valid_beam_count = 0
            self._scan_first_t = None
            self._scan_last_t = None
            self._scan_speed_sum = 0

        def _discontinuity(self, reason):
            if self._collecting and self._beam_count:
                self.stats["dropped_partial_scans"] += 1
            self.stats["discontinuities"] += 1
            self.stats["last_reason"] = reason
            self._collecting = False
            self._previous_raw = None
            self._clear_scan()
            self._pending_gap = True
            self._gap_reason = reason
            self.on_discontinuity({"reason": reason, "stats": dict(self.stats)})

        def _forget_timing(self):
            self._last_packet = None
            self._last_timestamp = None

        def _finish_scan(self):
            if not self._collecting or not self._beam_count:
                return
            self.stats["scans"] += 1
            scan = {
                "t": self._scan_last_t, "start_t": self._scan_first_t,
                "end_t": self._scan_last_t, "points": self._points,
                "beam_count": self._beam_count,
                "sector_beam_count": self._sector_beam_count,
                "valid_beam_count": self._valid_beam_count,
                "hz": self._scan_speed_sum / self._beam_count / 360,
                "complete": True, "source": "serial",
                "gap_before": self._pending_gap, "gap_reason": self._gap_reason,
                "sector": {"start_deg": 55, "end_deg": 180},
                "scan_index": self.stats["scans"],
            }
            self._pending_gap = False
            self._gap_reason = None
            self.on_scan(scan)

        def push(self, packet):
            now = self._now()
            if (not isinstance(packet, dict)
                    or not isinstance(packet.get("points"), list)
                    or len(packet["points"]) != POINTS_PER_PACKET
                    or not _finite(packet.get("speed_dps"))
                    or not 360 <= packet["speed_dps"] <= 10800
                    or not _finite(packet.get("timestamp_ms"))
                    or not 0 <= packet["timestamp_ms"] < 30000
                    or not _finite(packet.get("span_deg"))
                    or not 0 < packet["span_deg"] <= 60):
                self.stats["invalid_packets"] += 1
                self._discontinuity("invalid_packet")
                self._forget_timing()
                return
            self.stats["packets"] += 1
            duration = packet["span_deg"] / packet["speed_dps"]
            beam_interval = duration / 11
            elapsed = None
            discontinuity = None
            if self._last_packet:
                previous = self._last_packet
                delta = (packet["timestamp_ms"] - self._last_timestamp + TIMESTAMP_PERIOD_MS) % TIMESTAMP_PERIOD_MS
                if (delta == 0 and packet["start_angle"] == previous["start_angle"]
                        and packet["end_angle"] == previous["end_angle"]):
                    self.stats["duplicate_packets"] += 1
                    return
                if delta > TIMESTAMP_PERIOD_MS / 2 or delta == 0:
                    self.stats["backwards_packets"] += 1
                    self._discontinuity("timestamp_backwards_or_reset")
                    self._forget_timing()
                    self._last_host = now
                    return
                elapsed = delta / 1000
                if packet["timestamp_ms"] < self._last_timestamp:
                    self.stats["timestamp_wraps"] += 1
                angular_gap = (packet["start_angle"] - previous["end_angle"]) % 360
                expected_step = max(packet["span_deg"], previous["span_deg"]) / 11
                if angular_gap <= 0 or angular_gap > expected_step * 1.85 + 0.04:
                    discontinuity = "missing_or_out_of_order_packet"
                if delta > max(20, duration * 4000):
                    discontinuity = "device_time_gap"
                if self._last_host is not None and now - self._last_host > 0.750:
                    discontinuity = "host_time_gap"
                if discontinuity:
                    self.stats["packet_gaps"] += 1
                    self._discontinuity(discontinuity)
            if elapsed is None:
                gap = 0 if self._last_host is None else max(0.001, now - self._last_host)
                self._clock = (0 if self._last_beam_time is None else self._last_beam_time + gap + beam_interval) + duration
            else:
                self._clock += elapsed
            first_t = self._clock - duration
            sample_interval = beam_interval
            # Keep packet ends on the unwrapped device clock. This prevents integer
            # speed rounding from accumulating into time drift at packet overlaps.
            if self._last_beam_time is not None and first_t <= self._last_beam_time:
                sample_interval = (self._clock - self._last_beam_time) / POINTS_PER_PACKET
                first_t = self._clock - sample_interval * 11
            for i, point in enumerate(packet["points"]):
                raw = point["angle_deg"]
                t = first_t + sample_interval * i
                if not _finite(raw) or not 0 <= raw < 360:
                    self.stats["invalid_packets"] += 1
                    self._discontinuity("invalid_point_angle")
                    self._forget_timing()
                    return
                if self._previous_raw is not None and raw < self._previous_raw - 180:
                    self._finish_scan()
                    self._clear_scan()
                    self._collecting = True
                self._previous_raw = raw
                self._last_beam_time = t
                if not self._collecting:
                    continue
                if self._scan_first_t is None:
                    self._scan_first_t = t
                self._scan_last_t = t
                self._beam_count += 1
                self._scan_speed_sum += packet["speed_dps"]
                angle = ((-raw if self.reverse else raw) + self.offset_deg) % 360
                if not (55 <= angle <= 215):
                    continue
                self._sector_beam_count += 1
                distance = point["distance_m"]
                if not _finite(distance) or not 0.03 <= distance <= 12:
                    continue
                self._valid_beam_count += 1
                radians = math.radians(angle)
                self._points.append([distance * math.cos(radians), distance * math.sin(radians), t, angle])
            self._last_packet = packet
            self._last_timestamp = packet["timestamp_ms"]
            self._last_host = now
    return types.SimpleNamespace(Parser=Parser, ScanAssembler=ScanAssembler)


def load_recording(path):
    with zipfile.ZipFile(path) as z:
        meta=json.loads(z.read('recording.json'))
        scans=[json.loads(line) for line in z.read('scans.jsonl').splitlines()]
        raw=z.read('serial.bin') if 'serial.bin' in z.namelist() else None
    if raw:
        try:
            recovered=[]
            proto=_protocol_module()
            assembler=proto.ScanAssembler(on_scan=recovered.append,offset_deg=float(meta.get('offset_deg',0)))
            parser=proto.Parser(on_packet=assembler.push)
            parser.push(raw)
            if recovered:
                scans=recovered
                meta['recovered_from_raw']=True
        except Exception as exc:
            meta['raw_recovery_error']=str(exc)
    if not scans: raise ValueError('No scans in recording')
    return meta, scans


def save_recording(path, scans, description, port):
    """Write the same .lidar ZIP layout used by the existing recorder."""
    import datetime
    meta = {"description": description, "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "source": "STL-19P/D500", "port": port,
            "sector": {"start_deg": 55, "end_deg": 180}, "scan_count": len(scans)}
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as z:
        z.writestr("recording.json", json.dumps(meta, indent=2))
        z.writestr("scans.jsonl", "\n".join(json.dumps(x, separators=(",", ":")) for x in scans) + "\n")

