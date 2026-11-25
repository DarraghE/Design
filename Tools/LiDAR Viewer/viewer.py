"""Replay LiDAR recordings and send hardware commands over one USB port."""
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import argparse
from contextlib import ExitStack
import json
from pathlib import Path
from queue import Empty, Full, Queue
import sys
from tempfile import NamedTemporaryFile
from threading import Event, Lock, Thread
from time import monotonic
import webbrowser

import serial

from lidar_core import load_recording
from scan_wire import apply_result, decode_live, encode_raw, encode_scan


class TestScreen:
    def __init__(self, board, lidar_source=None):
        self.board = board
        self.lidar_source = lidar_source
        self.commands = Queue(maxsize=32)
        self.ready = Event()
        self.next_scan_id = 0
        self.log_lines = deque(maxlen=100)
        self.lock = Lock()
        self.shutdown = Event()
        self.stop_replay = Event()
        self.playing = False
        self.replay = ('Waiting for live LiDAR' if lidar_source else 'Choose a .lidar recording')
        self.photo = b''
        self.photo_id = 0
        self.photo_duration = 0
        self.photo_expires = 0
        self.scan = None
        self.relay_status = 'Checking relay firmware…'

    def log(self, message):
        line = f'{datetime.now():%H:%M:%S}  {message}'
        with self.lock:
            self.log_lines.append(line)
        print(line, flush=True)

    def require_relay(self):
        if not self.ready.is_set():
            raise ValueError('Wait for the v1.13 relay to connect.')

    def request_camera_test(self):
        self.require_relay()
        self.commands.put_nowait((b'P', None))

    def request_lights(self):
        self.require_relay()
        self.commands.put_nowait((b'C', None))

    def play(self, scans):
        self.require_relay()
        if self.lidar_source is not None:
            raise ValueError("Restart the viewer without --lidar to replay a recording.")
        with self.lock:
            if self.playing:
                raise ValueError('Stop the current recording first.')
            self.playing = True
            self.replay = 'Starting replay'
            self.scan = None
            self.stop_replay.clear()
        Thread(target=self.replay_scans, args=(scans,), daemon=True).start()

    def queue_scan(self, command, metadata=None):
        while not self.shutdown.is_set() and not self.stop_replay.is_set() and self.ready.is_set():
            try:
                self.commands.put((command, metadata), timeout=0.05)
                return True
            except Full:
                continue
        return False

    def replay_scans(self, scans):
        started = monotonic()
        stopped = False
        try:
            first = scans[0]['t']
            if not self.queue_scan(b'R'):
                stopped = True
                return
            for index, scan in enumerate(scans, 1):
                elapsed = scan['t'] - first
                if self.stop_replay.wait(max(0, started + elapsed - monotonic())):
                    stopped = True
                    break
                self.next_scan_id += 1
                scan_id = self.next_scan_id
                frame, plot = encode_scan(scan_id, elapsed, scan['points'])
                metadata = dict(id=scan_id, points=plot, time=round(elapsed, 2),
                                progress=f'Scan {index}/{len(scans)}')
                if not self.queue_scan(frame, metadata):
                    stopped = True
                    break
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            self.log(f'Replay error: {error}')
            stopped = True
        finally:
            if stopped or not self.queue_scan(b''):
                with self.lock:
                    self.playing = False
                    self.replay = 'Playback stopped'

    def read_relay(self):
        """Send commands while receiving length-prefixed JPEGs on the same port."""
        if self.shutdown.wait(2):
            return
        buffer = bytearray()
        size = 0
        photo_header = None
        burst_folders = dict()
        light_deadline = None
        pending_scans = {}
        live_header = None
        received_lidar_at = None
        published_live_at = None
        try:
            self.board.reset_input_buffer()
            self.board.write(b'V')
            version_deadline = monotonic() + 4
            self.log(f'Relay connected at {self.board.port}, 460800 baud')
            while not self.shutdown.is_set():
                if version_deadline is not None and monotonic() >= version_deadline:
                    with self.lock:
                        self.relay_status = 'No v1.13 reply. Check the USB port and reflash the relay.'
                    self.log(self.relay_status)
                    version_deadline = None
                try:
                    command, metadata = self.commands.get_nowait()
                except Empty:
                    pass
                else:
                    if command == b'':
                        with self.lock:
                            self.playing = False
                            self.replay = ('Playback stopped' if self.stop_replay.is_set()
                                           else 'Playback complete')
                    elif metadata is not None:
                        if not self.stop_replay.is_set():
                            pending_scans[metadata['id']] = metadata
                            if len(pending_scans) > 64:
                                del pending_scans[next(iter(pending_scans))]
                            self.board.write(command)
                    else:
                        self.board.write(command)
                        if command == b'R':
                            pending_scans.clear()
                        if command == b'P':
                            self.log('Test Camera sent. If a burst is running, the test waits for it to finish.')
                        if command == b'C':
                            light_deadline = monotonic() + 4
                            self.log('Test lights sent to the ESP32.')
                if self.lidar_source is not None and self.ready.is_set():
                    count = min(1024, self.lidar_source.in_waiting)
                    if count:
                        data = self.lidar_source.read(count)
                        if data:
                            received_lidar_at = monotonic()
                            self.board.write(encode_raw(data))
                    with self.lock:
                        if received_lidar_at is None or monotonic() - received_lidar_at > 1:
                            self.replay = 'Waiting for LiDAR bytes from the USB sensor'
                        elif published_live_at is None or monotonic() - published_live_at > 2:
                            self.replay = 'LiDAR bytes received. Waiting for a complete ESP32 scan.'
                if light_deadline is not None and monotonic() >= light_deadline:
                    self.log('No LED pulse report from relay. Check that v1.13 firmware was flashed.')
                    light_deadline = None
                buffer.extend(self.board.read(min(8192, self.board.in_waiting or 1)))
                while buffer:
                    if size:
                        if len(buffer) < size:
                            break
                        jpeg = bytes(buffer[:size])
                        del buffer[:size]
                        size = 0
                        if live_header is not None:
                            self.publish_live(live_header, jpeg)
                            published_live_at = monotonic()
                            live_header = None
                        else:
                            burst, shot = photo_header
                            if burst and burst not in burst_folders:
                                stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                                burst_folders[burst] = Path.home() / 'Downloads' / f'LiDAR_{stamp}_Burst_{burst}'
                                if len(burst_folders) > 32:
                                    del burst_folders[next(iter(burst_folders))]
                            self.publish_photo(jpeg, burst_folders.get(burst), shot)
                            photo_header = None
                        continue
                    newline = buffer.find(b'\n')
                    if newline < 0:
                        if len(buffer) > 16384:
                            buffer.clear()
                        break
                    line = bytes(buffer[:newline]).strip()
                    del buffer[:newline + 1]
                    if line.startswith(b'LIVE '):
                        _, stamp, close, incoming, speed, count = line.split()
                        size = int(count) * 5
                        if not 0 <= size <= 5120:
                            raise ValueError('Invalid live scan size from relay')
                        live_header = (stamp, close, incoming, speed)
                        if not size:
                            self.publish_live(live_header, b'')
                            published_live_at = monotonic()
                            live_header = None
                    elif line.startswith(b'JPEG '):
                        _, length, burst, shot = line.split()
                        size = int(length)
                        photo_header = (int(burst), int(shot))
                        if not 0 < size <= 128 * 1024 or not 1 <= photo_header[1] <= 10:
                            raise ValueError('Invalid burst JPEG header from relay')
                    elif line.startswith((b'ERR ', b'INFO ')):
                        self.log(line.decode(errors='replace'))
                        if line.startswith(b'ERR LED'):
                            light_deadline = None
                    elif line.startswith(b'SCAN '):
                        result = apply_result(line, pending_scans)
                        if result is not None:
                            with self.lock:
                                self.scan = result
                                if self.playing:
                                    self.replay = result['progress']
                    elif line.startswith(b'EVENT '):
                        _, close, approach = line.split()
                        if close != b'0':
                            self.log(f'ESP32 detected {int(close)} close pass(es). The relay starts a burst when idle.')
                        if approach != b'0':
                            self.log(f'ESP32 detected {int(approach)} incoming-car warning(s).')
                    elif line.startswith((b'READY 1.13 RADIO ', b'READY 1.20 RADIO ')):
                        # 1.20: sent by v1.20, v1.21 and v1.25 (= v1.21 + phone link).
                        version_deadline = None
                        self.ready.set()
                        firmware = line.split()[1].decode()
                        with self.lock:
                            self.relay_status = (f'Main Module v{firmware} · Camera radio ready'
                                                 if line.endswith(b' OK') else
                                                 f'Main Module v{firmware} · Camera radio failed to start')
                        self.log(self.relay_status)
                    elif line == b'OK LED PULSE GPIO2':
                        light_deadline = None
                        self.log('ESP32 LED task completed a pulse on D9 / GPIO2.')
                    elif line == b'OK FLASH':
                        self.log('Relay returned the old acknowledgement. Flash v1.13 for LED task diagnostics.')
        except (OSError, ValueError, serial.SerialException) as error:
            with self.lock:
                self.relay_status = f'Relay USB error: {error}'
            self.log(self.relay_status)
        finally:
            self.ready.clear()

    def publish_live(self, header, payload):
        result = decode_live(header, payload)
        with self.lock:
            self.scan = result
            self.replay = 'Live LiDAR · processed on ESP32'

    def publish_photo(self, jpeg, folder, shot):
        if not (jpeg.startswith(b'\xff\xd8') and jpeg.endswith(b'\xff\xd9')):
            self.log('Invalid JPEG received')
            return
        with self.lock:
            now = monotonic()
            if folder is not None or self.photo_duration != 5 or now >= self.photo_expires:
                self.photo = jpeg
                self.photo_id += 1
                self.photo_duration = 5 if folder is not None else 1
                self.photo_expires = now + self.photo_duration
        if folder is None:
            self.log(f'Camera test photo received, {len(jpeg):,} bytes')
            return
        self.log(f'Photo {shot} received, {len(jpeg):,} bytes')  # bursts: 10 photos before v1.25, 5 since
        path = folder / f'photo_{shot:02d}.jpg'
        try:
            folder.mkdir(parents=True, exist_ok=True)
            path.write_bytes(jpeg)
            self.log(f'Saved {folder.name}/{path.name} in Downloads')
        except OSError as error:
            self.log(f'Could not save photo: {error}')


def handler_for(screen):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, body=b'', kind='text/plain; charset=utf-8', status=200):
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format, *args):
            pass

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/':
                self.reply(Path(__file__).with_name('index.html').read_bytes(), 'text/html; charset=utf-8')
            elif path == '/state':
                with screen.lock:
                    state = dict(replay=screen.replay, playing=screen.playing,
                                 photo=screen.photo_id if monotonic() < screen.photo_expires else 0,
                                 photo_duration=screen.photo_duration, scan=screen.scan, relay=screen.relay_status,
                                 live_usb=screen.lidar_source is not None, log=list(screen.log_lines))
                self.reply(json.dumps(state).encode(), 'application/json')
            elif path == '/latest':
                with screen.lock:
                    photo = screen.photo if self.path == f'/latest?{screen.photo_id}' else b''
                self.reply(photo, 'image/jpeg', 200 if photo else 404)
            else:
                self.send_error(404)

        def do_POST(self):
            if self.headers.get('X-Lidar-Test') != '1':
                self.send_error(403)
                return
            try:
                if self.path == '/play':
                    length = int(self.headers.get('Content-Length', 0))
                    if not 0 < length <= 64 * 1024 * 1024:
                        raise ValueError('Select a .lidar recording under 64 MB.')
                    with NamedTemporaryFile() as recording:
                        recording.write(self.rfile.read(length))
                        recording.flush()
                        _, scans = load_recording(recording.name)
                    screen.play(scans)
                elif self.path == '/stop':
                    screen.stop_replay.set()
                elif self.path == '/photo':
                    screen.request_camera_test()
                elif self.path == '/lights':
                    screen.request_lights()
                else:
                    self.send_error(404)
                    return
            except Full:
                self.reply(b'Relay queue is full. Try again shortly.', status=503)
                return
            except Exception as error:
                self.reply(str(error).encode(), status=400)
                return
            self.reply()

    return Handler


def main():
    parser = argparse.ArgumentParser(description='LiDAR replay and live ESP32 viewer')
    parser.add_argument('relay', nargs='?', default='/dev/cu.wchusbserial14110')
    parser.add_argument('--lidar', help='USB port of the STL-19P sensor')
    args = parser.parse_args()
    if args.relay == args.lidar:
        parser.error('Relay and LiDAR need separate USB ports.')
    with ExitStack() as connections:
        board = connections.enter_context(serial.Serial(
            args.relay, 460800, timeout=0.005, write_timeout=0.25))
        source = None
        if args.lidar:
            source = connections.enter_context(serial.Serial(args.lidar, 230400, timeout=0))
        screen = TestScreen(board, source)
        with ThreadingHTTPServer(('127.0.0.1', 0), handler_for(screen)) as server:
            worker = Thread(target=screen.read_relay, daemon=True)
            worker.start()
            url = f'http://127.0.0.1:{server.server_port}'
            print(f'Open {url}', flush=True)
            webbrowser.open(url)
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass
            finally:
                screen.stop_replay.set()
                screen.shutdown.set()
                worker.join(1)


if __name__ == '__main__':
    try:
        main()
    except serial.SerialException as error:
        sys.exit(f'Serial connection error: {error}')
