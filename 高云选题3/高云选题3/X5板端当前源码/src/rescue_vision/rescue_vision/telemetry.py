"""Local HTTP telemetry. Network threads enqueue commands, never operate ROS/I/O."""
from collections import OrderedDict, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from queue import Queue, Full, Empty
from urllib.parse import urlsplit
import copy
import hmac
import json
import math
import os
import re
import secrets
import threading
import time


TOPICS = {'scene': '/scene/report', 'mission': '/mission/state',
          'traffic': '/traffic/state', 'fpga': '/fpga/link',
          'voice': '/voice/status', 'navigation': '/navigation/state',
          'interaction': '/interaction/result'}
ACTIONS = {'navigate', 'inspect', 'report', 'stop', 'resume'}


def runtime_token(path):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        value = path.read_text(encoding='ascii').strip()
        if not re.fullmatch(r'[A-Za-z0-9_-]{32,128}', value):
            raise ValueError('Invalid dashboard token file; generate a new token')
        if os.name != 'nt':
            path.chmod(0o600)
        return value
    value = secrets.token_urlsafe(32)
    fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w', encoding='ascii') as f:
        f.write(value + '\n')
    return value


def validate_command(data, now, building_ids=()):
    if not isinstance(data, dict):
        raise ValueError('command must be an object')
    if set(data) - {'command_id', 'source', 'action', 'building_id', 'text', 'issued_at', 'ttl_s'}:
        raise ValueError('unknown command field')
    if data.get('source') != 'web' or data.get('action') not in ACTIONS:
        raise ValueError('source or action is not allowed')
    cid = data.get('command_id')
    if not isinstance(cid, str) or not re.fullmatch(r'[A-Za-z0-9_-]{8,80}', cid):
        raise ValueError('invalid command_id')
    for name in ('issued_at', 'ttl_s'):
        v = data.get(name)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError('invalid timestamp or TTL')
    if not 0 < data['ttl_s'] <= 10 or not -.25 <= now-data['issued_at'] <= data['ttl_s']:
        raise ValueError('expired or future command')
    text = data.get('text', '')
    if not isinstance(text, str) or len(text) > 80 or any(ord(c) < 32 for c in text):
        raise ValueError('invalid text')
    bid = data.get('building_id')
    if bid is not None and (not isinstance(bid, str) or bid not in building_ids):
        raise ValueError('unknown building_id')
    if data['action'] == 'navigate' and bid is None and not text.strip():
        raise ValueError('navigate requires text or building_id')
    return copy.deepcopy(data)


class TelemetryStore:
    def __init__(self, building_ids=(), demo=False):
        self.lock = threading.Lock()
        self.states = {}
        self.events = deque(maxlen=30)
        self.commands = Queue(maxsize=32)
        self.seen = OrderedDict()
        self.jpeg = None
        self.frame_time = None
        self.building_ids = set(building_ids)
        self.demo = bool(demo)

    def update(self, topic, value):
        if topic not in TOPICS or not isinstance(value, dict):
            raise ValueError('unknown topic or invalid state')
        encoded = json.dumps(value, ensure_ascii=False, allow_nan=False)
        if len(encoded.encode('utf-8')) > 65536:
            raise ValueError('state too large')
        with self.lock:
            previous = self.states.get(topic, {}).get('data')
            self.states[topic] = {'data': copy.deepcopy(value), 'received': time.monotonic()}
            if previous != value and topic in ('voice', 'interaction'):
                self.events.append({'kind': topic, 'stamp': time.time(), 'data': copy.deepcopy(value)})
            elif previous != value and (value.get('error') or value.get('warning')):
                self.events.append({'kind': 'alert', 'stamp': time.time(), 'data': copy.deepcopy(value)})

    def set_frame(self, jpeg):
        if not isinstance(jpeg, bytes) or len(jpeg) > 400000 or not jpeg.startswith(b'\xff\xd8'):
            raise ValueError('invalid or oversized JPEG')
        with self.lock:
            self.jpeg, self.frame_time = jpeg, time.monotonic()

    def frame(self):
        with self.lock:
            if self.jpeg is None or time.monotonic()-self.frame_time > 3:
                return None
            return self.jpeg

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            states = {k: {'data': copy.deepcopy(v['data']), 'age_s': round(now-v['received'], 3),
                          'stale': now-v['received'] > 3} for k, v in self.states.items()}
            return {'mode': 'recorded_demo' if self.demo else 'live_ros', 'server_time': time.time(),
                    'states': states, 'events': list(self.events),
                    'preview_age_s': None if self.frame_time is None else round(now-self.frame_time, 3),
                    'commands_pending': self.commands.qsize()}

    def enqueue(self, data):
        # Browser clocks may differ. An authenticated HTTP submission begins its
        # lifetime on server receipt; a supplied future time cannot extend it.
        # The browser timestamp is checked for syntax only and never forwarded.
        if not isinstance(data, dict):
            raise ValueError('command must be an object')
        client_stamp = data.get('issued_at')
        if isinstance(client_stamp, bool) or not isinstance(client_stamp, (int, float)) or not math.isfinite(client_stamp):
            raise ValueError('invalid client timestamp')
        data = copy.deepcopy(data)
        data['issued_at'] = time.time()
        data = validate_command(data, data['issued_at'], self.building_ids)
        with self.lock:
            now = time.monotonic()
            while self.seen and now-next(iter(self.seen.values())) > 300:
                self.seen.popitem(last=False)
            cid = data['command_id']
            if cid in self.seen:
                return {'accepted': False, 'duplicate': True, 'command_id': cid}
            try:
                self.commands.put_nowait(data)
            except Full:
                raise ValueError('command queue full')
            self.seen[cid] = now
            if len(self.seen) > 4096:
                self.seen.popitem(last=False)
        return {'accepted': True, 'command_id': cid, 'status': 'queued_not_executed'}

    def drain(self):
        while True:
            try:
                data = self.commands.get_nowait()
            except Empty:
                return
            try:
                yield validate_command(data, time.time(), self.building_ids)
            except ValueError:
                continue


class DashboardServer:
    def __init__(self, store, web_dir, token, bind='127.0.0.1', port=8088,
                 allowed_hosts=('127.0.0.1', 'localhost')):
        self.store, self.web_dir, self.token = store, Path(web_dir), token
        self.allowed_hosts = {str(h).lower() for h in allowed_hosts}
        if not self.allowed_hosts:
            raise ValueError('allowed_hosts is required')
        owner = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.0'

            def setup(self):
                super().setup()
                self.connection.settimeout(3)

            def log_message(self, *args):
                pass  # Tokens and request bodies never enter access logs.

            def reply(self, code, data, kind='application/json; charset=utf-8'):
                if not isinstance(data, bytes):
                    data = json.dumps(data, ensure_ascii=False, allow_nan=False).encode('utf-8')
                self.send_response(code)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('X-Frame-Options', 'DENY')
                self.send_header('Referrer-Policy', 'no-referrer')
                self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' blob:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'")
                self.end_headers()
                self.wfile.write(data)

            def trusted(self):
                try:
                    host = self.headers.get('Host', '')
                    parsed = urlsplit('//'+host)
                    valid = (parsed.hostname in owner.allowed_hosts and not parsed.username
                             and not parsed.password and not parsed.path and not parsed.query
                             and not parsed.fragment and (parsed.port or 80) == owner.http.server_port)
                    origin = self.headers.get('Origin')
                    if origin is not None and origin != 'http://'+host:
                        valid = False
                except ValueError:
                    valid = False
                if not valid:
                    self.reply(403, {'error': 'Host or Origin rejected'})
                return valid

            def authorized(self):
                if not self.trusted():
                    return False
                supplied = self.headers.get('X-Rescue-Token', '')
                if not hmac.compare_digest(supplied.encode('utf-8'), owner.token.encode('utf-8')):
                    self.reply(401, {'error': 'Token required'})
                    return False
                return True

            def do_GET(self):
                if urlsplit(self.path).query:
                    self.reply(400, {'error': 'Query parameters are not accepted'})
                    return
                if self.path.startswith('/api/'):
                    if not self.authorized():
                        return
                    if self.path == '/api/state':
                        self.reply(200, owner.store.snapshot())
                    elif self.path == '/api/frame':
                        image = owner.store.frame()
                        self.reply(200, image, 'image/jpeg') if image else self.reply(404, {'error': 'No fresh frame'})
                    else:
                        self.reply(404, {'error': 'Not found'})
                    return
                if not self.trusted():
                    return
                assets = {'/': ('index.html', 'text/html; charset=utf-8'),
                          '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                          '/style.css': ('style.css', 'text/css; charset=utf-8')}
                if self.path not in assets:
                    self.reply(404, {'error': 'Not found'})
                    return
                name, kind = assets[self.path]
                self.reply(200, (owner.web_dir/name).read_bytes(), kind)

            def do_POST(self):
                if not self.authorized():
                    return
                if self.path != '/api/command':
                    self.reply(404, {'error': 'Not found'}); return
                if self.headers.get('Transfer-Encoding') or self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    self.reply(415, {'error': 'JSON with Content-Length required'}); return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 2048:
                        raise ValueError('invalid request length')
                    body = self.rfile.read(length)
                    if len(body) != length:
                        raise ValueError('incomplete request')
                    data = json.loads(body.decode('utf-8'))
                    result = owner.store.enqueue(data)
                    self.reply(202 if result['accepted'] else 200, result)
                except (ValueError, UnicodeError) as exc:
                    self.reply(400, {'error': str(exc)})

        self.http = ThreadingHTTPServer((bind, int(port)), Handler)
        self.http.daemon_threads = True
        self.thread = None

    @property
    def port(self):
        return self.http.server_port

    def start(self):
        self.thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        if self.thread is not None:
            self.http.shutdown()
            self.thread.join(timeout=3)
        self.http.server_close()
