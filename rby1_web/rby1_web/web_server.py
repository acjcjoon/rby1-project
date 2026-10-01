"""Standard-library HTTP server and ROS-independent command mailbox."""
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import threading
import time


class CommandMailbox:
    """Serialize web inputs; never extend a velocity's life when publishing it."""

    def __init__(self, max_linear=0.2, max_angular=0.4, clock=time.monotonic,
                 operator_enabled=False):
        for value in (max_linear, max_angular):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('speed limits must be finite and positive')
        self.max_linear, self.max_angular = max_linear, max_angular
        self.clock = clock
        self.lock = threading.RLock()
        self.snapshot = {}
        self.updated_at = None
        self.events = deque(maxlen=20)
        self.actions = deque()
        self.sequences = {}
        self.owner = None
        self.gesture = None
        self.revoked = {}
        self.velocity = None
        self.velocity_at = 0.0
        self.stop_pending = False
        self.operator_enabled = operator_enabled
        self.operator = {}
        self.nav_owner = None
        self.nav_seen = 0.0
        self.nav_stop_pending = False

    def update_operator(self, **fields):
        with self.lock:
            self.operator.update(fields)

    def navigation_finished(self):
        with self.lock:
            self.nav_owner = None

    def update_state(self, snapshot):
        with self.lock:
            self.snapshot = snapshot
            self.updated_at = self.clock()

    def fresh(self):
        return self.updated_at is not None and self.clock() - self.updated_at < 1.0

    def release_motion(self):
        if self.owner is not None:
            self.revoked[self.owner] = self.gesture
        self.owner, self.gesture, self.velocity = None, None, None

    def submit(self, packet):
        if not isinstance(packet, dict):
            raise ValueError('command must be an object')
        if packet.get('schema_version', 1) != 1:
            raise ValueError('unsupported web command schema version')
        session, seq = packet.get('session'), packet.get('seq')
        if not isinstance(session, str) or not 1 <= len(session) <= 80:
            raise ValueError('invalid browser session')
        if type(seq) is not int or seq < 0:
            raise ValueError('invalid command sequence')
        operation = packet.get('operation')
        arguments = {}
        if operation == 'set_velocity':
            gesture = packet.get('gesture')
            if not isinstance(gesture, str) or not 1 <= len(gesture) <= 80:
                raise ValueError('invalid hold gesture')
            raw = packet.get('arguments', {})
            if not isinstance(raw, dict):
                raise ValueError('arguments must be an object')
            for key, limit in (('vx', self.max_linear), ('vy', self.max_linear),
                               ('wz', self.max_angular)):
                number = raw.get(key, 0.0)
                if type(number) not in (int, float) or not math.isfinite(number):
                    raise ValueError('velocity must be finite')
                if abs(number) > limit:
                    raise ValueError('velocity exceeds configured limit')
                arguments[key] = float(number)
            if math.hypot(arguments['vx'], arguments['vy']) > self.max_linear:
                raise ValueError('combined linear velocity exceeds configured limit')
        elif self.operator_enabled and operation in (
                'waypoint_save', 'waypoint_delete', 'waypoint_reload',
                'navigate', 'nav_cancel'):
            raw = packet.get('arguments', {})
            if not isinstance(raw, dict):
                raise ValueError('arguments must be an object')
            if operation in ('waypoint_save', 'waypoint_delete', 'navigate'):
                name = raw.get('name')
                if not isinstance(name, str) or not 1 <= len(name.strip()) <= 100:
                    raise ValueError('waypoint name must contain 1–100 characters')
                arguments['name'] = name.strip()
        elif operation not in ('stop', 'power_on', 'servo_on', 'enable',
                                'stream_on', 'stream_off', 'disable'):
            raise ValueError('unsupported mobile-base operation')
        with self.lock:
            if seq <= self.sequences.get(session, -1):
                raise ValueError('out-of-order command rejected')
            if session not in self.sequences and len(self.sequences) >= 128:
                raise ValueError('browser session limit reached; restart server')
            self.sequences[session] = seq
            now = self.clock()
            if self.owner and now - self.velocity_at >= 0.3:
                self.release_motion()
                self.stop_pending = True
            if operation in ('stop', 'nav_cancel'):
                self.release_motion()
                self.stop_pending = True
                self.actions.clear()
                self.nav_owner = None
                self.nav_stop_pending = self.operator_enabled
                return
            if not self.fresh():
                raise ValueError('control backend is offline')
            if self.owner and self.owner != session:
                raise ValueError('another browser is driving; STOP is available')
            if operation == 'set_velocity':
                if self.nav_owner is not None:
                    self.nav_stop_pending = True
                    self.nav_owner = None
                    self.actions.clear()
                if self.revoked.get(session) == gesture:
                    raise ValueError('motion stopped; release and press again')
                self.gesture = gesture
                self.owner, self.velocity, self.velocity_at = session, arguments, now
            else:
                if len(self.actions) >= 8:
                    raise ValueError('too many pending commands')
                if operation in ('stream_off', 'disable'):
                    self.release_motion()
                    self.stop_pending = True
                    self.actions.clear()
                    self.nav_owner = None
                    self.nav_stop_pending = self.operator_enabled
                routes = {
                    'power_on': ('request_power', {'enabled': True, 'target': 'all'}),
                    'servo_on': ('request_servo', {'enabled': True, 'target': 'all'}),
                    'enable': ('request_control_manager', {'command': 'enable'}),
                    'disable': ('request_control_manager', {'command': 'disable'}),
                    'stream_on': ('request_stream', {'enabled': True}),
                    'stream_off': ('request_stream', {'enabled': False}),
                }
                if operation == 'navigate':
                    if self.nav_owner is not None:
                        raise ValueError('navigation already active; cancel before another goal')
                    self.release_motion()
                    self.stop_pending = True
                    self.actions.clear()
                    self.nav_owner, self.nav_seen = session, now
                self.actions.append((now, routes.get(operation, (operation, arguments))))

    def drain(self):
        with self.lock:
            output = []
            now = self.clock()
            if self.nav_owner is not None and (now - self.nav_seen > 1.5 or not self.fresh()):
                self.nav_owner = None
                self.nav_stop_pending = True
                self.stop_pending = True
            if not self.fresh() or (self.velocity and now - self.velocity_at >= 0.3):
                if self.velocity:
                    self.stop_pending = True
                self.release_motion()
            if self.stop_pending:
                output.append(('stop', {}))
                self.stop_pending = False
            if self.nav_stop_pending:
                output.append(('nav_cancel', {}))
                self.nav_stop_pending = False
            while self.actions:
                submitted, command = self.actions.popleft()
                if self.fresh() and now - submitted < 1.0:
                    output.append(command)
            if self.velocity:
                output.append(('set_velocity', dict(self.velocity)))
            return output

    def status(self, session=None):
        with self.lock:
            if session is not None and session == self.nav_owner:
                self.nav_seen = self.clock()
            return {'schema_version': 1, 'mode': 'vslam' if self.operator_enabled else 'mobile',
                    'operator': {key: value for key, value in self.operator.items() if key != 'map'},
                    'connected': self.fresh(), 'snapshot': self.snapshot,
                    'events': list(self.events), 'limits': {
                        'linear': self.max_linear, 'angular': self.max_angular}}

    def add_event(self, packet):
        with self.lock:
            self.events.append(packet)


def create_server(address, mailbox):
    token = secrets.token_urlsafe(32)
    page = (Path(__file__).parent / 'static/index.html').read_text(encoding='utf-8')
    page = page.replace('__API_TOKEN__', json.dumps(token))

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(2.0)

        def log_message(self, *_args):
            pass

        def reply(self, status, value, content_type='application/json'):
            data = value.encode('utf-8') if isinstance(value, str) else json.dumps(
                value, allow_nan=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', content_type + '; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Frame-Options', 'DENY')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/':
                self.reply(200, page, 'text/html')
            elif self.path == '/api/state':
                self.reply(200, mailbox.status(self.headers.get('X-RBY1-Session')))
            elif self.path == '/api/map':
                with mailbox.lock:
                    value = mailbox.operator.get('map')
                self.reply(200, value)
            else:
                self.reply(404, {'error': 'not found'})

        def do_POST(self):
            if self.path != '/api/command':
                self.reply(404, {'error': 'not found'})
                return
            # Browsers on another origin cannot read the page token or send
            # this custom header. No permissive CORS headers are provided.
            if self.headers.get('X-RBY1-Token') != token:
                self.reply(403, {'error': 'invalid page token'})
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 2048:
                    raise ValueError('invalid body length')
                packet = json.loads(self.rfile.read(size))
                mailbox.submit(packet)
            except (ValueError, TypeError) as exc:
                self.reply(400, {'error': str(exc)})
                return
            self.reply(202, {'queued': True})

    server = ThreadingHTTPServer(address, Handler)
    server.daemon_threads = True
    return server
