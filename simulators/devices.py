"""Persistent loopback RDR/FCS simulators. No physical devices or optical output."""
import math
import socket
import struct
import threading
import time


class Devices:
    def __init__(self, rdr_port=0, fcs_port=0):
        self.rdr = self._socket(rdr_port)
        self.fcs = self._socket(fcs_port)
        self.rdr_port = self.rdr.getsockname()[1]
        self.fcs_port = self.fcs.getsockname()[1]
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.scenario = 'normal'
        self.target_count = 10
        self.error = None
        self.connected = self.reporting = False
        self.mode = self.token = self.rdr_tx = self.rx = 0
        self.rdr_peer = None
        self.last_receive = 0
        self.pending = None
        self.cache = {}
        self.excluded = set()
        self.pan = 0.0
        self.tilt = 12.3
        self.fcs_tx = 0
        self.elevation, self.azimuth = 12.3, 180.2
        self.target_elevation, self.target_azimuth = self.elevation, self.azimuth
        self.demo_until = self.laser_until = 0
        self.received = {'RDR': 0, 'FCS': 0}
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    @staticmethod
    def _socket(port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', port))
        sock.setblocking(False)
        return sock

    def configure(self, scenario='normal', target_count=10):
        if scenario not in ('normal', 'empty', 'rdr-offline', 'fcs-offline', 'fcs-fault'):
            raise ValueError('unknown scenario')
        if not 0 <= int(target_count) <= 10:
            raise ValueError('target_count must be 0..10')
        with self.lock:
            self.scenario, self.target_count = scenario, int(target_count)

    def _packet(self, kind, rx=0, a=0, b=0, result=0):
        self.rdr_tx = (self.rdr_tx + 1) & 65535
        return [kind, self.rdr_tx, rx, a, b, self.token >> 16, self.token & 65535,
                result, round(self.tilt * 10), round(self.pan * 10),
                int(self.connected) | (4 if self.reporting else 0) | (self.mode << 8), 0]

    def _send_rdr(self, fields):
        if self.rdr_peer and self.scenario != 'rdr-offline':
            self.rdr.sendto(struct.pack('!12H', *(v & 65535 for v in fields)), self.rdr_peer)

    def _rdr_request(self, raw, peer, now):
        if len(raw) != 24 or self.scenario == 'rdr-offline':
            return
        q = list(struct.unpack('!12H', raw))
        kind, seq = q[:2]
        if kind not in (0xED01, 0xED02, 0xED03, 0xED04, 0xED06):
            return
        self.received['RDR'] += 1
        token = (q[5] << 16) | q[6]
        if not token:
            return
        if kind == 0xED01 and not self.connected:
            self.connected, self.reporting, self.mode = True, False, 0
            self.token, self.cache, self.excluded = token, {}, set()
            self.rdr_peer = peer
        if token != self.token or peer != self.rdr_peer:
            return
        self.last_receive = now
        if seq in self.cache:
            original, response = self.cache[seq]
            if original == raw and response is not None:
                self._send_rdr(response)
            return
        self.rx = seq
        response = None
        if kind in (0xED01, 0xED02):
            response = self._packet(kind - 0xF00, seq, int(self.mode == 2))
        elif kind == 0xED03:
            self.reporting = bool(q[3])
        elif kind == 0xED04:
            self.excluded.discard(q[3]) if q[4] else self.excluded.add(q[3])
            response = self._packet(0xDE04, seq, q[3], q[4])
        elif kind == 0xED06:
            self.mode = 1 if q[3] else 3
            self.pending = (now + .35, raw, q)
            response = self._packet(0xDE06, seq, q[3], result=1)
        self.cache[seq] = raw, response
        if len(self.cache) > 128:
            del self.cache[next(iter(self.cache))]
        if response is not None:
            self._send_rdr(response)

    def _fcs_request(self, raw, peer, now):
        if len(raw) < 8 or self.scenario == 'fcs-offline':
            return
        kind, sequence, reply_to = struct.unpack_from('!3H', raw)
        if kind not in (0xEF01, 0xEF02, 0xEF03, 0xEF04, 0xEF05):
            return
        if len(raw) != (12 if kind == 0xEF03 else 8) or reply_to:
            return
        self.received['FCS'] += 1
        self.fcs_tx = (self.fcs_tx + 1) & 65535
        header = struct.pack('!3H', kind + 0xF00, self.fcs_tx, sequence)
        result = 0
        if kind == 0xEF03:
            elevation, azimuth, padding = struct.unpack_from('!hhH', raw, 6)
            if padding or not 0 <= elevation <= 600 or not 1200 <= azimuth <= 2400:
                result = 2
            elif self.scenario == 'fcs-fault':
                result = 3
            else:
                self.target_elevation, self.target_azimuth = elevation / 10, azimuth / 10
        elif kind == 0xEF04:
            enabled = struct.unpack_from('!H', raw, 6)[0]
            result = 0 if enabled in (0, 1) else 2
            self.laser_until = now + 1 if enabled == 1 and not result else 0
        elif kind == 0xEF05:
            if self.scenario == 'fcs-fault':
                result = 3
            else:
                self.demo_until, self.laser_until = now + 2, now + 1
        if kind == 0xEF02:
            moving = abs(self.elevation - self.target_elevation) > .05 or abs(self.azimuth - self.target_azimuth) > .05
            flags = 1 | (2 if moving else 0) | (4 if now < self.laser_until else 0)
            flags |= (8 if now < self.demo_until else 0) | (16 if self.scenario == 'fcs-fault' else 0)
            response = header + struct.pack('!Hhh', flags, round(self.elevation * 10), round(self.azimuth * 10))
        elif kind == 0xEF04:
            response = header + struct.pack('!3H', result, int(now < self.laser_until), 0)
        else:
            response = header + struct.pack('!H', result)
        self.fcs.sendto(response, peer)

    @staticmethod
    def _toward(value, target, step):
        return value + max(-step, min(step, target - value))

    def run(self):
        previous, report_at = time.monotonic(), 0
        try:
            while not self.done.wait(.01):
                now = time.monotonic()
                with self.lock:
                    dt, previous = min(now - previous, .1), now
                    self.elevation = self._toward(self.elevation, self.target_elevation, dt * 25)
                    self.azimuth = self._toward(self.azimuth, self.target_azimuth, dt * 25)
                    if self.mode == 2:
                        self.pan = 35 * math.sin(now * .18)
                    if self.connected and now - self.last_receive > 3.5:
                        self.connected, self.reporting, self.mode, self.pending = False, False, 0, None
                    if self.pending and now >= self.pending[0]:
                        _, raw, q = self.pending
                        self.pending = None
                        self.mode = 2 if q[3] else 0
                        response = self._packet(0xDE06, q[1], q[3])
                        self.cache[q[1]] = raw, response
                        self._send_rdr(response)
                    for sock, handler in ((self.rdr, self._rdr_request), (self.fcs, self._fcs_request)):
                        for _ in range(32):
                            try:
                                raw, peer = sock.recvfrom(512)
                            except BlockingIOError:
                                break
                            handler(raw, peer, now)
                    if self.connected and self.reporting and self.mode == 2 and now >= report_at:
                        report_at = now + .2
                        for index in range(self.target_count):
                            ident = 1001 + index
                            if ident in self.excluded:
                                continue
                            detected = self.scenario != 'empty'
                            x, y, z = -1400 + 300 * index, 350 + 65 * index, 2800 + 500 * index
                            self.rdr_tx = (self.rdr_tx + 1) & 65535
                            self._send_rdr([0xDE03, self.rdr_tx, self.rx, ident,
                                x if detected else 0, y if detected else 0, z if detected else 0,
                                round(self.tilt * 10) if detected else -32768,
                                round(self.pan * 10) if detected else -32768, int(detected), 0, 0])
        except BaseException as error:
            self.error = error
            self.done.set()

    def snapshot(self):
        with self.lock:
            return {'scenario': self.scenario, 'target_count': self.target_count,
                    'rdr_connected': self.connected, 'rdr_mode': self.mode, 'reporting': self.reporting,
                    'rdr_pan_deg': round(self.pan, 1), 'fcs_elevation_deg': round(self.elevation, 1),
                    'fcs_azimuth_deg': round(self.azimuth, 1), 'received': dict(self.received)}

    def close(self):
        self.done.set()
        self.thread.join(2)
        self.rdr.close()
        self.fcs.close()
