"""Persistent loopback RDR/FCS simulators. No physical devices or optical output."""
import math
import socket
import struct
import threading
import time


class Devices:
    def __init__(self, rdr_port=0, fcs_port=0, enable_fcs=True):
        self.rdr = self._socket(rdr_port)
        self.fcs = self._socket(fcs_port) if enable_fcs else None
        self.rdr_port = self.rdr.getsockname()[1]
        self.fcs_port = self.fcs.getsockname()[1] if self.fcs else None
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.scenario = 'normal'
        self.target_count = 10
        self.targets = None
        self.rdr_world = (0., 0., 0.)
        self.target_reset_until = 0.
        self.retired_ids = set()
        self.error = None
        self.connected = self.reporting = False
        self.lidar_inhibited = False
        self.mode = self.token = self.rdr_tx = self.rx = 0
        self.rdr_peer = None
        self.last_receive = 0
        self.pending = None
        self.cache = {}
        self.excluded = set()
        self.pan = 180.0  # Device north reference; DE03 carries 1800 at north.
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

    def configure(self, scenario='normal', target_count=10, targets=None, rdr_world=(0,0,0)):
        if scenario not in ('normal', 'empty', 'rdr-offline', 'fcs-offline', 'fcs-fault'):
            raise ValueError('unknown scenario')
        if not 0 <= int(target_count) <= 10:
            raise ValueError('target_count must be 0..10')
        def position(value):
            if len(value)!=3 or any(isinstance(v,bool) for v in value):
                raise ValueError('XYZ must contain three finite numbers')
            xyz=tuple(float(v) for v in value)
            if not all(math.isfinite(v) for v in xyz) or math.hypot(*xyz)>3.4028234e38:
                raise ValueError('XYZ must fit finite float32 metres')
            return xyz
        origin=position(rdr_world)
        scene=None
        if targets is not None:
            scene=[]
            if not isinstance(targets,list) or len(targets)>10:
                raise ValueError('targets must be a list with at most 10 entries')
            ids=set()
            for target in targets:
                ident=target['id']
                if isinstance(ident,bool) or not isinstance(ident,int) or not 1<=ident<=65535 or ident in ids:
                    raise ValueError('target IDs must be unique integers in 1..65535')
                ids.add(ident)
                scene.append({'id':ident,'xyz':position(target['xyz'])})
        with self.lock:
            old_ids={t['id'] for t in self.targets} if self.targets is not None else set(range(1001,1001+self.target_count))
            new_ids={t['id'] for t in scene} if scene is not None else set(range(1001,1001+int(target_count)))
            if self.connected and new_ids-old_ids:
                raise ValueError('Restart the dummy to introduce new target IDs; existing IDs can be repositioned live')
            if scene!=self.targets or origin!=self.rdr_world:
                self.target_reset_until=time.monotonic()+.8
                if self.connected:
                    self.retired_ids.update(old_ids-new_ids)
            self.targets,self.rdr_world=scene,origin
            self.scenario, self.target_count = scenario, int(target_count)

    def _packet(self, kind, rx=0, a=0, b=0, result=0):
        self.rdr_tx = (self.rdr_tx + 1) & 65535
        return [kind, self.rdr_tx, rx, a, b, self.token >> 16, self.token & 65535,
                result, round(self.tilt * 10), round(self.pan * 10),
                int(self.connected) | (4 if self.reporting else 0) | (self.mode << 8) |
                24 | (64 if self.lidar_inhibited else 0) |
                (32 if self.mode == 2 and not self.lidar_inhibited else 0), 0]

    def _send_rdr(self, fields):
        if self.rdr_peer and self.scenario != 'rdr-offline':
            self.rdr.sendto(struct.pack('!12H', *(v & 65535 for v in fields)), self.rdr_peer)

    def _rdr_request(self, raw, peer, now):
        if len(raw) != 24 or self.scenario == 'rdr-offline':
            return
        q = list(struct.unpack('!12H', raw))
        kind, seq = q[:2]
        if kind not in (0xED01, 0xED02, 0xED03, 0xED04, 0xED06, 0xED07):
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
        elif kind == 0xED07:
            valid = q[3] in (0, 1) and q[4] == 0
            if valid:
                self.lidar_inhibited = q[3] == 0
            response = self._packet(0xDE07, seq, q[3], q[4], result=0 if valid else 2)
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
            if enabled == 0:
                self.demo_until = 0
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

    @staticmethod
    def _sensor_xyz(east, north, up, pan, tilt):
        """Express a fixed scene point in the moving sensor's right/up/front axes."""
        heading = math.radians(pan - 180)
        elevation = math.radians(tilt)
        right = east * math.cos(heading) - north * math.sin(heading)
        forward = east * math.sin(heading) + north * math.cos(heading)
        return (-right,  # Match the physical lidar's mirrored wire X.
                up * math.cos(elevation) - forward * math.sin(elevation),
                up * math.sin(elevation) + forward * math.cos(elevation))

    def run(self):
        previous, report_at = time.monotonic(), 0
        started = previous
        try:
            while not self.done.wait(.01):
                now = time.monotonic()
                with self.lock:
                    dt, previous = min(now - previous, .1), now
                    self.elevation = self._toward(self.elevation, self.target_elevation, dt * 25)
                    self.azimuth = self._toward(self.azimuth, self.target_azimuth, dt * 25)
                    if self.mode == 2:
                        elapsed = now - started
                        self.pan = 180 + 35 * math.sin(elapsed * .18)
                        self.tilt = 12.3 + 8 * math.sin(elapsed * .23)
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
                        if sock is None:
                            continue
                        for _ in range(32):
                            try:
                                raw, peer = sock.recvfrom(512)
                            except BlockingIOError:
                                break
                            handler(raw, peer, now)
                    if self.connected and self.reporting and self.mode == 2 and not self.lidar_inhibited and self.scenario != 'rdr-offline' and now >= report_at:
                        report_at = now + .2
                        scene=self.targets if self.targets is not None else [
                            {'id':1001+i,'xyz':(-1.4+.3*i,2.8+.5*i,.35+.065*i)} for i in range(self.target_count)]
                        retired=self.retired_ids
                        self.retired_ids=set()
                        for target in scene+[{'id':ident,'xyz':(0,0,0)} for ident in retired]:
                            ident = target['id']
                            if ident in self.excluded:
                                continue
                            detected = self.scenario != 'empty' and ident not in retired and now>=self.target_reset_until
                            pan, tilt = round(self.pan * 10), round(self.tilt * 10)
                            # Fixed world scene (east/north/up); only the sensor
                            # coordinates change while the observation pose moves.
                            xyz=target['xyz']
                            # Custom inputs are world metres. Default scene keeps
                            # the existing RDR-relative fixture convention.
                            relative=tuple(v-o for v,o in zip(xyz,self.rdr_world)) if self.targets is not None else xyz
                            x, y, z = self._sensor_xyz(*relative, pan / 10, tilt / 10)
                            self.rdr_tx = (self.rdr_tx + 1) & 65535
                            self.rdr.sendto(struct.pack('!4H3fhhHH4x', 0xDE03, self.rdr_tx, self.rx, ident,
                                x if detected else 0, y if detected else 0, z if detected else 0,
                                tilt if detected else -32768,
                                pan if detected else -32768,
                                int(detected), 1 if detected else 0), self.rdr_peer)
        except BaseException as error:
            self.error = error
            self.done.set()

    def snapshot(self):
        with self.lock:
            return {'scenario': self.scenario, 'target_count': len(self.targets) if self.targets is not None else self.target_count,
                    'targets': self.targets, 'rdr_world': self.rdr_world,
                    'rdr_connected': self.connected, 'rdr_mode': self.mode, 'reporting': self.reporting,
                    'rdr_pan_deg': round(self.pan, 1), 'mock_fcs_enabled': self.fcs is not None,
                    'fcs_elevation_deg': round(self.elevation, 1) if self.fcs else None,
                    'fcs_azimuth_deg': round(self.azimuth, 1) if self.fcs else None, 'received': dict(self.received)}

    def close(self):
        self.done.set()
        self.thread.join(2)
        self.rdr.close()
        if self.fcs:
            self.fcs.close()
