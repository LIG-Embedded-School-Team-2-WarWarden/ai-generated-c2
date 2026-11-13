"""Loopback-only FCS EF01..EF05 dummy. No GPIO, UART or physical output."""
import argparse
import json
from pathlib import Path
import socket
import struct
import time


class FcsDummy:
    def __init__(self):
        self.owner = None
        self.tx = 0
        self.highest = None
        self.cache = {}
        self.pan, self.tilt = 1800, 0
        self.target = (self.pan, self.tilt)
        self.move_until = self.demo_until = 0.0
        self.light = False

    def flags(self, now):
        if self.move_until and now >= self.move_until:
            self.pan, self.tilt = self.target
            self.move_until = 0
        if self.demo_until and now >= self.demo_until:
            self.demo_until = 0
            self.light = False
        return (1 if self.owner else 0) | (2 if self.move_until else 0) | (4 if self.light else 0) | (8 if self.demo_until else 0)

    def receive(self, raw, peer, now):
        flags = self.flags(now)
        if len(raw) < 8:
            return None
        kind, seq, rx = struct.unpack_from('!3H', raw)
        if kind not in range(0xEF01, 0xEF06):
            return None
        length = 12 if kind == 0xEF03 else 8
        pad = 10 if kind == 0xEF03 else 8 if kind == 0xEF04 else 6
        invalid = len(raw) != length or rx != 0 or any(raw[pad:])
        result = 2 if invalid else 0
        if kind == 0xEF01 and not invalid:
            if self.owner != peer and flags & 14:
                result = 1
            else:
                if self.owner != peer:
                    self.cache.clear(); self.highest = None
                self.owner = peer
        elif self.owner != peer:
            return None
        elif not invalid:
            if seq in self.cache:
                old, reply = self.cache[seq]
                if old == raw:
                    return reply
                result = 2
            elif self.highest is not None and not 0 < ((seq-self.highest) & 65535) < 32768:
                result = 2
            else:
                self.highest = seq
                if kind == 0xEF04:
                    on = struct.unpack_from('!H', raw, 6)[0]
                    if on > 1: result = 2
                    elif on == 0: self.light = False; self.demo_until = 0
                    elif self.demo_until: result = 1
                    else: self.light = True
                elif kind in (0xEF03, 0xEF05):
                    if flags & 10: result = 1
                    elif kind == 0xEF03:
                        tilt, pan = struct.unpack_from('!hh', raw, 6)
                        if not (-300 <= tilt <= 900 and 450 <= pan <= 3150): result = 2
                        else: self.target = (pan, tilt); self.move_until = now + .25
                    else: self.light = True; self.demo_until = now + 2
        if kind == 0xEF02 and result:
            return None
        self.tx = (self.tx + 1) & 65535
        header = struct.pack('!3H', kind + 0xF00, self.tx, seq)
        if kind == 0xEF02:
            reply = header + struct.pack('!Hhh', self.flags(now), self.tilt, self.pan)
        elif kind == 0xEF04:
            reply = header + struct.pack('!3H', result, int(self.light), 0)
        else:
            reply = header + struct.pack('!H', result)
        if kind != 0xEF01 and not invalid and seq not in self.cache:
            self.cache[seq] = (raw, reply)
            if len(self.cache) > 256: del self.cache[next(iter(self.cache))]
        if kind != 0xEF02:
            print(json.dumps({'message': hex(kind), 'sequence': seq, 'result': result, 'flags': self.flags(now)}), flush=True)
        return reply


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=22000)
    parser.add_argument('--stop-file', type=Path)
    parser.add_argument('--status-file', type=Path)
    args = parser.parse_args()
    dummy = FcsDummy()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', args.port)); sock.settimeout(.1)
        print(json.dumps({'ready': True, 'endpoint': sock.getsockname(), 'physical_output': False}), flush=True)
        next_status = 0
        while not args.stop_file or not args.stop_file.exists():
            now = time.monotonic()
            try:
                raw, peer = sock.recvfrom(512)
                reply = dummy.receive(raw, peer, time.monotonic())
                if reply: sock.sendto(reply, peer)
            except socket.timeout:
                pass
            if args.status_file and now >= next_status:
                args.status_file.write_text(json.dumps({'dummy': True, 'flags': dummy.flags(now),
                    'pan_deg': dummy.pan/10, 'tilt_deg': dummy.tilt/10, 'physical_output': False}), encoding='utf-8')
                next_status = now + .5


if __name__ == '__main__':
    main()
