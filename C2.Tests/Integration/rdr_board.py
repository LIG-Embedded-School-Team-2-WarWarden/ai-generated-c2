"""Native console + independent loopback ED/DE board; never contacts hardware."""

import argparse
import json
from pathlib import Path
import queue
import socket
import struct
import subprocess
import tempfile
import threading
import time

ROOT = Path(__file__).resolve().parents[1]


class Board:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(0.02)
        self.port = self.socket.getsockname()[1]
        self.peer = None
        self.done = self.drop = self.connected = self.reporting = False
        self.mode = self.token = self.tx = self.rx = self.starts = self.stops = 0
        self.last = 0
        self.pending = None
        self.cache = {}
        self.excluded = set()
        self.requests = []
        self.wire = []
        self.echo_arguments = True
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def packet(self, kind, rx=0, a=0, b=0, result=0):
        self.tx = (self.tx + 1) & 65535
        return [kind, self.tx, rx, a, b, self.token >> 16, self.token & 65535,
                result, 123, -153, int(self.connected) | (4 if self.reporting else 0) | (self.mode << 8), 0]

    def send(self, fields):
        if self.peer and not self.drop:
            self.socket.sendto(struct.pack("!12H", *(v & 65535 for v in fields)), self.peer)

    def run(self):
        emit = 0
        try:
            while not self.done:
                now = time.monotonic()
                if self.connected and now - self.last > 3.5:
                    self.connected = self.reporting = False
                    self.mode = 0
                    self.pending = None
                if self.pending and now >= self.pending[0]:
                    _, request = self.pending
                    self.pending = None
                    self.mode = 2 if request[3] else 0
                    response = self.packet(0xDE06, request[1], request[3])
                    if not self.echo_arguments:
                        response[3] = response[4] = 0
                    self.cache[request[1]] = response
                    self.send(response)
                if self.connected and self.reporting and self.mode == 2 and not self.drop and now >= emit:
                    emit = now + 0.15
                    for ident, xyz in [(7, (469, -109, 2229)), (19, (-750, 0, 2600))]:
                        if ident in self.excluded:
                            continue
                        self.tx = (self.tx + 1) & 65535
                        self.socket.sendto(struct.pack('!4H3fhhH6x', 0xDE03, self.tx, self.rx, ident, *(v / 1000 for v in xyz), 123, -153, 1), self.peer)
                try:
                    data, source = self.socket.recvfrom(512)
                except socket.timeout:
                    continue
                assert len(data) == 24, f"Unexpected RDR datagram length: {len(data)}"
                request = list(struct.unpack("!12H", data))
                assert request[0] in (0xED01, 0xED02, 0xED03, 0xED04, 0xED06)
                self.requests.append(request[0])
                self.wire.append(data)
                if self.drop:
                    continue
                self.peer = source
                token = (request[5] << 16) | request[6]
                if request[0] == 0xED01 and not self.connected:
                    self.connected = True
                    self.reporting = False
                    self.mode = 0
                    self.token = token
                    self.cache = {}
                    self.excluded = set()
                if token != self.token:
                    continue
                self.last = now
                if request[1] in self.cache:
                    if self.cache[request[1]] is not None:
                        self.send(self.cache[request[1]])
                    continue
                self.rx = request[1]
                response = None
                if request[0] in (0xED01, 0xED02):
                    response = self.packet(request[0] - 0xF00, request[1], int(self.mode == 2))
                elif request[0] == 0xED03:
                    self.reporting = bool(request[3])
                elif request[0] == 0xED04:
                    if request[4]:
                        self.excluded.discard(request[3])
                    else:
                        self.excluded.add(request[3])
                    response = self.packet(0xDE04, request[1], request[3], request[4])
                elif request[0] == 0xED06:
                    self.mode = 1 if request[3] else 3
                    self.pending = (now + 0.15, request)
                    if request[3]:
                        self.starts += 1
                    else:
                        self.stops += 1
                    response = self.packet(0xDE06, request[1], request[3], result=1)
                self.cache[request[1]] = response
                if response is not None and not self.echo_arguments and request[0] in (0xED04, 0xED06):
                    response[3] = response[4] = 0
                if response is not None:
                    self.send(response)
        except BaseException as error:
            self.error = error

    def close(self):
        self.done = True
        self.thread.join(2)
        self.socket.close()
