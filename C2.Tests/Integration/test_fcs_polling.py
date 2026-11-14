"""Loopback regression: delayed replies, transient loss and genuine disconnect.

No commands are sent to physical devices. Runs the production console polling loop.
"""
import argparse
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--console', type=Path, required=True)
    args = parser.parse_args()
    requests, errors = [], []
    done, stopping = threading.Event(), threading.Event()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server, \
            tempfile.TemporaryDirectory(prefix='fcs-polling-', dir=ROOT) as directory:
        server.bind(('127.0.0.1', 0))
        server.settimeout(.1)
        runtime = Path(directory)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(('127.0.0.1', 0))
            rdr_port = probe.getsockname()[1]
        stop = runtime / 'stop'
        event_log = runtime / 'events.log'

        def serve():
            sequence = status_count = connects = 0
            try:
                while not stopping.is_set():
                    try:
                        raw, peer = server.recvfrom(256)
                    except socket.timeout:
                        continue
                    kind, request_sequence = struct.unpack_from('!HH', raw)
                    requests.append((kind, time.monotonic(), status_count))
                    if kind == 0xEF01:
                        connects += 1
                        assert connects == 1 or status_count >= 10, 'premature reconnect'
                        reply = struct.pack('!4H', 0xFE01, sequence, request_sequence, 0)
                    elif kind == 0xEF02:
                        status_count += 1
                        # Two misses must recover without a handshake. Three later
                        # misses must still disconnect and trigger a handshake.
                        if status_count in (5, 6, 8, 9, 10):
                            continue
                        if status_count <= 4:
                            time.sleep(.12)  # Valid response rejected by old 80 ms window.
                        reply = struct.pack('!6H', 0xFE02, sequence, request_sequence, 1, 0, 0)
                        if status_count == 11:
                            assert connects == 2, 'three misses did not reconnect'
                            done.set()
                    elif kind == 0xEF04:  # Console shutdown sends OFF to this dummy only.
                        assert struct.unpack_from('!H', raw, 6)[0] == 0
                        reply = struct.pack('!6H', 0xFE04, sequence, request_sequence, 0, 0, 0)
                    else:
                        raise AssertionError(f'unexpected request {kind:x}')
                    server.sendto(reply, peer)
                    sequence = (sequence + 1) & 65535
            except BaseException as error:
                errors.append(error)
                done.set()

        thread = threading.Thread(target=serve)
        thread.start()
        with (runtime / 'stdout.log').open('w') as output:
            process = subprocess.Popen([
                str(args.console.resolve()), '--headless', '--bind', '127.0.0.1',
                '--fcs-ip', '127.0.0.1', '--fcs-port', str(server.getsockname()[1]),
                '--fcs-local-port', '0', '--rdr-local-ip', '127.0.0.1',
                '--rdr-ip', '127.0.0.1', '--rdr-local-port', str(rdr_port), '--rdr-port', '59999',
                '--ipc-suffix', '_polling_' + uuid.uuid4().hex,
                '--stop-file', str(stop), '--event-log', str(event_log)],
                cwd=runtime, stdout=output, stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW)
            try:
                deadline = time.monotonic() + 25
                while not done.wait(.1) and time.monotonic() < deadline:
                    assert process.poll() is None, (runtime / 'stdout.log').read_text()
                assert done.is_set(), 'polling scenario timed out: ' + (runtime / 'stdout.log').read_text()[-4000:]
                if errors:
                    raise errors[0]
            finally:
                stop.touch()
                try:
                    process.wait(timeout=8)
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.wait()
                    stopping.set()
                    thread.join(timeout=2)
            assert process.returncode == 0, (runtime / 'stdout.log').read_text()
        polls = [(kind, t) for kind, t, _ in requests if kind in (0xEF01, 0xEF02)]
        # A connection now includes one immediate read-only freshness probe.
        gaps = [b[1] - a[1] for a, b in zip(polls, polls[1:]) if a[0] != 0xEF01]
        assert all(.9 <= gap < 2 for gap in gaps), gaps
        log = event_log.read_text()
        assert log.count('FCS EF02 response_timeout') == 5, log[-5000:]
        assert log.count('reason=consecutive_status_timeouts') == 1, log[-5000:]
        assert log.count('reason=fresh_status_confirmed') == 2, log[-5000:]
        assert 'status_missed count=2 limit=3' in log
        print('PASS: 120ms replies accepted; 1Hz polling; two misses recover; '
              'three misses disconnect and reconnect; polling gaps '
              f'{min(gaps):.3f}..{max(gaps):.3f}s')


if __name__ == '__main__':
    main()
