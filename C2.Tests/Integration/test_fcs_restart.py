"""Restart ECS against a persistent same-peer FCS replay cache (loopback only)."""
import argparse
import contextlib
import io
import json
from pathlib import Path
import queue
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'simulators'))
from fcs_dummy import FcsDummy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--console', type=Path, required=True)
    args = parser.parse_args()
    errors, rejected_controls = [], []
    done = threading.Event()
    board = FcsDummy()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server, \
            tempfile.TemporaryDirectory(prefix='fcs-restart-', dir=ROOT) as directory:
        server.bind(('127.0.0.1', 0)); server.settimeout(.1)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(('127.0.0.1', 0)); client_port = probe.getsockname()[1]
        peer = ('127.0.0.1', client_port)
        board.owner = peer
        # A previous ECS queried these numbers, then exited without clearing FCS.
        for seq in range(2, 61):
            board.receive(struct.pack('!4H', 0xEF02, seq, 0, 0), peer, time.monotonic())
        board.tilt = 123  # Cached replies still contain zero: must not be accepted.

        def serve():
            try:
                while not done.is_set():
                    try:
                        raw, source = server.recvfrom(256)
                    except socket.timeout:
                        continue
                    kind = struct.unpack_from('!H', raw)[0]
                    assert source == peer, 'test must preserve the UDP endpoint'
                    assert kind in (0xEF01, 0xEF02, 0xEF04), kind
                    if kind == 0xEF04:
                        assert struct.unpack_from('!H', raw, 6)[0] == 0
                    with contextlib.redirect_stdout(io.StringIO()):
                        reply = board.receive(raw, source, time.monotonic())
                    if reply:
                        if kind == 0xEF04 and struct.unpack_from('!H', reply, 6)[0]:
                            rejected_controls.append(raw.hex())
                        server.sendto(reply, source)
            except BaseException as error:
                errors.append(error)

        thread = threading.Thread(target=serve); thread.start()
        try:
            for run in range(5):
                runtime = Path(directory)
                config = json.loads((ROOT / 'configs/team-demo.json').read_text())
                config['fcs']['enabled'] = True
                config['fcs']['local'] = {'ip': '127.0.0.1', 'port': client_port}
                config['fcs']['peer'] = {'ip': '127.0.0.1', 'port': server.getsockname()[1]}
                config['rdr']['local'] = {'ip': '127.0.0.1', 'port': 0}
                config['rdr']['peer'] = {'ip': '127.0.0.1', 'port': 59999}
                path = runtime / 'config.json'; path.write_text(json.dumps(config))
                log = runtime / f'events-{run}.log'
                process = subprocess.Popen([str(args.console.resolve()), '--config', str(path),
                    '--ipc-suffix', '_restart_' + uuid.uuid4().hex, '--event-log', str(log)],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    text=True, creationflags=subprocess.CREATE_NO_WINDOW)
                output = queue.Queue()
                connected = threading.Event()

                def reader(stream):
                    for line in stream:
                        if 'connection_changed connected=1' in line:
                            connected.set()
                        try: output.put(json.loads(line))
                        except json.JSONDecodeError: pass

                reader_thread = threading.Thread(target=reader, args=(process.stdout,), daemon=True)
                reader_thread.start()
                try:
                    assert output.get(timeout=5)['ready']
                    assert connected.wait(8), log.read_text()[-4000:]
                    process.stdin.write('fcs status\n'); process.stdin.flush()
                    status = output.get(timeout=8)
                    assert status.get('connected'), status
                    assert status.get('confirmed'), status
                    process.stdin.write('fcs laser off\n'); process.stdin.flush()
                    off = output.get(timeout=5)
                    assert off.get('confirmed') and off.get('result') == 0, off
                    assert not rejected_controls, rejected_controls
                    text = log.read_text()
                    assert 'session_ready' in text, text[-3000:]
                    # The first session must reject a cached FE02, then find a
                    # fresh one. Later sessions exercise quarter-ring advancement.
                    if run == 0:
                        assert 'reason=stale_response_sequence' in text
                    fresh = [line for line in text.splitlines() if 'rx_match id=0xFE02' in line]
                    assert fresh and all(line.endswith('007b0708') for line in fresh), fresh
                    if run % 2:
                        process.kill()  # Abrupt Visual Studio stop, no graceful cleanup.
                    else:
                        process.stdin.write('quit\n'); process.stdin.flush()
                    process.wait(timeout=8)
                    if run % 2 == 0: assert process.returncode == 0, process.stderr.read()
                finally:
                    if process.poll() is None: process.kill(); process.wait()
                    reader_thread.join(timeout=1)
                    process.stdin.close(); process.stdout.close(); process.stderr.close()
                if errors: raise errors[0]
            print('PASS: five same-port ECS starts, retained FCS cache, fresh status, '
                  'OFF accepted, graceful and abrupt exits')
        finally:
            done.set(); thread.join(timeout=2)


if __name__ == '__main__':
    main()
