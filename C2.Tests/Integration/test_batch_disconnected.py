"""GUI batch IPC routing with FCS disabled and no device/simulator processes."""
import argparse
import json
from pathlib import Path
import queue
import socket
import struct
import subprocess
import tempfile
import threading
import uuid

from test_station import Gui

ROOT = Path(__file__).resolve().parents[2]


def run(executable):
    process = gui = None
    # Reserve a loopback sink. It never responds, so ECS cannot connect a device.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sink, tempfile.TemporaryDirectory(
            prefix='batch-ipc-', ignore_cleanup_errors=True) as directory:
        sink.bind(('127.0.0.1', 0))
        config = json.loads((ROOT / 'configs/team-demo.json').read_text())
        config['fcs']['enabled'] = False
        for name in ('rdr', 'fcs'):
            config[name]['local'] = {'ip': '127.0.0.1', 'port': 0}
            config[name]['peer'] = {'ip': '127.0.0.1', 'port': sink.getsockname()[1]}
        path = Path(directory) / 'config.json'
        path.write_text(json.dumps(config))
        stop = Path(directory) / 'stop'
        suffix = '_batch_ipc_' + uuid.uuid4().hex
        try:
            process = subprocess.Popen([str(executable), '--config', str(path), '--headless',
                '--stop-file', str(stop), '--ipc-suffix', suffix,
                '--event-log', str(Path(directory) / 'events.log')],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            ready = queue.Queue()

            def read_output():
                for line in process.stdout:
                    try:
                        data = json.loads(line)
                        if data.get('ready'):
                            ready.put(data)
                    except json.JSONDecodeError:
                        pass

            threading.Thread(target=read_output, daemon=True).start()
            assert ready.get(timeout=10)['ready']
            gui = Gui(suffix)
            for target in (0, 7, 65535):
                assert gui.command(1200, target=target) == 0, gui.last_response
                assert gui.last_response[5:] == (1, target, 0, 0, 0), gui.last_response
            for code in range(1200, 1208):
                assert gui.command(code, target=7, rdr=2) == 1
                assert gui.command(code, target=7, fcs=2) == 1
                assert gui.command(code, target=65536) == 1
                assert gui.command(code, target=7, version=2) == 1
            for code in (1201, 1203, 1204, 1205, 1206, 1207):
                assert gui.command(code, target=7) == 19, (code, gui.last_response)
            assert gui.command(1202, target=7) == 0  # idempotent release of absent row
            assert gui.command(1010) == 5  # original manual fallback remains available
            assert gui.command(19) == 2
            assert gui.command(20) == 2
            gui.frame()  # unchanged 492-byte ABI and checksum
            with open('\\\\.\\pipe\\ECS_RdrControl' + suffix, 'r+b', buffering=0) as pipe:
                pipe.write(struct.pack('<7I3f', 0x50495045, 3, 1200, 1, 1, 1, 0, 0, 0, 0))
                reply = b''
                while len(reply) < 40:
                    part = pipe.read(40 - len(reply))
                    assert part, 'control pipe disconnected'
                    reply += part
                assert struct.unpack('<10I', reply)[4] == 1  # wrong pipe
            gui.close()
            gui = None
            stop.touch()
            process.wait(timeout=10)
            assert process.returncode == 0, process.stderr.read()
            print('PASS: batch IPC discovery, offline rejection, IDs, version, pipe routing; no FCS process')
        finally:
            if gui:
                gui.close()
            if process and process.poll() is None:
                process.kill()
                process.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--console', type=Path, required=True)
    args = parser.parse_args()
    run(args.console.resolve())
