"""Optical-exclusion scenarios using loopback only; no physical output."""
import argparse
import json
from pathlib import Path
import queue
import struct
import subprocess
import sys
import tempfile
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'simulators'))
from devices import Devices


class Probe(Devices):
    def __init__(self):
        self.events = []
        self.legacy = self.bad_stop = self.drop_stop = self.drop_on_reply = False
        super().__init__()

    def _packet(self, kind, *args, **kwargs):
        p = super()._packet(kind, *args, **kwargs)
        if self.legacy:
            p[10] &= ~120
        if self.bad_stop and kind == 0xDE07 and p[3] == 0:
            p[10] |= 32  # A success result with actual ON must never permit FCS ON.
        return p

    def _rdr_request(self, raw, peer, now):
        if len(raw) == 24 and struct.unpack_from('!H', raw)[0] == 0xED07:
            value = struct.unpack_from('!H', raw, 6)[0]
            self.events.append(('rdr', value))
            if value == 0 and self.drop_stop:
                return
        super()._rdr_request(raw, peer, now)

    def _fcs_request(self, raw, peer, now):
        kind = struct.unpack_from('!H', raw)[0]
        if kind in (0xEF04, 0xEF05):
            value = struct.unpack_from('!H', raw, 6)[0] if kind == 0xEF04 else 1
            self.events.append(('fcs', value, self.lidar_inhibited))
            if value and not self.lidar_inhibited:
                raise AssertionError('FCS output preceded physical lidar inhibition')
            if value and self.drop_on_reply:
                self.laser_until = now + 1
                return
        super()._fcs_request(raw, peer, now)


def wait_for(predicate, timeout=6):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError('condition timed out')


def run(executable, scenario):
    board = Probe()
    setattr(board, scenario, True) if scenario != 'normal' else None
    process = None
    try:
        with tempfile.TemporaryDirectory(prefix='c2-lidar-', ignore_cleanup_errors=True) as directory:
            config = json.loads((ROOT / 'configs/team-demo.json').read_text(encoding='utf-8'))
            for name, port in [('rdr', board.rdr_port), ('fcs', board.fcs_port)]:
                config[name]['local'] = {'ip': '127.0.0.1', 'port': 0}
                config[name]['peer'] = {'ip': '127.0.0.1', 'port': port}
            path = Path(directory) / 'config.json'
            path.write_text(json.dumps(config), encoding='utf-8')
            process = subprocess.Popen([str(executable), '--config', str(path),
                '--ipc-suffix', '_lidar_' + uuid.uuid4().hex,
                '--event-log', str(Path(directory) / 'events.log')],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            output = queue.Queue()
            def reader():
                for line in process.stdout:
                    try:
                        output.put(json.loads(line))
                    except json.JSONDecodeError:
                        pass
            threading.Thread(target=reader, daemon=True).start()
            assert output.get(timeout=8)['ready']
            def ask(command):
                process.stdin.write(command + '\n')
                process.stdin.flush()
                return output.get(timeout=6)
            wait_for(lambda: ask('rdr status')['connected'])
            wait_for(lambda: ask('fcs status')['connected'])
            ask('rdr start')
            wait_for(lambda: ask('rdr status')['mode'] == 2)
            reply = ask('fcs laser on')
            if scenario in ('legacy', 'bad_stop', 'drop_stop'):
                assert 'rejected' in reply, reply
                assert not any(e[:2] == ('fcs', 1) for e in board.events), board.events
                time.sleep(.3)
                assert not any(e == ('rdr', 1) for e in board.events), board.events
            else:
                assert reply['result'] == (-1 if scenario == 'drop_on_reply' else 0), reply
                assert board.lidar_inhibited
                wait_for(lambda: not board.lidar_inhibited)
                relevant = [e[:2] for e in board.events]
                pause = relevant.index(('rdr', 0))
                on = relevant.index(('fcs', 1))
                off = relevant.index(('fcs', 0))
                resume = relevant.index(('rdr', 1))
                assert pause < on < off < resume, relevant
                assert board.mode == 2
                if scenario == 'normal':
                    # A pre-existing stopped surveillance mode must stay stopped.
                    ask('rdr stop')
                    wait_for(lambda: ask('rdr status')['mode'] == 0)
                    second = ask('fcs laser on')
                    assert second.get('result') == 0, second
                    wait_for(lambda: not board.lidar_inhibited)
                    assert board.mode == 0
            assert board.error is None, board.error
            process.stdin.write('quit\n');process.stdin.flush()
            process.wait(timeout=8)
            assert process.returncode == 0, process.stderr.read()
            print('PASS: lidar interlock', scenario)
    finally:
        if process and process.poll() is None:
            process.kill();process.wait()
        board.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--console', type=Path, required=True)
    args = parser.parse_args()
    for case in ('normal', 'legacy', 'bad_stop', 'drop_stop', 'drop_on_reply'):
        run(args.console.resolve(), case)
