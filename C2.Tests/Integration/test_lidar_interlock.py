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
from test_station import Gui, valid_count


class Probe(Devices):
    def __init__(self):
        self.events = []
        self.legacy = self.bad_stop = self.drop_stop = self.drop_on_reply = False
        self.manual_off_busy = self.manual_off_drop = self.manual_off_wrong_state = False
        self.lost_stop_ack = False
        self.off_requests = 0
        super().__init__()

    def _send_rdr(self, fields):
        if getattr(self, 'boot_resume_drop', False) and fields[0] == 0xDE07 and fields[3] == 1:
            return
        if self.lost_stop_ack and fields[0] == 0xDE07 and fields[3] == 0:
            return  # The device applied OFF, but every completion reply is lost.
        super()._send_rdr(fields)

    def _packet(self, kind, *args, **kwargs):
        p = super()._packet(kind, *args, **kwargs)
        if self.legacy:
            p[10] &= ~120
        if getattr(self, 'boot_unknown', False):
            p[10] &= ~16
        if self.bad_stop and kind == 0xDE07 and p[3] == 0:
            p[10] |= 32  # A success result with actual ON must never permit FCS ON.
        return p

    def _rdr_request(self, raw, peer, now):
        if len(raw) == 24 and struct.unpack_from('!H', raw)[0] == 0xED06:
            self.events.append(('rdr_start', struct.unpack_from('!H', raw, 6)[0]))
        if len(raw) == 24 and struct.unpack_from('!H', raw)[0] == 0xED07:
            value = struct.unpack_from('!H', raw, 6)[0]
            self.events.append(('rdr', value))
            if value == 0 and self.drop_stop:
                return
        super()._rdr_request(raw, peer, now)

    def _fcs_request(self, raw, peer, now):
        if getattr(self, 'boot_fcs_disconnected', False):
            self.events.append(('fcs_disconnected_request', struct.unpack_from('!H', raw)[0]))
            return
        kind = struct.unpack_from('!H', raw)[0]
        if kind in (0xEF04, 0xEF05):
            value = struct.unpack_from('!H', raw, 6)[0] if kind == 0xEF04 else 1
            self.events.append(('fcs', value, self.lidar_inhibited))
            if value and not self.lidar_inhibited:
                raise AssertionError('FCS output preceded physical lidar inhibition')
            if value and self.drop_on_reply:
                self.laser_until = now + 60
                return
            if not value:
                self.off_requests += 1
                if getattr(self, 'boot_off_failure', False):
                    self.fcs_tx = (self.fcs_tx + 1) & 65535
                    self.fcs.sendto(struct.pack('!6H', 0xFE04, self.fcs_tx,
                        struct.unpack_from('!H', raw, 2)[0], 3, 0, 0), peer)
                    return
                if self.off_requests == 1 and (self.manual_off_busy or self.manual_off_drop or self.manual_off_wrong_state):
                    if not self.manual_off_drop:
                        self.fcs_tx = (self.fcs_tx + 1) & 65535
                        self.fcs.sendto(struct.pack('!6H', 0xFE04, self.fcs_tx,
                            struct.unpack_from('!H', raw, 2)[0], int(self.manual_off_busy), 1, 0), peer)
                    return
        super()._fcs_request(raw, peer, now)
        if kind == 0xEF04 and struct.unpack_from('!H', raw, 6)[0] == 1:
            self.laser_until = now + 60  # Remain ON until ECS successfully sends OFF.


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
    boot = scenario.startswith('boot_') or scenario == 'gui_off_no_target'
    if boot:
        board.lidar_inhibited = scenario != 'boot_no_inhibit'
        board.manual_off_busy = scenario == 'boot_off_busy'
        board.manual_off_drop = scenario == 'boot_off_drop'
    process = None
    gui = None
    try:
        with tempfile.TemporaryDirectory(prefix='c2-lidar-', ignore_cleanup_errors=True) as directory:
            config = json.loads((ROOT / 'configs/team-demo.json').read_text(encoding='utf-8'))
            for name, port in [('rdr', board.rdr_port), ('fcs', board.fcs_port)]:
                config[name]['local'] = {'ip': '127.0.0.1', 'port': 0}
                config[name]['peer'] = {'ip': '127.0.0.1', 'port': port}
            if scenario == 'boot_fcs_disabled':
                config['fcs']['enabled'] = False
            path = Path(directory) / 'config.json'
            path.write_text(json.dumps(config), encoding='utf-8')
            suffix = '_lidar_' + uuid.uuid4().hex
            process = subprocess.Popen([str(executable), '--config', str(path),
                '--ipc-suffix', suffix,
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
            if scenario not in ('boot_fcs_disconnected', 'boot_fcs_disabled'):
                wait_for(lambda: ask('fcs status')['connected'])
            if boot:
                gui = Gui(suffix)
                assert valid_count(gui.frame()) == 0
                if scenario == 'gui_off_no_target':
                    assert gui.command(2) == 0
                    wait_for(lambda: not board.lidar_inhibited)
                    assert board.mode == 0  # Recovery does not start surveillance.
                    board.events.clear()
                if scenario == 'boot_unknown':
                    assert gui.command(6, target=1) == 16
                    assert ('rdr_start', 1) not in board.events and ('rdr', 1) not in board.events
                    assert board.off_requests == 0
                    gui.close(); gui = None
                    process.stdin.write('quit\n'); process.stdin.flush(); process.wait(timeout=8)
                    assert process.returncode == 0
                    print('PASS: lidar interlock', scenario)
                    return
                assert gui.command(6, target=1) == 0
                if scenario in ('boot_resume_drop', 'boot_off_failure'):
                    expected = 21 if scenario == 'boot_resume_drop' else 22
                    wait_for(lambda: gui.command(18) == expected, timeout=15)
                    assert struct.unpack_from('<I', gui.frame(), 40)[0] == 0
                    assert not any(e[:2] == ('fcs', 1) for e in board.events)
                    if scenario == 'boot_off_failure':
                        assert ('rdr_start', 1) not in board.events and ('rdr', 1) not in board.events
                    gui.close(); gui = None
                    process.stdin.write('quit\n'); process.stdin.flush(); process.wait(timeout=8)
                    assert process.returncode == 0
                    print('PASS: lidar interlock', scenario)
                    return
                wait_for(lambda: not board.lidar_inhibited)
                gui.wait(lambda frame: valid_count(frame) == 10)
                if scenario in ('boot_off_busy', 'boot_off_drop'):
                    assert board.off_requests >= 2, board.events
                assert not any(e[:2] == ('fcs', 1) for e in board.events), board.events
                off_event = ('fcs', 0, scenario not in ('boot_no_inhibit', 'gui_off_no_target'))
                if scenario in ('boot_fcs_disconnected', 'boot_fcs_disabled'):
                    assert board.off_requests == 0, board.events
                    assert not any(e[0] == 'fcs_disconnected_request' and e[1] in (0xEF04, 0xEF05) for e in board.events), board.events
                    assert board.events.index(('rdr_start', 1)) < board.events.index(('rdr', 1))
                else:
                    assert board.events.index(off_event) < board.events.index(('rdr_start', 1)) < board.events.index(('rdr', 1))
                wait_for(lambda: gui.command(18) == 0)
                assert struct.unpack_from('<I', gui.frame(), 40)[0] == 1
                assert board.error is None, board.error
                gui.close(); gui = None
                process.stdin.write('quit\n'); process.stdin.flush()
                process.wait(timeout=8)
                assert process.returncode == 0, process.stderr.read()
                print('PASS: lidar interlock', scenario)
                return
            ask('rdr start')
            wait_for(lambda: ask('rdr status')['mode'] == 2)
            # Mode may be published before the startup command completes.
            wait_for(lambda: ask('rdr status')['result'] == 0)
            wait_for(lambda: ask('fcs status').get('flags') == 1)
            if scenario == 'legacy':
                ask('rdr report on')
                wait_for(lambda: ask('rdr status')['reporting'])
                gui = Gui(suffix)
                gui.wait(lambda frame: valid_count(frame) == 10)
                assert gui.command(1, target=1001) == 0
                def assigned():
                    gui.command(0)
                    return gui.last_response[7] == 2
                wait_for(assigned)
                assert gui.command(3, target=1001) == 15, gui.last_response
                assert not any(e[:2] == ('fcs', 1) for e in board.events), board.events
                logged = (Path(directory) / 'events.log').read_text(encoding='utf-8')
                assert 'lidar_unsupported supported=0' in logged, logged[-3000:]
                gui.close(); gui = None
            reply = ask('fcs laser on')
            if scenario == 'lost_stop_ack':
                assert 'rejected' in reply, reply
                assert board.lidar_inhibited
                assert not any(e[:2] == ('fcs', 1) for e in board.events)
                stop_count = board.events.count(('rdr', 0))
                # Recovery must complete well before the former 40-second timeout.
                recovered_at = time.monotonic()
                assert ask('fcs laser off')['result'] == 0
                wait_for(lambda: not board.lidar_inhibited, timeout=5)
                assert time.monotonic() - recovered_at < 5
                assert board.events.count(('rdr', 0)) == stop_count, board.events
                assert ('rdr', 1) in board.events
            elif scenario in ('legacy', 'bad_stop', 'drop_stop'):
                assert 'rejected' in reply, reply
                assert not any(e[:2] == ('fcs', 1) for e in board.events), board.events
                time.sleep(.3)
                assert not any(e == ('rdr', 1) for e in board.events), board.events
            else:
                assert reply.get('result') == (-1 if scenario == 'drop_on_reply' else 0), reply
                assert board.lidar_inhibited
                if scenario.startswith('manual_off_'):
                    off = ask('fcs laser off')
                    expected = {'manual_off_busy': 1, 'manual_off_drop': -1, 'manual_off_wrong_state': 0}[scenario]
                    assert off['result'] == expected, off
                    assert board.lidar_inhibited
                wait_for(lambda: not board.lidar_inhibited)
                assert board.laser_until <= time.monotonic()
                if scenario.startswith('manual_off_'):
                    assert board.off_requests >= 2, board.events
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
        if gui:
            gui.close()
        if process and process.poll() is None:
            process.kill();process.wait()
        board.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--console', type=Path, required=True)
    args = parser.parse_args()
    for case in ('normal', 'legacy', 'bad_stop', 'drop_stop', 'drop_on_reply',
                 'manual_off_busy', 'manual_off_drop', 'manual_off_wrong_state', 'lost_stop_ack',
                 'boot_inhibited', 'boot_off_busy', 'boot_off_drop', 'gui_off_no_target',
                 'boot_no_inhibit', 'boot_resume_drop', 'boot_off_failure', 'boot_fcs_disconnected',
                 'boot_fcs_disabled', 'boot_unknown'):
        run(args.console.resolve(), case)
