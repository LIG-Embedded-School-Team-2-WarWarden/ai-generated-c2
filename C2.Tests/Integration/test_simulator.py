"""Exercise persistent device simulators through the real C2/GUI v3 IPC."""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
from test_station import Gui

ROOT = Path(__file__).resolve().parents[2]


def eventually(action, predicate, timeout=12):
    end = time.monotonic() + timeout
    value = None
    while time.monotonic() < end:
        value = action()
        if predicate(value):
            return value
        time.sleep(.08)
    raise AssertionError(f'timed out: {value}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--console', type=Path, default=ROOT / 'x64/Release/C2.Console.exe')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='c2-simulator-') as directory:
        runtime = Path(directory)
        process = subprocess.Popen([sys.executable, str(ROOT / 'simulators/run.py'), '--headless',
            '--console', str(args.console.resolve()),
            '--runtime', str(runtime)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            creationflags=subprocess.CREATE_NO_WINDOW)
        gui = None
        try:
            def state():
                file = runtime / 'simulator-state.json'
                return json.loads(file.read_text(encoding='utf-8')) if file.exists() else {}
            simulator = eventually(state, lambda s: s.get('rdr_connected') and s.get('received', {}).get('FCS', 0) >= 2)
            gui = Gui(simulator['ipc_suffix'])
            assert gui.command(6, target=1) == 0
            raw = gui.wait(lambda b: struct.unpack_from('<I', b, 128)[0] == 10)
            assert struct.unpack_from('<f', raw, 52)[0] > 12.29
            assert gui.command(1, target=1001) == 0

            def response(command, target=0):
                # Capture fields beyond the basic Gui.command() result.
                gui.sequence += 1
                gui.pipe.write(struct.pack('<7I3f', 0x50495045, 3, command, gui.sequence, 1, 1, target, 0, 0, 0))
                raw = b''
                while len(raw) < 40:
                    part = gui.pipe.read(40 - len(raw))
                    assert part, 'simulator IPC disconnected'
                    raw += part
                fields = struct.unpack('<10I', raw)
                assert fields[:4] == (0x50495045, 3, gui.sequence, command)
                return fields
            eventually(lambda: response(0), lambda r: r[7] == 2)
            assert response(3, 1001)[4] == 0
            eventually(lambda: response(0), lambda r: r[8] == 1)
            assert response(2, 1001)[4] == 0

            # Target 1001 is RDR-local (right,up,forward)=(-1.4,.35,2.8).
            # These independent angle fixtures cover each installation offset,
            # both devices together, and invariance under a common translation.
            for label, rdr_xyz, fcs_xyz, azimuth, elevation in (
                ('RDR moved right', (1, 0, 0), (0, 0, 0), 188.1, 7.1),
                ('FCS moved right', (0, 0, 0), (.6, 0, 0), 215.5, 5.8),
                ('both moved in XYZ', (1, 2, .5), (-.5, .5, .1), 178.7, 9.9),
                ('common translation', (5, -3, 2), (5, -3, 2), 206.6, 6.4),
            ):
                assert gui.command(5, rdr=1, fcs=0, xyz=rdr_xyz) == 0
                assert gui.command(5, rdr=0, fcs=1, xyz=fcs_xyz) == 0
                assert response(1, 1001)[4] == 0
                eventually(lambda: response(0), lambda r: r[7] == 2)
                # The simulator publishes its JSON snapshot every 200ms; wait
                # for that independent observer after C2 confirms alignment.
                measured = eventually(state, lambda s: abs(s.get('fcs_azimuth_deg', -999)-azimuth)<.11
                    and abs(s.get('fcs_elevation_deg', -999)-elevation)<.11, timeout=3)
                assert abs(measured['fcs_azimuth_deg'] - azimuth) < .11, (label, measured, azimuth)
                assert abs(measured['fcs_elevation_deg'] - elevation) < .11, (label, measured, elevation)
                assert response(2, 1001)[4] == 0
            assert response(1, 1001)[4] == 0
            eventually(lambda: response(0), lambda r: r[7] == 2)
            # Reapplying an identical position keeps the current assignment.
            assert gui.command(5, rdr=1, fcs=0, xyz=(5, -3, 2)) == 0
            assert response(0)[7] == 2
            # A changed installation invalidates the old alignment without retargeting.
            assert gui.command(5, rdr=0, fcs=1, xyz=(5.6, -3, 2)) == 0
            assert response(0)[7] == 3
            assert response(3, 1001)[4] != 0
            assert response(1, 1001)[4] == 0
            eventually(lambda: response(0), lambda r: r[7] == 2)
            measured = eventually(state, lambda s: abs(s.get('fcs_azimuth_deg', -999)-215.5)<.11, timeout=3)
            assert abs(measured['fcs_azimuth_deg'] - 215.5) < .11
            assert response(2, 1001)[4] == 0
            # Undefined bearing and mechanical-limit violations stay rejected.
            assert gui.command(5, rdr=1, fcs=0, xyz=(0, 0, 0)) == 0
            assert gui.command(5, rdr=0, fcs=1, xyz=(-1.4, 2.8, 0)) == 0
            assert response(1, 1001)[4] == 1
            assert gui.command(5, rdr=0, fcs=1, xyz=(0, 0, 10)) == 0
            assert response(1, 1001)[4] == 1
            assert gui.command(5, rdr=1, fcs=0, xyz=(0, 0, 0)) == 0
            assert gui.command(5, rdr=0, fcs=1, xyz=(0, 0, 0)) == 0
            print('PASS: RDR/FCS XYZ positions affect FCS angles; common translation and alignment invalidation verified')

            def scenario(value):
                path = runtime / 'control.json'
                temporary = runtime / 'control.update'
                temporary.write_text(json.dumps({'scenario': value, 'target_count': 10}), encoding='utf-8')
                temporary.replace(path)
            scenario('empty')
            gui.wait(lambda b: struct.unpack_from('<I', b, 128)[0] == 0)
            scenario('normal')
            gui.wait(lambda b: struct.unpack_from('<I', b, 128)[0] == 10)
            scenario('fcs-fault')
            gui.wait(lambda b: struct.unpack_from('<I', b, 76)[0] == 0)
            scenario('normal')
            gui.wait(lambda b: struct.unpack_from('<I', b, 76)[0] == 1)
            scenario('fcs-offline')
            gui.wait(lambda b: struct.unpack_from('<I', b, 84)[0] == 0)
            scenario('normal')
            gui.wait(lambda b: struct.unpack_from('<I', b, 84)[0] == 1)
            scenario('rdr-offline')
            gui.wait(lambda b: struct.unpack_from('<I', b, 44)[0] == 0)
            scenario('normal')
            raw = gui.wait(lambda b: struct.unpack_from('<I', b, 44)[0] == 1)
            assert struct.unpack_from('<I', raw, 40)[0] == 0  # Reconnect never restarts a scan.
            assert gui.command(6, target=1) == 0
            gui.wait(lambda b: struct.unpack_from('<I', b, 128)[0] == 10)
            assert gui.command(6, target=0) == 0
            gui.wait(lambda b: struct.unpack_from('<I', b, 128)[0] == 0)
            print('PASS: persistent simulator: 10 observations, assignment/demo/cancel, target loss, FCS fault/offline, RDR reconnect and GUI start/stop')
        finally:
            if gui:
                gui.close()
            (runtime / 'stop.txt').touch()
            try:
                out, error = process.communicate(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                out, error = process.communicate()
                raise AssertionError(f'simulator cleanup failed: {error!r}')
            if process.returncode:
                raise AssertionError(f'simulator exited {process.returncode}: {out!r} {error!r}')


if __name__ == '__main__':
    main()
