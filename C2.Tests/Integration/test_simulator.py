"""Exercise persistent simulators through frame-v3 / command-v3 IPC."""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
from test_station import Gui, target_records, valid_count

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
        process = subprocess.Popen([sys.executable, str(ROOT / 'simulators/run.py'), '--headless', '--isolated-ipc',
            '--console', str(args.console.resolve()),
            '--runtime', str(runtime)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            creationflags=subprocess.CREATE_NO_WINDOW)
        gui = None
        try:
            def state():
                file = runtime / 'simulator-state.json'
                try:
                    return json.loads(file.read_text(encoding='utf-8')) if file.exists() else {}
                except (PermissionError, FileNotFoundError, json.JSONDecodeError):
                    return {}  # Retry while Windows replaces the simulator snapshot.
            simulator = eventually(state, lambda s: s.get('rdr_connected') and s.get('received', {}).get('FCS', 0) >= 2)
            gui = Gui(simulator['ipc_suffix'])
            assert gui.command(6, target=1) == 0
            raw = gui.wait(lambda b: valid_count(b) == 10)
            first = next(t for t in target_records(raw) if t[0] == 1001)
            assert all(abs(a-b)<1e-5 for a,b in zip(first[4:7], (-1.4,2.8,.35)))
            initial_pose = struct.unpack_from('<2f', raw, 48)
            expected = {t[0]: t[4:7] for t in target_records(raw)}
            # Require both reported angles to change; a fixed motor must not
            # accidentally pass the fixed-world-coordinate regression.
            def both_axes_moved(frame):
                pose = struct.unpack_from('<2f', frame, 48)
                return valid_count(frame) == 10 and all(
                    abs(a-b) >= .2 for a,b in zip(pose, initial_pose))
            for _ in range(2):
                raw = gui.wait(both_axes_moved)
                for target in target_records(raw):
                    assert all(abs(a-b)<1e-5 for a,b in zip(target[4:7], expected[target[0]]))
                initial_pose = struct.unpack_from('<2f', raw, 48)
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
            events = runtime / 'events.log'

            def diagnostic_rows(event):
                if not events.exists():
                    return []
                return [dict(field.split('=', 1) for field in line.split() if '=' in field)
                        for line in events.read_text(encoding='utf-8').splitlines() if event in line]

            busy = eventually(lambda: diagnostic_rows('FCS busy_observed=1'),
                              lambda rows: any(int(row['flags']) & 8 for row in rows))[-1]
            busy_response = response(7)
            assert (busy_response[4], busy_response[8], busy_response[9]) == (0, 1, 1), busy_response
            blocked = response(1, 1002)
            assert blocked[4] == 2, blocked
            request = eventually(lambda: diagnostic_rows('GUI assignment_received'),
                                 lambda rows: rows and rows[-1]['request_id'] == str(blocked[2]))[-1]
            assert request['busy_observed'] == '1', request
            assert int(busy['status_rx_us']) <= int(request['status_rx_us']) <= int(request['request_rx_us'])
            eventually(lambda: response(0), lambda r: r[8] == 1)
            cleared = eventually(lambda: diagnostic_rows('FCS busy_observed=0'),
                                 lambda rows: rows and int(rows[-1]['status_rx_us']) > int(request['request_rx_us']))[-1]
            assert cleared['observation'] == 'change', cleared
            assert response(2, 1001)[4] == 0
            eventually(lambda: response(7), lambda r: (r[4], r[8], r[9]) == (0, 0, 1))
            # FCS completion is not RDR reacquisition. Wait for fresh objects
            # before testing ordinary installation-coordinate edits.
            gui.wait(lambda frame: valid_count(frame) == 10)

            # Target 1001 is fixed at base (east,north,up)=(-1.4,2.8,.35).
            # These independent angle fixtures cover each installation offset,
            # both devices together, and invariance under a common translation.
            for label, rdr_xyz, fcs_xyz, azimuth, elevation in (
                ('RDR moved right', (1, 0, 0), (0, 0, 0), 171.9, 7.1),
                ('FCS moved right', (0, 0, 0), (.6, 0, 0), 144.5, 5.8),
                ('both moved in XYZ', (1, 2, .5), (-.5, .5, .1), 181.3, 9.9),
                ('common translation', (5, -3, 2), (5, -3, 2), 153.4, 6.4),
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
            measured = eventually(state, lambda s: abs(s.get('fcs_azimuth_deg', -999)-144.5)<.11, timeout=3)
            assert abs(measured['fcs_azimuth_deg'] - 144.5) < .11
            assert response(2, 1001)[4] == 0
            # Undefined bearing is still rejected locally. Mechanical-limit
            # violations are sent to the simulated device, which rejects them.
            assert gui.command(5, rdr=1, fcs=0, xyz=(0, 0, 0)) == 0
            assert gui.command(5, rdr=0, fcs=1, xyz=(-1.4, 2.8, 0)) == 0
            assert response(1, 1001)[4] == 1
            assert gui.command(5, rdr=0, fcs=1, xyz=(0, 0, 10)) == 0
            assert response(1, 1001)[4] == 6
            assert gui.command(5, rdr=1, fcs=0, xyz=(0, 0, 0)) == 0
            assert gui.command(5, rdr=0, fcs=1, xyz=(0, 0, 0)) == 0
            print('PASS: RDR/FCS XYZ positions affect FCS angles; common translation and alignment invalidation verified')

            def scenario(value):
                path = runtime / 'control.json'
                temporary = runtime / 'control.update'
                temporary.write_text(json.dumps({'scenario': value, 'target_count': 10}), encoding='utf-8')
                temporary.replace(path)
            scenario('empty')
            raw = gui.wait(lambda b: valid_count(b) == 0)
            assert {t[0] for t in target_records(raw)} == set(range(1001, 1011))
            assert all(t[4:7] == (0, 0, 0) and t[9] == 0
                for t in target_records(raw))
            scenario('normal')
            gui.wait(lambda b: valid_count(b) == 10)
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
            gui.wait(lambda b: valid_count(b) == 10)
            assert gui.command(6, target=0) == 0
            gui.wait(lambda b: valid_count(b) == 0)
            print('PASS: persistent simulator: raw/pose/world invariance, 10 observations, assignment/demo/cancel, target loss, FCS fault/offline, RDR reconnect and GUI start/stop')
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
