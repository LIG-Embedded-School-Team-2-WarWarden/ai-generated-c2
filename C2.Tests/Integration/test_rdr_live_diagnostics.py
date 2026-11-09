"""Read-only live diagnostics against a loopback lidar; no FCS or hardware."""
import argparse
import csv
import json
from pathlib import Path
import queue
import subprocess
import tempfile
import threading
import time
import uuid
from rdr_board import Board
from test_station import Gui, target_records, valid_count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--console', type=Path, required=True)
    parser.add_argument('--filtered', action='store_true')
    parser.add_argument('--adaptive', action='store_true')
    parser.add_argument('--axes', action='store_true', help='exercise the live XYZ noise profile on loopback')
    args = parser.parse_args()
    if args.adaptive and not args.filtered:
        parser.error('--adaptive requires --filtered')
    if args.axes and not args.adaptive:
        parser.error('--axes requires --adaptive --filtered')
    root = Path(__file__).resolve().parents[2]
    board = Board()
    board.observations = [(7, (-2, 1, 3), 0, 1800)]
    process = gui = None
    try:
        with tempfile.TemporaryDirectory() as directory:
            config = {
                'rdr': {'protocol': 'edde24', 'local': {'ip': '127.0.0.1', 'port': 0},
                        'peer': {'ip': '127.0.0.1', 'port': board.port},
                        'gui_kalman': {'enabled': args.filtered,
                                       'measurement_stddev_m': .03 if args.adaptive else .05,
                                       'process_stddev_m_sqrt_s': .01 if args.adaptive else .03,
                                       'adaptive_motion': args.adaptive,
                                       'motion_process_stddev_m_sqrt_s': .08,
                                       'motion_trigger_sigma': 2.5}},
                'fcs': {'protocol': 'effe', 'enabled': False,
                        'local': {'ip': '127.0.0.1', 'port': 0},
                        'peer': {'ip': '127.0.0.1', 'port': 1}},
            }
            if args.axes:
                # Copy only filter parameters. All network endpoints stay on loopback.
                config['rdr']['gui_kalman'] = json.loads((root / 'configs/rdr-live.json').read_text(
                    encoding='utf-8'))['rdr']['gui_kalman']
            path = Path(directory) / 'config.json'
            path.write_text(json.dumps(config), encoding='utf-8')
            suffix = '_live_test_' + uuid.uuid4().hex
            recording = Path(directory) / 'coordinates.csv'
            process = subprocess.Popen(
                [str(args.console.resolve()), '--config', str(path),
                 '--ipc-suffix', suffix, '--rdr-record', str(recording),
                 '--rdr-record-seconds', '2'],
                cwd=root, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            output = queue.Queue()

            def read():
                for line in process.stdout:
                    try:
                        output.put(json.loads(line))
                    except json.JSONDecodeError:
                        pass

            reader = threading.Thread(target=read, daemon=True)
            reader.start()

            def wait(predicate, timeout=6):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    try:
                        value = output.get(timeout=max(.001, deadline - time.monotonic()))
                    except queue.Empty:
                        break
                    if predicate(value):
                        return value
                raise AssertionError('diagnostic output did not reach expected state')

            def send(command):
                process.stdin.write(command + '\n')
                process.stdin.flush()

            def live(predicate):
                return wait(lambda value: isinstance(value, dict) and
                            value.get('type') == 'rdr_live' and predicate(value))

            def xyz(expected):
                def matches(value):
                    return any(row['id'] == 7 and row['world_valid'] and
                               all(abs(a-b) < 1e-5 for a, b in zip(row['world_xyz'], expected))
                               for row in value['targets'])
                return matches

            def gui_xyz(expected):
                tolerance = .001 if args.filtered else 1e-5
                return gui.wait(lambda frame: any(row[0] == 7 and row[9] and
                    all(abs(a-b) < tolerance for a, b in zip(row[4:7], expected))
                    for row in target_records(frame)))

            wait(lambda value: value.get('ready'))
            gui = Gui(suffix)
            send('rdr live on')
            wait(lambda value: value.get('rdr_live') is True)
            live(lambda value: value['connected'])
            assert board.starts == 0 and not board.reporting
            assert set(board.requests) <= {0xED01, 0xED02}, 'monitor sent a sensor control'

            send('rdr report on')
            live(lambda value: value['reporting'])
            send('rdr start')
            live(xyz((2, 3, 1)))
            gui_xyz((2, 3, 1))
            board.observations = [(7, (-5, 2, 8), 0, 1800)]
            live(xyz((5, 8, 2)))
            if args.filtered:
                gui.wait(lambda frame: any(row[0] == 7 and row[9] and
                    2 < row[4] < 5 and 3 < row[5] < 8 and 1 < row[6] < 2
                    for row in target_records(frame)))
            gui_xyz((5, 8, 2))
            # A new observation pose must be used immediately for the same ID.
            board.observations = [(7, (-5, 2, 8), 0, 2700)]
            live(xyz((8, -5, 2)))
            gui_xyz((8, -5, 2))
            board.track_state = 0
            live(lambda value: bool(value['targets']) and all(
                not row['world_valid'] and row['world_xyz'] is None for row in value['targets']))
            gui.wait(lambda frame: valid_count(frame) == 0)
            send('rdr live off')
            wait(lambda value: value.get('rdr_live') is False)
            try:
                unexpected = output.get(timeout=.5)
            except queue.Empty:
                unexpected = None
            assert unexpected is None, ('output continued after monitor stopped', unexpected)
            assert board.reporting and board.mode == 2, 'monitor off stopped the sensor'
            # GUI live updates require no console monitoring command.
            board.observations = [(7, (-1, 4, 6), 0, 1800)]
            board.track_state = 1
            gui_xyz((1, 6, 4))
            board.observations = [(7, (-3, 2, 4), 0, 1800)]
            gui_xyz((3, 4, 2))
            board.track_state = 2
            board.observations = [(7, (-6, 3, 7), 0, 1800)]
            if args.filtered:
                # Predictions expire after 1.5 s at the sender. Verify a
                # filtered intermediate position, not convergence after expiry.
                gui.wait(lambda frame: any(row[0] == 7 and row[9] and
                    3.01 < row[4] < 5.99 and 4.01 < row[5] < 6.99 and 2.01 < row[6] < 2.99
                    for row in target_records(frame)), timeout=1.3)
            else:
                gui_xyz((6, 7, 3))
            # Restoring real observations retains the display history and
            # allows convergence without extending the prediction lifetime.
            board.track_state = 1
            gui_xyz((6, 7, 3))
            board.track_state = 0
            gui.wait(lambda frame: valid_count(frame) == 0)
            board.track_state = 1
            gui_xyz((6, 7, 3))
            board.observations = [(7, (-6, 3, 7), 0, -32768)]
            gui.wait(lambda frame: valid_count(frame) == 0)
            gui.close()
            gui = None
            send('quit')
            process.wait(timeout=5)
            reader.join(timeout=1)
            assert process.returncode == 0, process.stderr.read()
            assert board.error is None, board.error
            metadata = json.loads(Path(str(recording) + '.json').read_text())
            assert metadata['status'] == 'completed', metadata
            assert metadata['filter']['adaptive_motion'] == args.adaptive
            assert metadata['filter']['model'] == 'position_random_walk'
            if args.axes:
                for parameter in ('measurement_stddev_xyz_m', 'process_stddev_xyz_m_sqrt_s',
                                  'motion_process_stddev_xyz_m_sqrt_s'):
                    assert metadata['filter'][parameter] == config['rdr']['gui_kalman'][parameter]
            with recording.open(newline='') as source:
                rows = list(csv.DictReader(source))
            assert len(rows) == metadata['rows'] and len(rows) >= 3
            assert all(0 <= float(row['elapsed_s']) < 2 for row in rows)
            keys = [(row['generation'], row['target_id'], row['received_s'],
                     row['track_state'], row['valid']) for row in rows]
            assert len(keys) == len(set(keys)), 'redraws were counted as new samples'
            valid = [row for row in rows if row['valid'] == '1']
            assert valid and all(abs(float(row['before_x']) + float(row['sensor_x'])) < 1e-5
                                 for row in valid if row['observation_pan'] == '1800')
            changed = [row for row in valid if float(row['before_x']) == 5]
            assert changed, 'new received coordinates were not recorded'
            if args.filtered:
                assert any(2 < float(row['after_x']) < 5 for row in changed)
            else:
                assert all(abs(float(row['before_x']) - float(row['after_x'])) < 1e-5 for row in valid)
            print(f'PASS: GUI world XYZ (Kalman={args.filtered}, adaptive={args.adaptive}, independent_axes={args.axes}), observed/predicted smoothing, transitions, loss/unknown pose; loopback lidar only')
            print('PASS: paired coordinate recording, receive deduplication, automatic duration, metadata')
    finally:
        if gui:
            gui.close()
        if process and process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        board.close()


if __name__ == '__main__':
    main()
