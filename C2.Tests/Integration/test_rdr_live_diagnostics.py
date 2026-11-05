"""Read-only live diagnostics against a loopback lidar; no FCS or hardware."""
import argparse
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
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    board = Board()
    board.observations = [(7, (-2, 1, 3), 0, 1800)]
    process = gui = None
    try:
        with tempfile.TemporaryDirectory() as directory:
            config = {
                'rdr': {'protocol': 'edde24', 'local': {'ip': '127.0.0.1', 'port': 0},
                        'peer': {'ip': '127.0.0.1', 'port': board.port}},
                'fcs': {'protocol': 'effe', 'enabled': False,
                        'local': {'ip': '127.0.0.1', 'port': 0},
                        'peer': {'ip': '127.0.0.1', 'port': 1}},
            }
            path = Path(directory) / 'config.json'
            path.write_text(json.dumps(config), encoding='utf-8')
            suffix = '_live_test_' + uuid.uuid4().hex
            process = subprocess.Popen(
                [str(args.console.resolve()), '--config', str(path),
                 '--ipc-suffix', suffix],
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
                return gui.wait(lambda frame: any(row[0] == 7 and row[9] and
                    all(abs(a-b) < 1e-5 for a, b in zip(row[4:7], expected))
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
            gui_xyz((6, 7, 3))
            board.track_state = 1
            board.observations = [(7, (-6, 3, 7), 0, -32768)]
            gui.wait(lambda frame: valid_count(frame) == 0)
            gui.close()
            gui = None
            send('quit')
            process.wait(timeout=5)
            reader.join(timeout=1)
            assert process.returncode == 0, process.stderr.read()
            assert board.error is None, board.error
            print('PASS: GUI latest world XYZ with monitor on/off, same-ID observed/predicted updates, pose, loss/unknown pose; loopback lidar only')
    finally:
        if gui:
            gui.close()
        if process and process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        board.close()


if __name__ == '__main__':
    main()
