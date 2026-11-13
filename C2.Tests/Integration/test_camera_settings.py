"""Camera settings persistence/validation over loopback IPC, with no hardware."""
import argparse
import ctypes
import json
from pathlib import Path
import struct
import tempfile
import time
from test_installation_persistence import running, positions, ROOT


def read(gui, group):
    assert gui.command(8, target=group, rdr=0, fcs=0) == 0
    return struct.unpack('<3f', struct.pack('<3I', *gui.last_response[5:8]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--console', type=Path, default=ROOT / 'x64/Release/C2.Console.exe')
    exe = parser.parse_args().console.resolve()
    with tempfile.TemporaryDirectory(prefix='camera-test-', dir=ROOT) as folder:
        runtime = Path(folder)
        config = json.loads((ROOT / 'configs/team-demo.json').read_text(encoding='utf-8'))
        for name in ('rdr', 'fcs'):
            config[name]['local'] = {'ip': '127.0.0.1', 'port': 0}
            config[name]['peer'] = {'ip': '127.0.0.1', 'port': 59999}
        config['fcs']['enabled'] = True
        config['user_note'] = {'keep': [1, 2, 3]}
        path = runtime / 'config.json'
        path.write_text(json.dumps(config), encoding='utf-8')
        expected = [(1.25, -2.5, 3.75), (-4, 5, -6), (2, 12, 4), (90, 60, 0)]
        with running(exe, runtime, path) as gui:
            assert read(gui, 0) == (0, 0, 0)
            assert read(gui, 1) == (0, 0, 0)
            assert read(gui, 2) == (1.5, 10, 3)
            assert read(gui, 3) == (120, 45, 0)
            gui.wait(lambda f: struct.unpack_from('<f', f, 68)[0] == 120)
            assert gui.command(5, rdr=1, fcs=0, xyz=(10, 20, 30)) == 0
            gui.wait(lambda f: positions(f)[0] == (10, 20, 30))
            installation_before = positions(gui.frame())
            for group, values in enumerate(expected):
                assert gui.command(9, target=group, rdr=0, fcs=0, xyz=values) == 0
                assert read(gui, group) == values
            gui.wait(lambda f: struct.unpack_from('<f', f, 68)[0] == expected[3][0])
            assert positions(gui.frame()) == installation_before
            stored = json.loads(path.read_text())
            assert stored['installation']['camera']['rdr_horizontal_fov_deg'] == 90
            assert stored['installation']['camera']['rdr_vertical_fov_deg'] == 60
            assert stored['user_note'] == config['user_note']
            assert stored['installation']['devices']['RDR-1']['position'] == [20, -10, 30]
            before = path.read_bytes()
            for values in [(float('nan'), 10, 3), (1, float('inf'), 3), (-1, 10, 3), (2, 1, 3), (1, 10, -1)]:
                assert gui.command(9, target=2, rdr=0, fcs=0, xyz=values) == 1
                assert path.read_bytes() == before
                assert read(gui, 2) == expected[2]
            assert gui.command(9, target=4, rdr=0, fcs=0) == 1
            assert gui.command(9, target=0, rdr=1, fcs=0) == 1
            for values in [(0,45,0), (361,45,0), (120,181,0), (120,-1,0), (float('inf'),45,0), (120,45,1)]:
                assert gui.command(9, target=3, rdr=0, fcs=0, xyz=values) == 1
                assert read(gui, 3) == expected[3]
                assert path.read_bytes() == before
            kernel = ctypes.WinDLL('kernel32', use_last_error=True)
            kernel.CreateFileW.argtypes = [ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p]
            kernel.CreateFileW.restype = ctypes.c_void_p
            kernel.CloseHandle.argtypes = [ctypes.c_void_p]
            handle = kernel.CreateFileW(str(path), 0x80000000, 1, None, 3, 0, None)
            assert handle != ctypes.c_void_p(-1).value
            try:
                assert gui.command(9, target=2, rdr=0, fcs=0, xyz=(3, 15, 5)) == 6
                assert read(gui, 2) == expected[2]
                assert gui.command(9, target=3, rdr=0, fcs=0, xyz=(100, 70, 0)) == 6
                assert read(gui, 3) == expected[3]
                assert struct.unpack_from('<f', gui.frame(), 68)[0] == expected[3][0]
                assert path.read_bytes() == before
            finally:
                kernel.CloseHandle(handle)
            assert not list(runtime.glob('*.tmp.*'))
            # Reject nonnumeric persisted data during reload without losing current values.
            invalid = json.loads(before)
            invalid['installation']['camera']['ranges'] = ['bad', 10, 3]
            path.write_text(json.dumps(invalid))
            time.sleep(.8)
            assert read(gui, 2) == expected[2]
            path.write_bytes(before)
        with running(exe, runtime, path) as gui:
            for group, values in enumerate(expected):
                assert read(gui, group) == values
            gui.wait(lambda f: positions(f) == installation_before and struct.unpack_from('<f', f, 68)[0] == expected[3][0])
    print('PASS: defaults, separate positions, save/restore, finite/range validation, failed-save rollback, metadata, invalid reload')

if __name__ == '__main__':
    main()
