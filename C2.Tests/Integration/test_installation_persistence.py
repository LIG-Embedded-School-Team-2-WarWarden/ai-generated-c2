"""Verify GUI position storage and restart recovery using loopback only."""
import argparse
from contextlib import contextmanager
import ctypes
import json
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import tempfile
import time
import uuid

from test_station import Gui

ROOT = Path(__file__).resolve().parents[2]


def positions(frame):
    return (struct.unpack_from('<3f', frame, 56),
            struct.unpack_from('<3f', frame, 100))


@contextmanager
def running(executable, runtime, config=None, cwd=None):
    stop = runtime / 'stop'
    stop.unlink(missing_ok=True)
    suffix = '_positions_' + uuid.uuid4().hex
    arguments = [str(executable), '--headless', '--stop-file', str(stop),
                 '--ipc-suffix', suffix, '--event-log', str(runtime / 'events.log')]
    if config:
        arguments += ['--config', str(config)]
    else:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as port_probe:
            port_probe.bind(('127.0.0.1', 0))
            local_port = port_probe.getsockname()[1]
        arguments += ['--bind', '127.0.0.1', '--rdr-local-ip', '127.0.0.1',
                      '--rdr-ip', '127.0.0.1', '--rdr-port', '59999', '--rdr-local-port', str(local_port),
                      '--fcs-ip', '127.0.0.1', '--fcs-port', '59998', '--fcs-local-port', '0']
    gui = None
    with (runtime / 'console.log').open('w+b') as output:
        process = subprocess.Popen(arguments, cwd=cwd or runtime, stdout=output,
                                   stderr=subprocess.STDOUT, creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                output.seek(0)
                log = output.read()
                if b'"ready":true' in log:
                    break
                assert process.poll() is None, log.decode(errors='replace')
                time.sleep(.05)
            else:
                raise AssertionError('ECS startup timed out')
            gui = Gui(suffix)
            yield gui
        finally:
            if gui:
                gui.close()
            stop.touch()
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            assert process.returncode == 0, (runtime / 'console.log').read_text(errors='replace')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--console', type=Path, default=ROOT / 'x64/Release/C2.Console.exe')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='c2-position-storage-', dir=ROOT) as directory:
        runtime = Path(directory)
        config = json.loads((ROOT / 'configs/team-demo.json').read_text(encoding='utf-8'))
        for name in ('rdr', 'fcs'):
            config[name]['local'] = {'ip': '127.0.0.1', 'port': 0}
            config[name]['peer'] = {'ip': '127.0.0.1', 'port': 59999}
        config['fcs']['enabled'] = True
        config['unrelated'] = {'keep': 'unchanged'}
        path = runtime / 'config.json'
        path.write_text(json.dumps(config), encoding='utf-8')
        executable = args.console.resolve()
        expected = ((1.25, -2.5, 3.75), (-4.5, 5.25, -6.0))
        with running(executable, runtime, path) as gui:
            gui.wait(lambda f: positions(f) == ((0, 0, 0), (0, 0, 0)))
            assert gui.command(5, rdr=1, fcs=0, xyz=expected[0]) == 0
            assert gui.command(5, rdr=0, fcs=1, xyz=expected[1]) == 0
            gui.wait(lambda f: positions(f) == expected)
            stored = json.loads(path.read_text())
            assert {k: v for k, v in stored.items() if k != 'installation'} == config
            settings = stored['installation']
            assert settings['devices'][settings['observer']]['position'] == [-2.5, -1.25, 3.75]
            assert settings['devices'][settings['effector']]['position'] == [5.25, 4.5, -6.0]
            before = path.read_bytes()
            assert gui.command(5, rdr=1, fcs=0, xyz=(float('nan'), 0, 0)) == 1
            assert path.read_bytes() == before

            # Allow reading but prevent replacement of the destination file.
            kernel = ctypes.WinDLL('kernel32', use_last_error=True)
            kernel.CreateFileW.argtypes = [ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_uint32,
                                          ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p]
            kernel.CreateFileW.restype = ctypes.c_void_p
            kernel.CloseHandle.argtypes = [ctypes.c_void_p]
            handle = kernel.CreateFileW(str(path), 0x80000000, 1, None, 3, 0, None)
            assert handle != ctypes.c_void_p(-1).value
            try:
                assert gui.command(5, rdr=1, fcs=0, xyz=(10, 20, 30)) == 6
                assert positions(gui.frame()) == expected
                assert path.read_bytes() == before
            finally:
                kernel.CloseHandle(handle)
            assert not list(runtime.glob('*.tmp.*'))

        # A new process with a different working directory restores both assets.
        other_cwd = runtime / 'other'
        other_cwd.mkdir()
        with running(executable, runtime, path, other_cwd) as gui:
            gui.wait(lambda f: positions(f) == expected)
            stored = json.loads(path.read_text())
            stored['installation']['note'] = 'preserve metadata'
            stored['installation']['devices']['unused'] = {'role': 'OTHER', 'position': [9, 8, 7]}
            settings = stored['installation']
            settings['devices'][settings['observer']]['position'] = [2, -1, 3]
            path.write_text(json.dumps(stored), encoding='utf-8')
            gui.wait(lambda f: positions(f)[0] == (1, 2, 3))
            assert gui.command(5, rdr=1, fcs=0, xyz=(1, 2, 3)) == 0
            after = json.loads(path.read_text())['installation']
            assert after['note'] == 'preserve metadata'
            assert after['devices']['unused'] == stored['installation']['devices']['unused']

        # No --config: the executable directory owns the persistent file.
        copied = runtime / 'C2.Console.exe'
        shutil.copy2(executable, copied)
        with running(copied, runtime) as gui:
            assert gui.command(5, rdr=0, fcs=1, xyz=expected[1]) == 0
        assert (runtime / 'installation.json').exists()
        with running(copied, runtime, cwd=other_cwd) as gui:
            gui.wait(lambda f: positions(f)[1] == expected[1])
    print('PASS: GUI position save, restart, failed save rollback, metadata, reload and default path; loopback only')


if __name__ == '__main__':
    main()
