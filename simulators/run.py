"""Run device simulators, the real C2, and optionally the WPF GUI together."""
import argparse
import json
import msvcrt
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
from devices import Devices

ROOT = Path(__file__).resolve().parents[1]


def write_json(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    # Windows readers can briefly hold a destination without FILE_SHARE_DELETE.
    # Retry the atomic replacement rather than terminating a running simulator.
    for attempt in range(20):
        try:
            temporary.replace(path)
            return
        except PermissionError:
            if attempt == 19:
                raise
            time.sleep(.01)


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description='RDR/FCS 모의기 — loopback UDP + C2 + WPF')
    parser.add_argument('--console', type=Path, default=ROOT / 'x64/Release/C2.Console.exe')
    parser.add_argument('--gui', type=Path, help='WPF executable to launch')
    parser.add_argument('--headless', action='store_true', help='omit simulator control window')
    parser.add_argument('--isolated-ipc', action='store_true', help='use unique IPC names for automated checks without WPF')
    parser.add_argument('--runtime', type=Path, default=ROOT / 'logs/simulator')
    parser.add_argument('--duration', type=float, default=0, help='optional unattended run limit, seconds')
    args = parser.parse_args()
    if args.isolated_ipc and args.gui:
        parser.error('--isolated-ipc cannot be used with the standard WPF GUI')
    if not args.console.is_file() or (args.gui and not args.gui.is_file()):
        parser.error('Build C2/WPF first, or use scripts/run_simulator.ps1 -Gui')
    args.runtime.mkdir(parents=True, exist_ok=True)
    instance_lock = (args.runtime / 'instance.lock').open('a+b')
    if instance_lock.tell() == 0:
        instance_lock.write(b'\0')
        instance_lock.flush()
    instance_lock.seek(0)
    try:
        msvcrt.locking(instance_lock.fileno(), msvcrt.LK_NBLCK, 1)
    except OSError:
        instance_lock.close()
        parser.error('This runtime is already in use. Close its simulator or use --runtime with another directory.')
    stop_file = args.runtime / 'stop.txt'
    if stop_file.exists():
        stop_file.unlink()
    control_file = args.runtime / 'control.json'
    state_file = args.runtime / 'simulator-state.json'
    control = {'scenario': 'normal', 'target_count': 10}
    write_json(control_file, control)
    devices = Devices()
    config = json.loads((ROOT / 'configs/team-demo.json').read_text(encoding='utf-8'))
    config['rdr']['peer']['port'] = devices.rdr_port
    config['fcs']['peer']['port'] = devices.fcs_port
    for section in ('rdr', 'fcs'):
        config[section]['local'] = {'ip': '127.0.0.1', 'port': 0}
        config[section]['peer']['ip'] = '127.0.0.1'
    config_path = args.runtime / 'config.json'
    write_json(config_path, config)
    exiting = False
    def stop(*_):
        nonlocal exiting
        exiting = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    children = []
    panel = None
    logs = []
    ipc_suffix = '_sim_' + uuid.uuid4().hex[:10] if args.isolated_ipc else ''
    try:
        for name in ('c2.stdout.log', 'c2.stderr.log'):
            logs.append((args.runtime / name).open('w', encoding='utf-8'))
        station = subprocess.Popen([str(args.console.resolve()), '--config', str(config_path.resolve()),
            '--headless', '--stop-file', str(stop_file.resolve()),
            '--ipc-suffix', ipc_suffix,
            '--status-file', str((args.runtime / 'c2-state.json').resolve())],
            cwd=ROOT, stdout=logs[0], stderr=logs[1], creationflags=subprocess.CREATE_NO_WINDOW)
        children.append(station)
        if args.gui:
            # WPF is explicitly requested as a visible interactive window.
            gui_env = dict(os.environ,
                AADC9_SHARED_MEMORY_NAME='Local\\ControlStationMock.SharedMemory.v3' + ipc_suffix,
                AADC9_SHARED_MEMORY_MUTEX_NAME='Local\\ControlStationMock.SharedMemory.Mutex.v3' + ipc_suffix)
            # Also use the default pipe when launching a GUI that supports overrides.
            gui_env.pop('AADC9_PIPE_NAME', None)
            children.append(subprocess.Popen([str(args.gui.resolve())], cwd=args.gui.resolve().parent, env=gui_env))
        print(f'RDR/FCS 모의기: 127.0.0.1:{devices.rdr_port} / 127.0.0.1:{devices.fcs_port}', flush=True)
        print(f'상태/시나리오: {args.runtime.resolve()}', flush=True)
        print('WPF의 라이다 구동 버튼으로 시작합니다. 종료: 제어창 닫기 또는 Ctrl+C.', flush=True)
        if not args.headless:
            import tkinter as tk
            from tkinter import ttk
            panel = tk.Tk()
            panel.title('RDR / FCS 모의기')
            panel.geometry('420x410')
            ttk.Label(panel, text='RDR / FCS 모의기', font=('맑은 고딕', 17, 'bold')).pack(pady=12)
            ttk.Label(panel, text='로컬 UDP → C2 → WPF\n실제 장비 출력 없이 화면과 통신을 확인합니다.', justify='center').pack()
            status = tk.StringVar()
            ttk.Label(panel, textvariable=status, justify='left').pack(pady=12)
            def choose(scenario):
                control['scenario'] = scenario
                write_json(control_file, control)
            for label, scenario in [('정상 · 표적 10개', 'normal'), ('표적 소실', 'empty'),
                ('RDR 연결 끊김', 'rdr-offline'), ('FCS 연결 끊김', 'fcs-offline'), ('FCS 결함', 'fcs-fault')]:
                ttk.Button(panel, text=label, command=lambda value=scenario: choose(value)).pack(fill='x', padx=35, pady=3)
            ttk.Button(panel, text='전체 종료', command=stop).pack(pady=10)
            panel.protocol('WM_DELETE_WINDOW', stop)
        started, last_poll, seen = time.monotonic(), 0, None
        while not exiting and not stop_file.exists():
            if devices.error:
                raise RuntimeError(f'device simulator failed: {devices.error}')
            if station.poll() is not None:
                raise RuntimeError(f'C2 exited ({station.returncode}); see {args.runtime / "c2.stderr.log"}')
            if args.duration and time.monotonic() - started > args.duration:
                break
            if args.gui and children[-1].poll() is not None:
                if children[-1].returncode:
                    raise RuntimeError(f'WPF exited with error {children[-1].returncode}')
                break
            if time.monotonic() - last_poll > .2:
                last_poll = time.monotonic()
                try:
                    modified = control_file.stat().st_mtime_ns
                    if modified != seen:
                        updated = json.loads(control_file.read_text(encoding='utf-8-sig'))
                        devices.configure(updated.get('scenario', 'normal'), updated.get('target_count', 10))
                        control, seen = updated, modified
                except (OSError, ValueError) as error:
                    print(f'시나리오 설정 대기: {error}', flush=True)
                state = devices.snapshot()
                state['runner_pid'] = os.getpid()
                state['c2_pid'] = station.pid
                state['ipc_suffix'] = ipc_suffix
                write_json(state_file, state)
                if panel:
                    status.set(f'시나리오: {state["scenario"]}\nRDR 연결={state["rdr_connected"]} / 모드={state["rdr_mode"]}\n'
                        f'FCS 고각={state["fcs_elevation_deg"]:.1f}° / 방위각={state["fcs_azimuth_deg"]:.1f}°')
            if panel:
                panel.update()
            time.sleep(.02)
    finally:
        stop_file.touch()
        if children:
            try:
                children[0].wait(timeout=5)  # Give C2 its ordinary stop/cleanup path.
            except subprocess.TimeoutExpired:
                children[0].terminate()
                children[0].wait(timeout=3)
        for child in children[1:]:
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=3)
        devices.close()
        for stream in logs:
            stream.close()
        if panel:
            panel.destroy()
        instance_lock.close()


if __name__ == '__main__':
    main()
