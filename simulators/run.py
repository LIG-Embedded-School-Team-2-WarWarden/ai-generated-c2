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


def station_config(rdr_port, fcs_port, real_fcs_config=None):
    source=real_fcs_config or ROOT/'configs/team-demo.json'
    config=json.loads(Path(source).read_text(encoding='utf-8-sig'))
    config['rdr']['local']={'ip':'127.0.0.1','port':0}
    config['rdr']['peer']={'ip':'127.0.0.1','port':rdr_port}
    if real_fcs_config:
        if config['fcs'].get('protocol')!='effe' or not config['fcs'].get('enabled',True):
            raise ValueError('Real FCS configuration must enable the effe protocol')
        # Preserve the real FCS local IP/port, peer and other configuration.
    else:
        config['fcs']['local']={'ip':'127.0.0.1','port':0}
        config['fcs']['peer']={'ip':'127.0.0.1','port':fcs_port}
    return config


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
    parser.add_argument('--runtime', type=Path)
    parser.add_argument('--real-fcs',action='store_true',help='dummy lidar with physical FCS; only RDR is simulated')
    parser.add_argument('--fcs-config',type=Path,default=ROOT/'configs/rdr-live.json',help='source configuration for the physical FCS')
    parser.add_argument('--print-plan',action='store_true',help='show endpoints without opening sockets or starting C2')
    parser.add_argument('--duration', type=float, default=0, help='optional unattended run limit, seconds')
    parser.add_argument('--target', nargs=4, action='append', metavar=('ID','X','Y','Z'),
                        help='fixed target ID and world XYZ in metres; repeat for multiple targets')
    args = parser.parse_args()
    args.runtime=args.runtime or ROOT/('logs/rdr-dummy-real-fcs' if args.real_fcs else 'logs/simulator')
    try:
        planned_config=station_config(0,0,args.fcs_config if args.real_fcs else None)
    except (OSError,ValueError,KeyError) as error:
        parser.error(str(error))
    if args.print_plan:
        print(json.dumps({'mode':'dummy-rdr/real-fcs' if args.real_fcs else 'dummy-rdr/dummy-fcs',
            'rdr':'127.0.0.1:auto','fcs':planned_config['fcs'],'targets':args.target},ensure_ascii=False,indent=2))
        return
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
    if args.target:
        try:
            control['targets']=[{'id':int(t[0]),'xyz':[float(v) for v in t[1:]]} for t in args.target]
        except ValueError:
            parser.error('--target requires integer ID and numeric world X Y Z')
    write_json(control_file, control)
    devices = Devices(enable_fcs=not args.real_fcs)
    try:
        devices.configure(**control)
    except (ValueError,TypeError,KeyError) as error:
        devices.close()
        parser.error(str(error))
    config=planned_config
    config['rdr']['peer']['port']=devices.rdr_port
    if not args.real_fcs:
        config['fcs']['peer']['port']=devices.fcs_port
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
    fcs_view = None
    logs = []
    ipc_suffix = '_sim_' + uuid.uuid4().hex[:10] if args.isolated_ipc else ''
    try:
        for name in ('c2.stdout.log', 'c2.stderr.log'):
            logs.append((args.runtime / name).open('w', encoding='utf-8'))
        station_started_ms=int(time.time()*1000)
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
        fcs_peer=config['fcs']['peer']
        print(f'RDR 더미: 127.0.0.1:{devices.rdr_port} / FCS {"실장비" if args.real_fcs else "모의기"}: {fcs_peer["ip"]}:{fcs_peer["port"]}', flush=True)
        print(f'상태/시나리오: {args.runtime.resolve()}', flush=True)
        print('WPF의 라이다 구동 버튼으로 시작합니다. 종료: 제어창 닫기 또는 Ctrl+C.', flush=True)
        if not args.headless:
            import tkinter as tk
            from tkinter import ttk
            panel = tk.Tk()
            title='라이다 더미 / FCS 실장비' if args.real_fcs else 'RDR / FCS 모의기'
            panel.title(title)
            panel.geometry('440x590' if args.real_fcs else '920x590')
            controls = ttk.Frame(panel, width=440)
            controls.pack(side='left', fill='y')
            ttk.Label(controls, text=title, font=('맑은 고딕', 17, 'bold')).pack(pady=12)
            description=(f'라이다만 더미입니다. FCS: {fcs_peer["ip"]}:{fcs_peer["port"]}\nGUI에서 표적 할당을 누르면 실장비가 이동합니다.'
                if args.real_fcs else '로컬 UDP → C2 → WPF\n실제 장비 출력 없이 화면과 통신을 확인합니다.')
            ttk.Label(controls, text=description, justify='center').pack()
            status = tk.StringVar()
            ttk.Label(controls, textvariable=status, justify='left').pack(pady=12)
            def choose(scenario):
                control['scenario'] = scenario
                write_json(control_file, control)
            scenarios=[('정상 · 지정 표적' if args.target else '정상 · 표적 10개','normal'),('표적 소실','empty'),('RDR 연결 끊김','rdr-offline')]
            if not args.real_fcs:scenarios += [('FCS 연결 끊김','fcs-offline'),('FCS 결함','fcs-fault')]
            for label, scenario in scenarios:
                ttk.Button(controls, text=label, command=lambda value=scenario: choose(value)).pack(fill='x', padx=35, pady=3)
            ttk.Label(controls,text='표적 월드 위치 (m): X 오른쪽 / Y 앞쪽 / Z 위쪽').pack(pady=(12,4))
            editor=ttk.Frame(controls);editor.pack()
            initial=control.get('targets') or [{'id':1001,'xyz':[-1.4,2.8,.35]}]
            ident=tk.StringVar(value=str(initial[0]['id']))
            selector=ttk.Combobox(editor,textvariable=ident,width=7,state='readonly',
                values=[str(t['id']) for t in control.get('targets',[]) ] or [str(1001+i) for i in range(10)])
            selector.grid(row=1,column=0,padx=3)
            entries=[]
            for column,label in enumerate(('ID','X','Y','Z')):
                ttk.Label(editor,text=label).grid(row=0,column=column)
            for column,value in enumerate(initial[0]['xyz'],1):
                variable=tk.StringVar(value=str(value));entries.append(variable)
                ttk.Entry(editor,textvariable=variable,width=9).grid(row=1,column=column,padx=3)
            def select_target(_event):
                scene=control.get('targets') or [
                    {'id':1001+i,'xyz':[-1.4+.3*i,2.8+.5*i,.35+.065*i]} for i in range(10)]
                selected=next(t for t in scene if t['id']==int(ident.get()))
                for variable,value in zip(entries,selected['xyz']):variable.set(str(value))
            selector.bind('<<ComboboxSelected>>',select_target)
            edit_status=tk.StringVar()
            def apply_target():
                try:
                    scene=control.get('targets') or [
                        {'id':1001+i,'xyz':[-1.4+.3*i,2.8+.5*i,.35+.065*i]} for i in range(10)]
                    xyz=[float(v.get()) for v in entries]
                    updated=[{'id':t['id'],'xyz':xyz if t['id']==int(ident.get()) else t['xyz']} for t in scene]
                    # Validate before replacing the running scenario.
                    devices.configure(control['scenario'],control['target_count'],updated,control.get('rdr_world',(0,0,0)))
                    control['targets']=updated;write_json(control_file,control)
                    edit_status.set('위치 적용: 소실 후 새 위치로 재검출합니다.' if args.real_fcs
                        else '위치 적용: 같은 ID로 새 좌표를 전송합니다.')
                except (ValueError,TypeError,KeyError) as error:
                    edit_status.set(str(error))
            ttk.Button(controls,text='표적 위치 적용',command=apply_target).pack(pady=6)
            ttk.Label(controls,textvariable=edit_status,wraplength=390).pack()
            ttk.Button(controls, text='전체 종료', command=stop).pack(pady=10)
            if not args.real_fcs:
                from fcs_view import FcsView
                panel.minsize(760, 520)
                fcs_view = FcsView(panel)
                fcs_view.pack(side='right', fill='both', expand=True)
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
                        devices.configure(updated.get('scenario', 'normal'), updated.get('target_count', 10),
                            updated.get('targets'),updated.get('rdr_world',devices.rdr_world))
                        control, seen = updated, modified
                except (OSError, ValueError) as error:
                    print(f'시나리오 설정 대기: {error}', flush=True)
                if control.get('targets') is not None:
                    try:
                        telemetry=json.loads((args.runtime/'c2-state.json').read_text(encoding='utf-8'))
                        if telemetry.get('timestamp_ms',0)<station_started_ms:
                            raise ValueError('Telemetry belongs to an earlier ECS run')
                        # ECS installation storage is (forward,left,up), whereas
                        # dummy world inputs and GUI positions are (east,north,up).
                        forward,left,up=telemetry['display_positions']['rdr']
                        origin=(-left,forward,up)
                        if tuple(devices.rdr_world)!=origin:
                            devices.configure(control.get('scenario','normal'),control.get('target_count',10),
                                control['targets'],origin)
                            control['rdr_world']=list(origin)
                            write_json(control_file,control)
                            print(f'RDR world position synchronized from ECS: {origin}; reacquiring targets',flush=True)
                    except (OSError,ValueError,KeyError,TypeError):
                        pass  # A later telemetry snapshot supplies the installation position.
                state = devices.snapshot()
                state['runner_pid'] = os.getpid()
                state['c2_pid'] = station.pid
                state['ipc_suffix'] = ipc_suffix
                state['fcs_mode']='real' if args.real_fcs else 'dummy'
                state['fcs_peer']=fcs_peer
                write_json(state_file, state)
                if panel:
                    fcs_status=('FCS 실측 각도·정렬 상태는 GUI에서 확인합니다.' if args.real_fcs else
                        f'FCS 고각={state["fcs_elevation_deg"]:.1f}° / 방위각={state["fcs_azimuth_deg"]:.1f}°')
                    status.set(f'시나리오: {state["scenario"]}\nRDR 연결={state["rdr_connected"]} / 모드={state["rdr_mode"]}\n'+fcs_status)
            if panel:
                if fcs_view:
                    fcs_view.update_pose(devices.snapshot())
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
