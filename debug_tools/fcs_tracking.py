"""Display tracking error and move one object in a running local simulator."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import tkinter as tk
from tkinter import ttk

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'simulators'))
from fcs_view import FcsView


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def object_position(state, target_id):
    return next((tuple(target['xyz']) for target in (state.get('targets') or [])
                 if target['id'] == target_id), None)


def simulated_direction(position, installation):
    """Compute a display reference from the simulated scene and mount origin."""
    east, north, up = position
    forward, left, height = installation
    x, y, z = east + left, north - forward, up - height
    horizontal = math.hypot(x, y)
    if horizontal == 0:
        return None
    return ((180 + math.degrees(math.atan2(x, y))) % 360,
            math.degrees(math.atan2(z, horizontal)))


def angle_error(reference, actual):
    return (reference[0] - actual[0] + 180) % 360 - 180, reference[1] - actual[1]


class StraightMotion:
    def __init__(self, start, end, speed, now):
        self.start, self.end, self.started = start, end, now
        self.duration = math.dist(start, end) / speed

    def at(self, now):
        progress = min(1, max(0, (now - self.started) / self.duration))
        return [round(a + (b - a) * progress, 4) for a, b in zip(self.start, self.end)], progress == 1


class TrackingWindow:
    def __init__(self, root, runtime, target_id, speed):
        self.root, self.runtime, self.target_id, self.speed = root, runtime, target_id, speed
        self.state_path = runtime / 'simulator-state.json'
        self.telemetry_path = runtime / 'c2-state.json'
        self.control_path = runtime / 'control.json'
        self.pending = runtime / f'fcs-tracking-{os.getpid()}.tmp'
        config = read_json(runtime / 'config.json')
        if any(config[device][side]['ip'] != '127.0.0.1'
               for device in ('rdr', 'fcs') for side in ('local', 'peer')):
            raise ValueError('로컬 RDR/FCS 모의 세션에서 실행하세요.')
        state = read_json(self.state_path)
        self.session = (state['runner_pid'], state['c2_pid'])
        self.motion = None
        self.last_write = 0
        self.last_position = None
        self.peak = [0.0, 0.0]
        root.title(f'FCS 추적 오차 · ID {target_id}')
        root.geometry('650x670')
        root.minsize(540, 490)
        self.view = FcsView(root)
        self.view.pack(fill='both', expand=True)
        details = ttk.LabelFrame(root, text='객체 방향과 모의 모터의 각도 차이', padding=(12, 8))
        details.pack(fill='x', padx=16, pady=(0, 10))
        self.readout = ttk.Label(details, justify='left', font=('Consolas', 11))
        self.readout.pack(anchor='w')
        self.message = ttk.Label(root, padding=(16, 0))
        self.message.pack(anchor='w')
        controls = ttk.Frame(root, padding=(16, 8))
        controls.pack(fill='x')
        self.start_button = ttk.Button(controls, text=f'직선 이동 · {speed:g} m/s', command=self.start)
        self.start_button.pack(side='left')
        self.stop_button = ttk.Button(controls, text='이동 정지', command=self.stop)
        self.stop_button.pack(side='left', padx=8)
        self.stop_button.state(['disabled'])
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.tick()

    def live_state(self):
        state = read_json(self.state_path)
        if (state['fcs_mode'] != 'dummy' or not state['mock_fcs_enabled']
                or (state['runner_pid'], state['c2_pid']) != self.session
                or time.time() - self.state_path.stat().st_mtime > 3):
            raise ValueError('처음 연결한 로컬 모의 세션이 종료되었습니다.')
        return state

    def start(self):
        try:
            state = self.live_state()
            if not state['reporting']:
                raise ValueError('WPF에서 라이다 구동을 먼저 눌러 주세요.')
            current = object_position(read_json(self.control_path), self.target_id)
            if current is None:
                raise ValueError(f'모의기 설정에 ID {self.target_id}의 위치가 없습니다.')
            end_x, end_z = (1.1, .95) if current[0] <= 0 else (-1.1, .35)
            self.motion = StraightMotion(current, (end_x, current[1], end_z), self.speed, time.monotonic())
            self.last_position = current
            self.last_write = 0
            self.peak = [0.0, 0.0]
            self.start_button.state(['disabled'])
            self.stop_button.state(['!disabled'])
            self.message.configure(text=f'ID {self.target_id} 직선 이동 중')
            print(f'ID {self.target_id} start={current} end={self.motion.end} '
                  f'speed={self.speed:g} m/s', flush=True)
        except (OSError, ValueError, KeyError) as error:
            self.message.configure(text=str(error))

    def stop(self, message='이동 정지'):
        self.motion = None
        self.start_button.state(['!disabled'])
        self.stop_button.state(['disabled'])
        self.message.configure(text=message)

    def move(self, now):
        control = read_json(self.control_path)
        current = object_position(control, self.target_id)
        if current != self.last_position:
            self.stop('모의기에서 객체 위치를 변경해 자동 이동을 중지했습니다.')
            return
        position, arrived = self.motion.at(now)
        for target in control['targets']:
            if target['id'] == self.target_id:
                target['xyz'] = position
                break
        self.pending.write_text(json.dumps(control), encoding='utf-8')
        try:
            self.pending.replace(self.control_path)
        except PermissionError:
            return  # Try on the next frame if the runner is reading the file.
        self.last_position = tuple(position)
        self.last_write = now
        if arrived:
            print(f'ID {self.target_id} reached {position}; '
                  f'peak error pan={self.peak[0]:.1f}°, tilt={self.peak[1]:.1f}°', flush=True)
            self.stop(f'ID {self.target_id} 직선 이동 완료')

    def tick(self):
        try:
            state = self.live_state()
            self.view.update_pose(state)
            position = object_position(state, self.target_id)
            if position is None or not state['reporting']:
                if self.motion:
                    self.stop('RDR 보고가 중지되어 객체 이동을 멈췄습니다.')
                self.readout.configure(text=f'ID {self.target_id}의 모의 좌표를 기다리는 중')
            else:
                telemetry = read_json(self.telemetry_path)
                if time.time() - self.telemetry_path.stat().st_mtime > 5:
                    raise ValueError('ECS의 설치 위치 정보를 기다리는 중')
                reference = simulated_direction(position, telemetry['display_positions']['eff'])
                if reference:
                    actual = state['fcs_azimuth_deg'], state['fcs_elevation_deg']
                    error = angle_error(reference, actual)
                    self.readout.configure(text=(
                        '        객체 방향    현재각     차이\n'
                        f'PAN     {reference[0]:7.1f}°   {actual[0]:7.1f}°  {error[0]:+6.1f}°\n'
                        f'TILT    {reference[1]:7.1f}°   {actual[1]:7.1f}°  {error[1]:+6.1f}°'
                    ))
                    if self.motion:
                        self.peak = [max(peak, abs(value)) for peak, value in zip(self.peak, error)]
                else:
                    self.readout.configure(text='수평 방향을 계산할 수 없는 위치입니다.')
                now = time.monotonic()
                if self.motion and now - self.last_write >= .2:
                    self.move(now)
        except (PermissionError, json.JSONDecodeError):
            pass  # The simulator may be replacing one of its snapshots.
        except (OSError, ValueError, KeyError) as error:
            self.stop(str(error))
            self.readout.configure(text='각도 데이터 대기 중')
        self.root.after(50, self.tick)

    def close(self):
        self.pending.unlink(missing_ok=True)
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path, default=ROOT / 'logs/fcs-3d')
    parser.add_argument('--target', type=int, default=7)
    parser.add_argument('--speed', type=float, default=.18, help='straight-line speed in m/s')
    parser.add_argument('--start', action='store_true', help='move after opening the window')
    args = parser.parse_args()
    if not math.isfinite(args.speed) or args.speed <= 0:
        parser.error('--speed must be finite and positive')
    root = tk.Tk()
    try:
        window = TrackingWindow(root, args.runtime, args.target, args.speed)
    except (OSError, ValueError, KeyError) as error:
        root.destroy()
        parser.error(str(error))
    if args.start:
        root.after(2500, window.start)
    root.mainloop()


if __name__ == '__main__':
    main()
