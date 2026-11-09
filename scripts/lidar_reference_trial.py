"""Mark a physical two-point trial on the ECS clock and compare published world XY."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time


def sequence():
    events = [(0, 'ready', 'A')]
    for lap in range(1, 4):
        events.extend([(lap, 'depart', 'A'), (lap, 'stop', 'B'),
                       (lap, 'depart', 'B'), (lap, 'stop', 'A')])
    return events + [(3, 'end', 'A')]


def mark(recording):
    import tkinter as tk
    from tkinter import messagebox, ttk

    metadata = Path(str(recording) + '.json')
    path = recording.with_suffix('.markers.csv')
    events = sequence()
    index, last_press = 0, 0.
    root = tk.Tk()
    root.title('라이다 기준점 측정')
    root.attributes('-topmost', True)
    body = ttk.Frame(root, padding=18)
    body.pack(fill='both', expand=True)
    label = ttk.Label(body, text='기록 상태 확인 중')
    label.pack(pady=6)
    button = ttk.Button(body, width=32)
    button.pack(ipady=7)
    ttk.Label(body, text=str(path), wraplength=480).pack(pady=6)
    titles = {'ready': 'A에서 측정 시작', 'depart': '출발', 'stop': '정지', 'end': '측정 종료'}

    def remaining():
        try:
            status = json.loads(metadata.read_text(encoding='utf-8'))
            if status['status'] == 'recording':
                return (status['started_unix_ms'] / 1000 + status['requested_duration_s']) - time.time()
        except (OSError, KeyError, json.JSONDecodeError):
            pass
        return 0.

    if path.exists():
        messagebox.showerror('측정 기록', f'이미 시각 기록이 있습니다. 새 측정 파일을 선택하세요.\n{path}')
        root.destroy()
        return
    with path.open('x', newline='', encoding='utf-8') as output:
        writer = csv.writer(output)
        writer.writerow(['lap', 'event', 'point', 'unix_ms', 'monotonic_ns'])
        output.flush()

        def wait_seconds():
            return max(0., 20 - (time.monotonic() - last_press)) if index and events[index][1] in ('depart', 'end') else 0.

        def press():
            nonlocal index, last_press
            if index >= len(events) or remaining() <= 0 or wait_seconds() > 0:
                return
            epoch_ms, mono_ns = time.time_ns() // 1_000_000, time.monotonic_ns()
            writer.writerow([*events[index], epoch_ms, mono_ns])
            output.flush()
            last_press = mono_ns / 1e9
            index += 1
            if index == len(events):
                button.configure(state='disabled', text='기록 완료')
                label.configure(text='세 번의 왕복을 기록했습니다. 창을 닫아도 됩니다.')
            else:
                display()

        def display():
            lap, action, point = events[index]
            left, wait = remaining(), wait_seconds()
            text = titles[action] if action in ('ready', 'end') else f'{point} {titles[action]}'
            button.configure(text=text, state='normal' if left > 0 and wait <= 0 else 'disabled')
            if left <= 0:
                label.configure(text='ECS 좌표 기록 대기 또는 종료')
            elif wait > 0:
                label.configure(text=f'{point}에 그대로 두세요. {math.ceil(wait)}초 남았습니다.')
            else:
                label.configure(text=f'{lap}/3 왕복 · 실제 동작 순간에 버튼을 누르세요.')

        def tick():
            if index < len(events):
                display()
                root.after(250, tick)

        button.configure(command=press)
        tick()
        root.mainloop()


def observed(row):
    return row['valid'] == '1' and row['track_state'] == '1'


def metrics(dwell, point, stop_ms, end_ms, prefix, tolerance_m, evaluate_settling):
    def xy(row):
        return (float(row[prefix + '_x']), float(row[prefix + '_y']))

    late = [xy(row) for row in dwell if observed(row) and int(row['capture_unix_ms']) >= end_ms - 10000]
    result = {'late_n': len(late)}
    if len(late) >= 3:
        errors = [(x - point[0], y - point[1]) for x, y in late]
        result.update(bias_x_cm=100 * statistics.mean(x for x, _ in errors),
                      bias_y_cm=100 * statistics.mean(y for _, y in errors),
                      rmse_x_cm=100 * math.sqrt(statistics.mean(x*x for x, _ in errors)),
                      rmse_y_cm=100 * math.sqrt(statistics.mean(y*y for _, y in errors)),
                      xy_rmse_cm=100 * math.sqrt(statistics.mean(x*x + y*y for x, y in errors)))
    if evaluate_settling:
        streak = []
        for row in dwell:
            if row['valid'] != '1':
                streak.clear()
                continue
            t = int(row['capture_unix_ms'])
            if (streak and t - streak[-1] > 3000):
                streak.clear()
            if math.dist(xy(row), point) > tolerance_m:
                streak.clear()
            elif observed(row):
                streak.append(t)
                if len(streak) == 3:
                    result.update(entry_s=(streak[0] - stop_ms) / 1000,
                                  confirmed_s=(t - stop_ms) / 1000)
                    break
        result['settling_status'] = 'confirmed' if 'entry_s' in result else 'unconfirmed'
    return result


def compare(recording, a, b, target_id, tolerance_cm):
    if not all(math.isfinite(v) for v in (*a, *b, tolerance_cm)) or tolerance_cm <= 0:
        raise ValueError('reference coordinates must be finite and tolerance positive')
    if math.dist(a, b) <= 2 * tolerance_cm / 100:
        raise ValueError('reference tolerance regions overlap')
    with recording.open(encoding='utf-8', newline='') as source:
        data = [row for row in csv.DictReader(source) if int(row['target_id']) == target_id]
    with recording.with_suffix('.markers.csv').open(encoding='utf-8', newline='') as source:
        marks = list(csv.DictReader(source))
    if [(int(m['lap']), m['event'], m['point']) for m in marks] != sequence():
        raise ValueError('incomplete or out-of-order physical markers')
    times = [int(m['unix_ms']) for m in marks]
    mono = [int(m['monotonic_ns']) for m in marks]
    if any(t2 <= t1 or abs((t2-t1) - (m2-m1)/1e6) > 100 for t1, t2, m1, m2 in zip(times, times[1:], mono, mono[1:])):
        raise ValueError('marker clock discontinuity')
    meta = json.loads(Path(str(recording) + '.json').read_text(encoding='utf-8'))
    if meta['status'] != 'completed' or not meta['started_unix_ms'] <= times[0] < times[-1] <= meta['updated_unix_ms']:
        raise ValueError('complete the ECS capture covering all physical markers first')
    if not data:
        raise ValueError('object ID absent in recording')
    intervals = []
    for i, marker in enumerate(marks[:-1]):
        if marker['event'] not in ('ready', 'stop'):
            continue
        stop, end = times[i:i+2]
        if end-stop < 19990:
            raise ValueError('a reference dwell was shorter than 20 seconds')
        point = a if marker['point'] == 'A' else b
        start = times[i-1] if i else stop
        prior = [row for row in data if int(row['capture_unix_ms']) <= start]
        segment = ([prior[-1]] if prior else []) + [row for row in data if start < int(row['capture_unix_ms']) < end]
        dwell = [row for row in data if stop <= int(row['capture_unix_ms']) < end]
        ts = [int(row['capture_unix_ms']) for row in segment]
        origin = {(row['generation'], row['origin_x'], row['origin_y'], row['origin_z']) for row in segment}
        continuous = (bool(prior) and bool(dwell) and len(origin) == 1 and end-ts[-1] <= 3000
                      and all(0 < y-x <= 3000 for x, y in zip(ts, ts[1:]))
                      and all(row['valid'] == '1' for row in segment))
        item = {'lap': int(marker['lap']), 'point': marker['point'], 'continuous_track': continuous}
        for prefix in ('before', 'after'):
            item[prefix] = metrics(dwell, point, stop, end, prefix, tolerance_cm / 100, bool(i) and continuous)
        intervals.append(item)
    return {'reference_world_xy_m': {'A': a, 'B': b}, 'target_id': target_id,
            'tolerance_cm': tolerance_cm, 'clock': 'ECS publish time minus manual physical stop marker',
            'criterion': 'First of three fresh OBSERVED samples inside XY tolerance, at most 3s apart. An out-of-band prediction breaks the streak. Confirmation time is reported separately.',
            'intervals': intervals}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('mark').add_argument('recording', type=Path)
    report = commands.add_parser('compare')
    report.add_argument('recording', type=Path)
    report.add_argument('--a', type=float, nargs=2, required=True)
    report.add_argument('--b', type=float, nargs=2, required=True)
    report.add_argument('--id', type=int, required=True)
    report.add_argument('--tolerance-cm', type=float, default=2)
    args = parser.parse_args()
    if args.command == 'mark':
        mark(args.recording)
    else:
        print(json.dumps(compare(args.recording, args.a, args.b, args.id, args.tolerance_cm), indent=2))
