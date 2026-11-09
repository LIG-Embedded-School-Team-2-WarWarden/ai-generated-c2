"""Tune GUI stability using a stationary recording and the production C++ filter.

No hardware access. The first half selects parameters; the second half is
reported separately. Motion response is synthetic, not measured on hardware.
"""
import argparse
import csv
from datetime import datetime, timedelta, timezone
import hashlib
import itertools
import json
import math
from pathlib import Path
import statistics
import subprocess


def jitter(points):
    return math.sqrt(sum(statistics.pvariance([p[a] for p in points]) for a in range(3))) * 100


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--tests', type=Path, required=True, help='C2.Tests.exe with offline replay support')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--per-axis', action='store_true', help='Estimate and tune independent world XYZ noise')
    args = parser.parse_args()
    with args.csv.open(newline='', encoding='utf-8') as source:
        rows = list(csv.DictReader(source))
    metadata = json.loads(Path(str(args.csv) + '.json').read_text(encoding='utf-8'))
    if metadata['status'] != 'completed' or not rows:
        parser.error('use a completed stationary recording')
    ids = sorted({int(r['target_id']) for r in rows})
    last, segment_start, segment_id = {}, {}, {}
    eligible_rows, segment_keys = [], []
    segments = 0
    for r in rows:
        id = int(r['target_id'])
        state = (r['generation'], *(r['origin_' + a] for a in 'xyz'))
        seen = float(r['received_s'])
        elapsed = float(r['elapsed_s'])
        if r['valid'] != '1':
            last.pop(id, None)
            eligible_rows.append(False)
            segment_keys.append(None)
            continue
        if id not in last or last[id][0] != state or not 0 < seen - last[id][1] <= 3:
            segments += 1
            segment_start[id], segment_id[id] = elapsed, segments
        last[id] = state, seen
        eligible_rows.append(elapsed - segment_start[id] >= 2)
        segment_keys.append((id, segment_id[id]))
    split = metadata['requested_duration_s'] / 2

    def indices(id, phase='all', observed=True):
        return [i for i, r in enumerate(rows) if int(r['target_id']) == id
                and eligible_rows[i]
                and (not observed or r['track_state'] == '1')
                and (phase != 'train' or float(r['elapsed_s']) < split)
                and (phase != 'validation' or float(r['elapsed_s']) >= split)]

    train = {id: indices(id, 'train') for id in ids}
    minimum = 15 if args.per_axis else 5
    if any(len(train[id]) < minimum or len(indices(id, 'validation')) < minimum for id in ids):
        parser.error(f'need at least {minimum} OBSERVED samples per object in each half after segment warmup')
    # Within-object scatter avoids treating different object positions as noise.
    variance = []
    for axis in 'xyz':
        ss = df = 0
        for segment in sorted({segment_keys[i] for id in ids for i in train[id]}):
            values = [float(rows[i]['before_' + axis]) for id in ids for i in train[id]
                      if segment_keys[i] == segment]
            if len(values) < 2:
                continue
            ss += (len(values) - 1) * statistics.variance(values)
            df += len(values) - 1
        if not df:
            parser.error('insufficient repeated training observations within continuous segments')
        variance.append(ss / df)
    measured_stddev = list(map(math.sqrt, variance))
    # Sparse stationary data: retain a 25% margin and round upward to 5 mm.
    measurement = max(.005, math.ceil(max(measured_stddev) * 1.25 / .005) * .005)
    baseline = metadata['filter']
    configs = [{'name': 'baseline', 'filter': baseline}]
    axis_measurement = [max(.003, round(math.ceil(value * 1.25 / .001) * .001, 6)) for value in measured_stddev]
    if args.per_axis:
        for ratios in itertools.product((.18, .24, .30, .36), repeat=3):
            for sigma in (2.5, 3., 3.5):
                configs.append({'name': 'axes_' + '_'.join(map(str, ratios)) + f'_s{sigma}', 'filter': {
                    'enabled': True, 'adaptive_motion': True, 'measurement_stddev_m': .03,
                    'process_stddev_m_sqrt_s': .01, 'motion_process_stddev_m_sqrt_s': .08,
                    'measurement_stddev_xyz_m': axis_measurement,
                    'process_stddev_xyz_m_sqrt_s': [round(r * q, 8) for r, q in zip(axis_measurement, ratios)],
                    'motion_process_stddev_xyz_m_sqrt_s': [round(r * 8 / 3, 8) for r in axis_measurement],
                    'motion_trigger_sigma': sigma}})
    for q in (() if args.per_axis else (.003, .005, .008, .01, .012, .015, .02)):
        configs.append({'name': f'q_{q:g}', 'filter': {
            'enabled': True, 'measurement_stddev_m': measurement,
            'process_stddev_m_sqrt_s': q, 'adaptive_motion': True,
            'motion_process_stddev_m_sqrt_s': .08, 'motion_trigger_sigma': 2.5}})
    datasets = [{'name': 'recorded', 'samples': [
        {'id': int(r['target_id']), 'generation': int(r['generation']),
         'seen': float(r['received_s']), 'track_state': int(r['track_state']),
         'valid': r['valid'] == '1', 'origin': [float(r['origin_' + a]) for a in 'xyz'],
         'world': [float(r['before_' + a]) if r['valid'] == '1' else 0 for a in 'xyz']} for r in rows]}]
    motion_axes = range(3) if args.per_axis else range(1)
    steps = (.01, .02, .05, .10, .20) if args.per_axis else (.05, .10, .20)
    for axis in motion_axes:
        for step in steps:
            datasets.append({'name': f'step_{axis}_{step:g}', 'samples': [
                {'id': 1, 'seen': t, 'world': [step if t >= 50 and a == axis else 0 for a in range(3)]}
                for t in range(90)]})
        datasets.append({'name': f'ramp_{axis}', 'samples': [
            {'id': 1, 'seen': t, 'world': [max(0, t - 49) * .02 if a == axis else 0 for a in range(3)]}
            for t in range(70)]})
    directory = args.output_dir or args.csv.parent / ('tuning-' + datetime.now(
        timezone(timedelta(hours=9))).strftime('%Y%m%d-%H%M%S'))
    directory.mkdir(parents=True, exist_ok=False)
    request_path, output_path = directory / 'replay-input.json', directory / 'replay-output.json'
    request_path.write_text(json.dumps({'configs': configs, 'datasets': datasets}), encoding='utf-8')
    subprocess.run([str(args.tests.resolve()), '--replay-gui-filter', str(request_path.resolve()),
                    str(output_path.resolve())], check=True)
    runs = {(r['config'], r['dataset']): r['xyz'] for r in json.loads(output_path.read_text())['runs']}
    baseline_error = max(abs(point[a] - float(row['after_' + axis]))
                         for point, row in zip(runs['baseline', 'recorded'], rows)
                         if row['valid'] == '1' for a, axis in enumerate('xyz'))
    if baseline_error > 1e-6:
        raise ValueError(f'baseline replay differs from the actual GUI recording: {baseline_error} m')
    evaluations = []
    for config in configs:
        name = config['name']
        output = runs[name, 'recorded']
        stats = {}
        for id in ids:
            stats[str(id)] = {}
            for phase in ('train', 'validation', 'all'):
                for observed in (True, False):
                    keep = indices(id, phase, observed)
                    if len(keep) < 2:
                        continue
                    key = phase + ('_observed' if observed else '_including_predicted')
                    before = [[float(rows[i]['before_' + a]) for a in 'xyz'] for i in keep]
                    after = [output[i] for i in keep]
                    raw_jitter, filtered_jitter = jitter(before), jitter(after)
                    stats[str(id)][key] = {'n': len(keep), 'raw_rms_cm': raw_jitter,
                        'filtered_rms_cm': filtered_jitter,
                        'reduction_percent': 100 * (1 - filtered_jitter / raw_jitter) if raw_jitter else None,
                        'mean_shift_cm': [100 * (statistics.mean(p[a] for p in after) -
                                                statistics.mean(p[a] for p in before)) for a in range(3)],
                        'raw_stddev_xyz_cm': [statistics.pstdev(p[a] for p in before) * 100 for a in range(3)],
                        'filtered_stddev_xyz_cm': [statistics.pstdev(p[a] for p in after) * 100 for a in range(3)],
                        'raw_peak_to_peak_xyz_cm': [(max(p[a] for p in before) - min(p[a] for p in before)) * 100 for a in range(3)],
                        'filtered_peak_to_peak_xyz_cm': [(max(p[a] for p in after) - min(p[a] for p in after)) * 100 for a in range(3)]}
        response = {}
        acceptable = True
        for axis in motion_axes:
            for step in steps:
                values = runs[name, f'step_{axis}_{step:g}'][50:]
                count = next((i + 1 for i, p in enumerate(values) if p[axis] >= step * .9), None)
                response[f'{"xyz"[axis]}_{int(step * 100)}cm_updates_to_90pct'] = count
                limit = 10 if step < .05 else 8 if step == .05 else 3
                acceptable &= count is not None and count <= limit
            ramp = runs[name, f'ramp_{axis}'][50:]
            lag = max(abs((i + 1) * .02 - p[axis]) * 100 for i, p in enumerate(ramp))
            response[f'{"xyz"[axis]}_ramp_2cm_s_max_lag_cm'] = lag
            if args.per_axis:
                acceptable &= lag <= 6
        score = statistics.mean(stats[str(id)]['train_observed']['filtered_rms_cm'] for id in ids)
        evaluations.append({'name': name, 'filter': config['filter'], 'training_score_cm': score,
                            'meets_response_constraints': acceptable, 'synthetic_response_1hz': response,
                            'recording_replay': stats})
    eligible = [r for r in evaluations if r['name'] != 'baseline' and r['meets_response_constraints']]
    if not eligible:
        raise ValueError('no candidate meets the response limits; configuration was not changed')
    selected = min(eligible, key=lambda r: r['training_score_cm'])
    proposed_name = selected['name']
    def validation_better(id):
        old = evaluations[0]['recording_replay'][str(id)]['validation_observed']
        new = selected['recording_replay'][str(id)]['validation_observed']
        return (new['filtered_rms_cm'] < old['filtered_rms_cm'] and
                all(n <= o * 1.05 for n, o in zip(new['filtered_stddev_xyz_cm'], old['filtered_stddev_xyz_cm'])))
    accepted = all(validation_better(id) for id in ids)
    if args.per_axis and not accepted:
        # Do not search candidates again using the validation data.
        selected = evaluations[0]
    report = {'source_csv': str(args.csv.resolve()), 'source_sha256': hashlib.sha256(args.csv.read_bytes()).hexdigest(),
        'method': 'Production C++ filter replay of stationary recorded world XYZ; first half used for parameter selection, second half reported separately. Synthetic motion checks at 1 Hz. No ground-truth position, no absolute accuracy claim.',
        'split_s': split, 'warmup_s_per_segment': 2, 'segments': segments,
        'invalid_rows': sum(r['valid'] != '1' for r in rows), 'per_axis': args.per_axis,
        'training_observed_counts': {id: len(train[id]) for id in ids},
        'training_pooled_stddev_xyz_m': measured_stddev, 'baseline_max_error_m': baseline_error,
        'selection_rule': 'Lowest training OBSERVED 3D jitter subject to 1Hz response limits: 5cm <=8 updates; 10/20cm <=3. Per-axis mode also requires 1/2cm <=10 updates and a 2cm/s ramp lag <=6cm on every axis.',
        'training_candidate': proposed_name, 'validation_accepted': accepted,
        'selected_name': selected['name'], 'selected_filter': selected['filter'], 'evaluations': evaluations}
    (directory / 'tuning-report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    (directory / 'selected-filter.json').write_text(json.dumps(selected['filter'], indent=2) + '\n', encoding='utf-8')
    with (directory / 'comparison.csv').open('w', newline='', encoding='utf-8') as target:
        writer = csv.writer(target)
        writer.writerow(['elapsed_s', 'target_id', 'track_state', 'valid', 'segment', 'past_warmup', 'phase', *('raw_' + a for a in 'xyz'),
                         *('baseline_' + a for a in 'xyz'), *('tuned_' + a for a in 'xyz')])
        for index, (row, old, new) in enumerate(zip(rows, runs['baseline', 'recorded'], runs[selected['name'], 'recorded'])):
            writer.writerow([row['elapsed_s'], row['target_id'], row['track_state'], row['valid'],
                             segment_keys[index][1] if segment_keys[index] else '', int(eligible_rows[index]),
                             'train' if float(row['elapsed_s']) < split else 'validation',
                             *(row['before_' + a] for a in 'xyz'), *(old or ['', '', '']), *(new or ['', '', ''])])
    print('Selected:', json.dumps(selected['filter']))
    for id in ids:
        for phase in ('all_observed', 'validation_observed', 'all_including_predicted'):
            old = evaluations[0]['recording_replay'][str(id)][phase]
            new = selected['recording_replay'][str(id)][phase]
            percent = f"{100 * (1 - new['filtered_rms_cm'] / old['filtered_rms_cm']):.1f}%" if old['filtered_rms_cm'] else 'n/a'
            print(f"ID {id} {phase} N={new['n']}: {old['filtered_rms_cm']:.3f} -> {new['filtered_rms_cm']:.3f} cm; additional reduction {percent}")
    print('Synthetic 1Hz response:', selected['synthetic_response_1hz'])
    print('Report:', (directory / 'tuning-report.json').resolve())


if __name__ == '__main__':
    main()
