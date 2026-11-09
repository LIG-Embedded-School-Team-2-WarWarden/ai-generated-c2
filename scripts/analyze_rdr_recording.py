"""Compare paired stationary world coordinates; standard library only.

Jitter is spread about each stream's own mean, not error against ground truth.
Keep objects stationary and retain the CSV metadata alongside this report.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def summarize(rows):
    result = {'samples': len(rows), 'first_s': float(rows[0]['elapsed_s']),
              'last_s': float(rows[-1]['elapsed_s'])}
    for phase in ('before', 'after'):
        axes = {}
        for axis in 'xyz':
            values = [float(row[f'{phase}_{axis}']) for row in rows]
            axes[axis] = {'mean_m': statistics.mean(values),
                          'stddev_cm': statistics.pstdev(values) * 100,
                          'peak_to_peak_cm': (max(values) - min(values)) * 100}
        result[phase] = axes
        result[f'{phase}_rms_3d_cm'] = math.sqrt(sum(
            axis['stddev_cm'] ** 2 for axis in axes.values()))
    before = result['before_rms_3d_cm']
    result['rms_3d_reduction_percent'] = (
        100 * (1 - result['after_rms_3d_cm'] / before) if before > 0 else None)
    result['mean_shift_cm'] = {axis: 100 * (result['after'][axis]['mean_m'] -
                                                 result['before'][axis]['mean_m']) for axis in 'xyz'}
    return result


def analyze(path, warmup, reference_segment=None, reference_enu=None):
    if (reference_segment is None) != (reference_enu is None):
        raise ValueError('supply both --reference-segment and --reference-enu')
    if reference_enu is not None and not all(map(math.isfinite, reference_enu)):
        raise ValueError('reference coordinates must be finite metres')
    metadata_path = Path(str(path) + '.json')
    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    with path.open(newline='', encoding='utf-8') as source:
        rows = list(csv.DictReader(source))
    active = {}
    segments = []
    invalid = 0
    for row in rows:
        key = (row['generation'], row['target_id'], *(row[f'origin_{axis}'] for axis in 'xyz'))
        if row['valid'] != '1':
            invalid += 1
            active.pop(key, None)
            continue
        numeric = [float(row[f'{phase}_{axis}']) for phase in ('before', 'after') for axis in 'xyz']
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError('non-finite valid coordinate in recording')
        previous = active.get(key)
        received = float(row['received_s'])
        if previous is None or not (0 < received - float(previous[-1]['received_s']) <= 3):
            segment = []
            segments.append((key, segment))
            active[key] = segment
        active[key].append(row)
    reports = []
    if reference_segment is not None and not 1 <= reference_segment <= len(segments):
        raise ValueError(f'reference segment {reference_segment} does not exist in this recording')
    for index, (key, segment) in enumerate(segments, 1):
        start = float(segment[0]['elapsed_s'])
        retained = [row for row in segment if float(row['elapsed_s']) - start >= warmup]
        item = {'segment': index, 'generation': int(key[0]), 'target_id': int(key[1]),
                'origin_m': list(map(float, key[2:])), 'total_valid_samples': len(segment),
                'warmup_excluded_samples': len(segment) - len(retained), 'groups': {}}
        for name, state in (('all', None), ('observed', '1'), ('predicted', '2')):
            group = [row for row in retained if state is None or row['track_state'] == state]
            if index == reference_segment and name == 'observed':
                error_cm = None
                if group:
                    error_cm = {
                        direction: 100 * (statistics.mean(float(row[f'before_{axis}']) for row in group) - ref)
                        for axis, direction, ref in zip('xyz', ('east', 'north', 'up'), reference_enu)
                    }
                item['reference_comparison'] = {
                    'reference_enu_m': list(reference_enu), 'observed_samples': len(group),
                    'before_mean_minus_reference_cm': error_cm}
            if len(group) >= 2:
                item['groups'][name] = summarize(group)
        reports.append(item)
    return {'source_csv': str(path.resolve()), 'recording_metadata': metadata,
            'method': 'Population standard deviation and 3D RMS about each stream mean; paired rows only; separate continuous target segments. Not absolute accuracy.',
            'warmup_s_per_segment': warmup, 'total_rows': len(rows),
            'invalid_rows_excluded': invalid, 'segments': reports}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--warmup-seconds', type=float, default=2)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--reference-segment', type=int, help='segment number from this recording summary')
    parser.add_argument('--reference-enu', nargs=3, type=float, metavar=('E', 'N', 'U'),
                        help='independently measured stationary world point, in metres')
    args = parser.parse_args()
    if not math.isfinite(args.warmup_seconds) or args.warmup_seconds < 0:
        parser.error('warmup must be nonnegative and finite')
    try:
        report = analyze(args.csv, args.warmup_seconds, args.reference_segment, args.reference_enu)
    except ValueError as exc:
        parser.error(str(exc))
    destination = args.output or args.csv.with_suffix('.summary.json')
    if destination.resolve() in (args.csv.resolve(), Path(str(args.csv) + '.json').resolve()):
        parser.error('summary output cannot overwrite the recording or metadata')
    destination.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print('Recording status:', report['recording_metadata']['status'])
    print('ID segment group N | XYZ stddev before -> after (cm) | 3D RMS before -> after (cm) | reduction')
    count = 0
    for segment in report['segments']:
        if 'reference_comparison' in segment:
            comparison = segment['reference_comparison']
            errors = comparison['before_mean_minus_reference_cm']
            detail = (' / '.join(f'{v:+.3f}' for v in errors.values()) if errors is not None
                      else 'no observed samples after warmup')
            print(f"Segment {segment['segment']} ID {segment['target_id']} observed "
                  f"N={comparison['observed_samples']} | mean minus reference E/N/U (cm): {detail}")
        for name, stats in segment['groups'].items():
            xyz = ' / '.join(f"{stats['before'][a]['stddev_cm']:.3f}->{stats['after'][a]['stddev_cm']:.3f}" for a in 'xyz')
            reduction = stats['rms_3d_reduction_percent']
            percent = f'{reduction:.1f}%' if reduction is not None else 'n/a'
            print(f"{segment['target_id']} {segment['segment']} {name} {stats['samples']} | {xyz} | "
                  f"{stats['before_rms_3d_cm']:.3f}->{stats['after_rms_3d_cm']:.3f} | {percent}")
            count += 1
    print('Summary:', destination.resolve())
    if not count:
        print('Insufficient paired samples after warmup; no reduction can be estimated.')


if __name__ == '__main__':
    main()
