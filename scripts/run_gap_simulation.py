"""Run independent UDP lidar/camera simulators against the built ECS console.

Example: python scripts/run_gap_simulation.py --console build/gap/C2.Console.exe
Only loopback devices are used. Reports contain actual FCS command angles/times.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--console', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('build/gap-simulation'))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    console = args.console.resolve(strict=True)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    scenarios = [('moving_right', 1, False), ('moving_left', -1, False),
                 ('stationary', 0, False), ('filtered_motion', 1, True)]
    results = []
    for name, direction, filtered in scenarios:
        report = output / f'{name}.json'
        command = [sys.executable, str(root / 'C2.Tests/Integration/test_station.py'),
                   '--console', str(console), '--gap', '--gap-direction', str(direction),
                   '--report', str(report)]
        if filtered:
            command.append('--filtered')
        result = subprocess.run(command, cwd=root, capture_output=True, text=True, timeout=60)
        (output / f'{name}.log').write_text(result.stdout + result.stderr, encoding='utf-8')
        row = {'scenario': name, 'passed': result.returncode == 0, 'exit_code': result.returncode}
        if row['passed']:
            row.update(json.loads(report.read_text(encoding='utf-8')))
        results.append(row)
        print(f"{'PASS' if row['passed'] else 'FAIL'}: {name}", flush=True)
    summary = {'passed': all(row['passed'] for row in results),
               'console': str(console), 'transport': '127.0.0.1 UDP + Windows IPC',
               'limitations': 'Protocol simulator: no physical gimbal dynamics or real lidar noise.',
               'scenarios': results}
    (output / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
