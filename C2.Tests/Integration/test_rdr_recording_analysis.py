"""Known-value checks for stationary jitter statistics, not real sensor results."""
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

root = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('analysis', root / 'scripts/analyze_rdr_recording.py')
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


class AnalysisTest(unittest.TestCase):
    def test_paired_statistics_and_segments(self):
        fields = ['generation', 'target_id', 'origin_x', 'origin_y', 'origin_z',
                  'valid', 'received_s', 'elapsed_s', 'track_state',
                  'before_x', 'before_y', 'before_z', 'after_x', 'after_y', 'after_z']
        rows = []
        # X alternates +/-2 cm before and +/-1 cm after. A large startup
        # transient must be excluded; predicted rows need their own statistics.
        for i in range(6):
            before = 100 if i < 2 else (0.02 if i % 2 else -0.02)
            after = before / 2
            rows.append(dict(zip(fields, [1, 7, 0, 0, 0, 1, i, i,
                                           1 if i < 4 else 2, before, 0, 0, after, 0, 0])))
        lost = dict(rows[-1], valid=0, elapsed_s=6, received_s=6)
        rows.extend([lost, dict(rows[-1], elapsed_s=7, received_s=7),
                     dict(rows[-1], elapsed_s=11, received_s=11)])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'fixture.csv'
            with path.open('w', newline='') as output:
                writer = csv.DictWriter(output, fields)
                writer.writeheader()
                writer.writerows(rows)
            Path(str(path) + '.json').write_text(json.dumps({'status': 'fixture'}))
            report = analysis.analyze(path, 2)
            rows[4]['before_z'] = 99  # Prediction must not affect the reference comparison.
            with path.open('w', newline='') as output:
                writer = csv.DictWriter(output, fields)
                writer.writeheader()
                writer.writerows(rows)
            comparison = analysis.analyze(path, 2, 1, (0.1, -0.2, 0.3))['segments']
            self.assertEqual(comparison[0]['reference_comparison']['observed_samples'], 2)
            self.assertEqual(comparison[0]['reference_comparison']['before_mean_minus_reference_cm'],
                             {'east': -10, 'north': 20, 'up': -30})
            self.assertNotIn('reference_comparison', comparison[1])
            empty = analysis.analyze(path, 2, 2, (0, 0, 0))['segments'][1]
            self.assertIsNone(empty['reference_comparison']['before_mean_minus_reference_cm'])
        self.assertEqual(report['invalid_rows_excluded'], 1)
        self.assertEqual(len(report['segments']), 3)
        segment = report['segments'][0]
        self.assertEqual(segment['warmup_excluded_samples'], 2)
        stats = segment['groups']['all']
        self.assertEqual(stats['samples'], 4)
        self.assertAlmostEqual(stats['before']['x']['stddev_cm'], 2)
        self.assertAlmostEqual(stats['after']['x']['stddev_cm'], 1)
        self.assertAlmostEqual(stats['before']['x']['peak_to_peak_cm'], 4)
        self.assertAlmostEqual(stats['rms_3d_reduction_percent'], 50)
        self.assertEqual(segment['groups']['observed']['samples'], 2)
        self.assertEqual(segment['groups']['predicted']['samples'], 2)
        self.assertFalse(report['segments'][1]['groups'])

    def test_zero_jitter_and_mean_shift(self):
        row = {'elapsed_s': '0', **{f'{p}_{a}': str(1 if p == 'before' else 2)
                                   for p in ('before', 'after') for a in 'xyz'}}
        stats = analysis.summarize([row, dict(row, elapsed_s='1')])
        self.assertIsNone(stats['rms_3d_reduction_percent'])
        self.assertEqual(stats['mean_shift_cm'], dict.fromkeys('xyz', 100))


if __name__ == '__main__':
    unittest.main()
