"""Read-only state queries against localhost devices; verifies diagnostic evidence."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import uuid
from test_lidar_interlock import Probe, ROOT, wait_for


def run(executable):
    board = Probe()
    board.boot_unknown = True
    process = None
    try:
        with tempfile.TemporaryDirectory(prefix='device-diagnostics-') as directory:
            root = Path(directory)
            config = json.loads((ROOT / 'configs/team-demo.json').read_text())
            for name, port in [('rdr', board.rdr_port), ('fcs', board.fcs_port)]:
                config[name]['local'] = {'ip': '127.0.0.1', 'port': 0}
                config[name]['peer'] = {'ip': '127.0.0.1', 'port': port}
            path = root / 'config.json'
            path.write_text(json.dumps(config))
            trace = root / 'events.devices.jsonl'
            process = subprocess.Popen([str(executable), '--config', str(path),
                '--event-log', str(root / 'events.log'), '--ipc-suffix', '_trace_' + uuid.uuid4().hex],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True,
                creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))

            def rows():
                result = []
                if trace.exists():
                    for line in trace.read_text().splitlines():
                        try: result.append(json.loads(line))
                        except json.JSONDecodeError: pass  # Concurrent append may be incomplete.
                return result

            wait_for(lambda: any(r['event'] == 'status_accepted' and not r['lidar_known'] for r in rows()))
            board.boot_unknown = False
            wait_for(lambda: any(r['event'] == 'status_accepted' and r['lidar_known'] for r in rows()))
            wait_for(lambda: any(r['source'] == 'FCS' and r['event'] == 'exchange_complete' for r in rows()))
            board.boot_fcs_disconnected = True
            wait_for(lambda: any(r['source'] == 'FCS' and r['event'] == 'exchange_timeout' for r in rows()))
            board.boot_fcs_disconnected = False
            process.stdin.write('quit\n'); process.stdin.flush()
            _, error = process.communicate(timeout=12)
            assert process.returncode == 0, error
            data = [json.loads(line) for line in trace.read_text().splitlines()]
            assert len({r['run'] for r in data}) == 1
            assert all(r['dropped_total'] == 0 for r in data)
            assert all('unix_us' in r and 'mono_us' in r for r in data)
            for r in data:
                if r['source'] == 'UDP' and r['event'] in ('rx', 'tx'):
                    assert len(bytes.fromhex(r['hex'])) == r['length'] and not r['truncated']
            statuses = [r for r in data if r['event'] == 'status_accepted']
            wire = [r for r in data if r['source'] == 'UDP' and r['event'] == 'rx']
            for status in statuses:
                assert bool(status['flags'] & 16) == status['lidar_known']
                assert any(r.get('tx_seq') == status['tx_seq'] and
                    r.get('message_id') == status['message_id'] and
                    int(r['hex'][40:44], 16) == status['flags'] for r in wire)
            print('PASS: device trace raw bytes, state transition, FCS timeout, shutdown flush')
    finally:
        if process and process.poll() is None:
            process.kill(); process.communicate()
        board.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--console', type=Path, required=True)
    run(parser.parse_args().console.resolve())
