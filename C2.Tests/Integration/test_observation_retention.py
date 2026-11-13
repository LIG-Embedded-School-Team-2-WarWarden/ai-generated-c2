"""Display continuity and command rejection with independent UDP/IPC fixtures."""
import argparse
import json
from pathlib import Path
import queue
import struct
import subprocess
import tempfile
import threading
import time
import uuid
from test_lidar_interlock import Probe, ROOT, wait_for
from test_station import Gui, target_records, valid_count


class PausedBoard(Probe):
    def __init__(self):
        self.allow_off = False
        super().__init__()

    def _fcs_request(self, raw, peer, now):
        if struct.unpack_from('!H', raw)[0] == 0xEF04 and struct.unpack_from('!H', raw, 6)[0] == 0 and not self.allow_off:
            self.fcs_tx = (self.fcs_tx + 1) & 65535
            self.fcs.sendto(struct.pack('!6H', 0xFE04, self.fcs_tx, struct.unpack_from('!H', raw, 2)[0], 1, 1, 0), peer)
            return
        super()._fcs_request(raw, peer, now)


def run(executable, new_identity=False):
    board = PausedBoard()
    process = gui = None
    try:
        with tempfile.TemporaryDirectory(prefix='retention-', ignore_cleanup_errors=True) as directory:
            config = json.loads((ROOT / 'configs/team-demo.json').read_text())
            for name, port in [('rdr', board.rdr_port), ('fcs', board.fcs_port)]:
                config[name]['local'] = {'ip': '127.0.0.1', 'port': 0}
                config[name]['peer'] = {'ip': '127.0.0.1', 'port': port}
            path = Path(directory) / 'config.json'; path.write_text(json.dumps(config))
            suffix = '_retention_' + uuid.uuid4().hex
            process = subprocess.Popen([str(executable), '--config', str(path), '--ipc-suffix', suffix,
                '--event-log', str(Path(directory) / 'events.log')], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            output = queue.Queue()
            def reader():
                for line in process.stdout:
                    try: output.put(json.loads(line))
                    except json.JSONDecodeError: pass
            threading.Thread(target=reader, daemon=True).start()
            assert output.get(timeout=8)['ready']
            def ask(command):
                process.stdin.write(command+'\n'); process.stdin.flush(); return output.get(timeout=6)
            wait_for(lambda: ask('rdr status')['connected'])
            wait_for(lambda: ask('fcs status')['connected'])
            gui = Gui(suffix)
            # Startup now requires an acknowledged FCS OFF before ED06/ED07.
            board.allow_off = True
            assert gui.command(6,target=1)==0
            before = target_records(gui.wait(lambda f: valid_count(f)==10))
            board.allow_off = False
            ids = {t[0] for t in before}
            assert ask('fcs laser on')['result']==0
            paused = target_records(gui.wait(lambda f: len(target_records(f))==10 and all(t[8]==4 and t[9]==0 for t in target_records(f))))
            assert {t[0] for t in paused}==ids
            # Longer than the normal 3s DE03 expiration: rows and coordinates persist.
            end = time.monotonic()+3.3
            while time.monotonic()<end:
                rows=target_records(gui.frame())
                assert {t[0] for t in rows}==ids and all(t[8]==4 and t[9]==0 for t in rows)
                assert gui.command(1, target=min(ids))!=0
                time.sleep(.1)
            # Prevent fresh data briefly after resume to exercise WAITING explicitly.
            with board.lock:
                board.scenario='empty'
                if new_identity:
                    board.targets=[{'id':2001,'xyz':(-1.4,2.8,.35)}]
            board.allow_off=True
            assert ask('fcs laser off')['result']==0
            gui.wait(lambda f: len(target_records(f))==10 and all(t[8]==5 and t[9]==0 for t in target_records(f)))
            assert gui.command(1, target=min(ids))!=0
            assert gui.command(3, target=min(ids))!=0
            with board.lock: board.scenario='normal'
            if new_identity:
                after=target_records(gui.wait(lambda f: any(t[0]==2001 and t[9] for t in target_records(f)),timeout=7))
                assert not any(t[0] in ids and t[9] for t in after)
                gui.wait(lambda f: all(t[0] not in ids or (not t[9] and t[8] not in (4,5)) for t in target_records(f)),timeout=5)
            else:
                after=target_records(gui.wait(lambda f: valid_count(f)==10))
                assert {t[0] for t in after}==ids and all(t[8]==0 for t in after)
            assert board.error is None,board.error
            gui.close();gui=None
            process.stdin.write('quit\n');process.stdin.flush();process.wait(timeout=8)
            assert process.returncode==0
            print('PASS: optical pause display, blocked commands, fresh reacquisition; new_identity=',new_identity)
    finally:
        if gui:gui.close()
        if process and process.poll() is None:process.kill();process.wait()
        board.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--console',type=Path,required=True)
    args=parser.parse_args()
    for changed in (False,True):run(args.console.resolve(),changed)
