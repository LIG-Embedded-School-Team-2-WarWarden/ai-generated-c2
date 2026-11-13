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


def run(executable, new_identity=False, gui_demo=False):
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
            board.allow_off=True  # Acknowledge the startup FCS OFF transaction.
            assert gui.command(6,target=1)==0
            before=target_records(gui.wait(lambda f: valid_count(f)==10))
            ident=before[0][0]
            wait_for(lambda: ask('rdr status')['result']==0)
            wait_for(lambda: ask('fcs status').get('flags')==1)
            wait_for(lambda: gui.command(18)==0)
            board.allow_off=False
            if new_identity:
                assert gui.command(1,target=ident)==0
                wait_for(lambda: gui.command(0)==0 and gui.last_response[7]==2)
            if gui_demo: assert gui.command(3,target=ident)==0
            else:
                enabled=ask('fcs laser on');assert enabled.get('result')==0,enabled
            gui.wait(lambda f: any(t[8]==4 for t in target_records(f)))
            if not new_identity:
                before_assign=gui.command(1,target=ident)
                changed=gui.command(5,rdr=1,fcs=0,xyz=(.01,0,0))
                frame=gui.wait(lambda f: valid_count(f)==0 and all(t[8] not in (4,5) for t in target_records(f)),timeout=2)
                after_assign=gui.command(1,target=ident)
                print(json.dumps({'case':'origin_change_during_pause','inhibited':board.lidar_inhibited,
                    'assign_before':before_assign,'position_change':changed,'valid_after':valid_count(frame),'assign_after':after_assign}))
                assert board.lidar_inhibited and before_assign!=0 and changed==0 and after_assign!=0
                assert valid_count(frame)==0
                board.allow_off=True
                assert ask('fcs laser off')['result']==0
                gui.wait(lambda f: valid_count(f)==10)
                assert gui.command(1,target=ident)==0
            else:
                assert gui.command(0)==0
                during=gui.last_response[7]
                board.allow_off=True
                assert ask('fcs laser off')['result']==0
                gui.wait(lambda f: valid_count(f)==10)
                assert gui.command(0)==0
                print(json.dumps({'case':'automatic_assignment_restore','during':during,'after':gui.last_response[7]}))
                assert gui.last_response[7]==3
                assert gui.command(3,target=ident)!=0
                assert gui.command(1,target=ident)==0
                wait_for(lambda: gui.command(0)==0 and gui.last_response[7]==2)
            board.allow_off=True
            assert board.error is None,board.error
            gui.close();gui=None
            process.stdin.write('quit\n');process.stdin.flush();process.wait(timeout=8)
            assert process.returncode==0
            print('PASS: pause position-edit gate and explicit reassignment',new_identity,'GUI demo=',gui_demo)
    finally:
        if gui:gui.close()
        if process and process.poll() is None:process.kill();process.wait()
        board.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--console',type=Path,required=True)
    args=parser.parse_args()
    for changed in (False,True):run(args.console.resolve(),changed)
    run(args.console.resolve(),True,gui_demo=True)
