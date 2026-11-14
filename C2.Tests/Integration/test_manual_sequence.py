"""GUI v3 manual flow against independent localhost FCS/RDR; no real devices."""
import argparse,ctypes,json,queue,subprocess,tempfile,threading,time,uuid
from pathlib import Path
from test_lidar_interlock import Probe,ROOT,wait_for
from test_station import Gui,target_records,valid_count

def run(executable,scenario):
    board=Probe();process=gui=None
    try:
        with tempfile.TemporaryDirectory(prefix='manual-sequence-',ignore_cleanup_errors=True) as directory:
            config=json.loads((ROOT/'configs/team-demo.json').read_text())
            for name,port in [('rdr',board.rdr_port),('fcs',board.fcs_port)]:
                config[name]['local']={'ip':'127.0.0.1','port':0}
                config[name]['peer']={'ip':'127.0.0.1','port':port}
            path=Path(directory)/'config.json';path.write_text(json.dumps(config))
            suffix='_manual_'+uuid.uuid4().hex;log=Path(directory)/'events.log'
            process=subprocess.Popen([str(executable),'--config',str(path),'--ipc-suffix',suffix,'--event-log',str(log)],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            output=queue.Queue()
            def reader():
                for line in process.stdout:
                    try:output.put(json.loads(line))
                    except json.JSONDecodeError:pass
            threading.Thread(target=reader,daemon=True).start()
            assert output.get(timeout=10)['ready']
            def ask(command):
                process.stdin.write(command+'\n');process.stdin.flush();return output.get(timeout=9)
            wait_for(lambda:ask('rdr status')['connected'])
            wait_for(lambda:ask('fcs status')['connected'])
            gui=Gui(suffix);assert gui.command(6,target=1)==0
            rows=target_records(gui.wait(lambda f:valid_count(f)==10))
            target=rows[0][0]
            def send(command,expected=0):
                value=gui.command(command,target=target)
                assert value==expected,(command,value,gui.last_response,log.read_text(errors='replace'))
            def state():
                gui.command(0);return gui.last_response[7]
            def until(expected,timeout=8):
                try:wait_for(lambda:state()==expected,timeout)
                except Exception:raise AssertionError((expected,gui.last_response,log.read_text(errors='replace')))
            send(1001);assert state()==102
            assert 'rejected' in ask('sequence confirm-hit '+str(target))
            send(1001) # Same registration is idempotent.
            send(1002,5);send(1006,5);send(1008,5)
            send(1005,1) # Cannot skip directly to emission.
            send(1003);until(103)
            if scenario=='unknown':board.boot_unknown=True
            if scenario=='drop_stop':board.drop_stop=True
            send(1004)
            if scenario in ('unknown','drop_stop'):
                until(3,9)
                assert not any(e[:2]==('fcs',1) for e in board.events)
                send(1005,1)
                board.boot_unknown=False;board.drop_stop=False
                send(2)
            else:
                until(104)
                assert board.lidar_inhibited
                time.sleep(.2);assert state()==104
                assert gui.command(1,target=target)!=0 # Legacy path cannot bypass ownership.
                if scenario=='cancel':
                    send(2);wait_for(lambda:not board.lidar_inhibited)
                    assert not any(e[:2]==('fcs',1) for e in board.events)
                else:
                    send(1005);send(1005) # Duplicate request must not emit twice.
                    until(106)
                    assert sum(e[:2]==('fcs',1) for e in board.events)==1,board.events
                    time.sleep(.25);assert board.lidar_inhibited # No automatic resume.
                    if scenario=='missing':board.scenario='empty'
                    if scenario=='new_id':
                        with board.lock:board.targets=[{'id':2001,'xyz':(-1.4,2.8,.35)}]
                    send(1007);until(107)
                    if scenario in ('missing','new_id'):
                        until(3);assert state()!=108 # Absence/new identity never becomes HIT.
                        if scenario=='missing':
                            assert ask('sequence confirm-hit '+str(target))['source']=='operator'
                            until(108)
                    else:
                        until(111)
                        gui.wait(lambda f:any(t[0]==target and t[9] for t in target_records(f)))
                        send(1009);assert state()==102
                    send(2)
                    if scenario=='five':
                        for row in rows[1:5]:
                            target=row[0]
                            gui.wait(lambda f:any(t[0]==target and t[9] for t in target_records(f)))
                            send(1001);send(1003);until(103)
                            send(1004);until(104);send(1005);until(106)
                            assert board.lidar_inhibited
                            send(1007);until(111);send(2)
                        assert sum(e[:2]==('fcs',1) for e in board.events)==5
            assert board.error is None,board.error
            gui.close();gui=None
            process.stdin.write('quit\n');process.stdin.flush();process.wait(timeout=10)
            assert process.returncode==0,process.stderr.read()
            print('PASS manual sequence:',scenario,flush=True)
    finally:
        if gui:gui.close()
        if process and process.poll() is None:process.kill();process.wait()
        board.close()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--console',type=Path,required=True);p.add_argument('--scenario')
    a=p.parse_args()
    # The board has a 10ms service loop; request precise sleeps in this process
    # while testing the production client's intentionally short 80ms deadline.
    ctypes.windll.winmm.timeBeginPeriod(1)
    try:
        for s in ([a.scenario] if a.scenario else ['normal','cancel','unknown','drop_stop','missing','new_id','five']):run(a.console.resolve(),s)
    finally:ctypes.windll.winmm.timeEndPeriod(1)
