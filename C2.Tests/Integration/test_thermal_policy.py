"""Loopback thermal policy; never uses physical device endpoints."""
import argparse, ctypes, json, queue, struct, subprocess, tempfile, threading, uuid
from pathlib import Path
from test_lidar_interlock import Probe, wait_for
from test_station import Gui, valid_count
from test_observation_retention import target_records
ROOT=Path(__file__).resolve().parents[2]
class ThermalProbe(Probe):
    def __init__(self, thermal):
        self.thermal=thermal;self.health_drop=False;self.protected=False
        super().__init__();self.boot_unknown=True
    def _packet(self, kind, *args, **kwargs):
        p=super()._packet(kind,*args,**kwargs)
        if self.protected:p[10]|=128
        return p
    def _rdr_request(self, raw, peer, now):
        q=struct.unpack('!12H',raw) if len(raw)==24 else None
        if q and q[0]==0xED08:
            if self.health_drop:return
            if self.connected and ((q[5]<<16)|q[6])==self.token:
                self._send_rdr(self._packet(0xDE08,q[1],self.thermal,1|(2 if self.protected else 0)))
            return
        super()._rdr_request(raw,peer,now)

def run(exe, thermal):
    board=ThermalProbe(thermal);proc=gui=None
    try:
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
            config=json.loads((ROOT/'configs/team-demo.json').read_text())
            for name,port in [('rdr',board.rdr_port),('fcs',board.fcs_port)]:
                config[name]['local']={'ip':'127.0.0.1','port':0}
                config[name]['peer']={'ip':'127.0.0.1','port':port}
            path=Path(tmp)/'config.json';path.write_text(json.dumps(config))
            suffix='_thermal_'+uuid.uuid4().hex
            proc=subprocess.Popen([str(exe),'--config',str(path),'--ipc-suffix',suffix,
                '--event-log',str(Path(tmp)/'events.log')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,text=True,creationflags=subprocess.CREATE_NO_WINDOW)
            output=queue.Queue()
            def reader():
                for line in proc.stdout:
                    try:output.put(json.loads(line))
                    except json.JSONDecodeError:pass
            threading.Thread(target=reader,daemon=True).start()
            assert output.get(timeout=10)['ready']
            def ask(cmd):
                proc.stdin.write(cmd+'\n');proc.stdin.flush();return output.get(timeout=9)
            wait_for(lambda:ask('rdr status')['connected'])
            wait_for(lambda:ask('fcs status')['connected'])
            gui=Gui(suffix)
            def health(state):
                return gui.command(20)==0 and gui.last_response[6]==state and gui.last_response[7]==1
            wait_for(lambda:health(thermal))
            assert gui.command(6,target=1)==0
            rows=target_records(gui.wait(lambda f:valid_count(f)==10));target=rows[0][0]
            def send(command,expected=0):
                result=gui.command(command,target=target);assert result==expected,(command,result,gui.last_response)
            def state():
                gui.command(0);return gui.last_response[7]
            send(1001);send(1003);wait_for(lambda:state()==103)
            send(1004);wait_for(lambda:state()==104)
            assert gui.command(19)==0 and gui.last_response[7]==0 # Never fabricated optical OFF
            send(1005);wait_for(lambda:state()==106)
            send(1007);wait_for(lambda:state()==111)
            send(2)
            # New warning failure is attributable without pretending the sensor caused the timeout.
            send(1001);send(1003);wait_for(lambda:state()==103)
            board.drop_stop=True;send(1004);wait_for(lambda:state()==3,9)
            gui.command(1010);assert gui.last_response[4]==27,gui.last_response
            board.drop_stop=False;send(2)
            # A latched RDR protection stops work even after the current sensor state cools.
            board.health_drop=True;board.thermal=9;board.protected=True
            # DE02 protection flag must block immediately without waiting for DE08.
            wait_for(lambda:gui.command(1001,target=target)==26)
            board.health_drop=False
            wait_for(lambda:health(9))
            send(1001,26)
            board.thermal=3;wait_for(lambda:health(3))
            assert gui.last_response[8]==1
            send(1001,26)
            # Expired telemetry is explicitly unknown; it is not normal temperature.
            board.health_drop=True
            wait_for(lambda:gui.command(20)==0 and gui.last_response[7]==0,5)
            assert gui.last_response[6]==255
            assert board.error is None,board.error
            gui.close();gui=None;proc.stdin.write('quit\n');proc.stdin.flush();proc.wait(timeout=10)
            assert proc.returncode==0,proc.stderr.read()
            print('PASS thermal warning',thermal,'STOP/capture/resume, failure cause, high-error latch, expiry',flush=True)
    finally:
        if gui:gui.close()
        if proc and proc.poll() is None:proc.kill();proc.wait()
        board.close()
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--console',type=Path,required=True);a=p.parse_args()
    ctypes.windll.winmm.timeBeginPeriod(1)
    try:
        for state in (7,8):run(a.console.resolve(),state)
    finally:ctypes.windll.winmm.timeEndPeriod(1)
