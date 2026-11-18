"""Calibration IPC regression against loopback devices only."""
import argparse
import ctypes
import json
from pathlib import Path
import struct
import tempfile
from test_angle_calibration import AngleProbe
from test_lidar_interlock import wait_for
from test_installation_persistence import running, ROOT
from test_station import target_records, valid_count
from test_camera_settings import read

class SignedAngleProbe(AngleProbe):
    def _fcs_request(self,raw,peer,now):
        if raw[:2]==b'\xef\x03':
            tilt,pan,padding=struct.unpack_from('!hhH',raw,6)
            if -300<=tilt<0 and 1200<=pan<=2400 and padding==0:
                self.points.append((tilt,pan))
                self.target_elevation,self.target_azimuth=tilt/10,pan/10
                self.fcs_tx=(self.fcs_tx+1)&65535
                self.fcs.sendto(struct.pack('!4H',0xFE03,self.fcs_tx,struct.unpack_from('!H',raw,2)[0],0),peer)
                return
        super()._fcs_request(raw,peer,now)

def run(exe):
    board=SignedAngleProbe()
    with board.lock: board.targets=[{"id":1001,"xyz":(-1.4,2.8,1.35)}]
    try:
        with tempfile.TemporaryDirectory(prefix='calibration-mode-') as folder:
            runtime=Path(folder)
            config=json.loads((ROOT/'configs/team-demo.json').read_text())
            for name,port in [('rdr',board.rdr_port),('fcs',board.fcs_port)]:
                config[name]['local']={'ip':'127.0.0.1','port':0}
                config[name]['peer']={'ip':'127.0.0.1','port':port}
            config['rdr']['gui_kalman']={'enabled':False}
            path=runtime/'config.json';path.write_text(json.dumps(config))
            with running(exe,runtime,path) as gui:
                wait_for(lambda:gui.command(6,target=1)==0)
                rows=target_records(gui.wait(lambda f:valid_count(f)>0));target=rows[0][0]
                wait_for(lambda:gui.command(1102)==0)
                def angles(command,target=0):
                    assert gui.command(command,target=target)==0,gui.last_response
                    return struct.unpack('<2f',struct.pack('<2I',*gui.last_response[5:7]))
                original=angles(1101,target)
                assert gui.command(9,target=4,rdr=0,fcs=0,xyz=(5,6,0))==0
                assert angles(1101,target)==original # reference never includes existing offset
                pan,tilt=angles(1102)
                count=len(board.points)
                for xyz in [(float('nan'),0,0),(pan+30.1,tilt,0),(pan,tilt+30.1,0),(pan,-31,0),(pan,91,0),(pan,tilt,1)]:
                    assert gui.command(1100,xyz=xyz)==1,(xyz,gui.last_response)
                assert gui.command(1100,target=target,xyz=(pan,tilt,0))==1
                assert len(board.points)==count
                assert gui.command(1100,xyz=((pan+1)%360, min(90,tilt+1),0))==0,gui.last_response
                assert len(board.points)==count+1
                wait_for(lambda:gui.command(1102)==0)
                actual=angles(1102)
                assert abs(actual[0]-(pan+1)%360)<.11 and abs(actual[1]-min(90,tilt+1))<.11,actual
                # One large key press is one absolute command, not queued small moves.
                for step in (5,10,30):
                    for sign in (1,-1):
                        p,t=angles(1102);count=len(board.points)
                        assert gui.command(1100,xyz=(p+sign*step,t,0))==0,gui.last_response
                        assert len(board.points)==count+1
                        wait_for(lambda:gui.command(1102)==0)
                        reached=angles(1102)
                        assert abs(reached[0]-(p+sign*step))<.11 and abs(reached[1]-t)<.11,reached
                p,t=angles(1102)
                assert gui.command(1100,xyz=(p,t+30,0))==0
                wait_for(lambda:gui.command(1102)==0)
                assert abs(angles(1102)[1]-t-30)<.11
                assert gui.command(1101,target=65535)!=0
                assert gui.command(1001,target=target)==0
                assert gui.command(1100,xyz=(actual[0],actual[1],0))!=0
                assert gui.command(1102)!=0 # sampling while an assignment owns the device is disallowed
                assert gui.command(2,target=target)==0
                wait_for(lambda:gui.command(1102)==0)
                count=len(board.points)
                assert gui.command(9,target=4,rdr=0,fcs=0,xyz=(2,3,0))==0
                assert read(gui,4)==(2,3,0) and len(board.points)==count
                # Signed actual tilt must remain unchanged for horizontal nudges.
                with board.lock:
                    board.elevation=board.target_elevation=-1.9
                wait_for(lambda:gui.command(1102)==0 and abs(struct.unpack('<f',struct.pack('<I',gui.last_response[6]))[0]+1.9)<.01)
                pan,tilt=angles(1102)
                assert gui.command(1100,xyz=(pan+.1,tilt,0))==0,gui.last_response
                wait_for(lambda:gui.command(1102)==0)
                p,t=angles(1102)
                assert abs(p-pan-.1)<.11 and abs(t+1.9)<.01,(p,t)
            assert board.error is None,board.error
        print('PASS: calibration baseline independence, raw feedback, bounded motion, malformed command rejection, assignment exclusion, save without motion')
    finally:board.close()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--console',type=Path,required=True);args=parser.parse_args()
    ctypes.windll.winmm.timeBeginPeriod(1)
    try:run(args.console.resolve())
    finally:ctypes.windll.winmm.timeEndPeriod(1)
