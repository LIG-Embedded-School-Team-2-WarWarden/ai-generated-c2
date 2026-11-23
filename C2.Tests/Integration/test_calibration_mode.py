"""Calibration IPC regression against loopback devices only."""
import argparse
import ctypes
import json
from pathlib import Path
import struct
import tempfile
import time
from test_angle_calibration import AngleProbe
from test_lidar_interlock import wait_for
from test_installation_persistence import running, ROOT
from test_station import target_records, valid_count
from test_camera_settings import read

class SignedAngleProbe(AngleProbe):
    def _fcs_request(self,raw,peer,now):
        if raw[:2]==b'\xef\x04' and struct.unpack_from('!H',raw,6)[0]==0 and getattr(self,'drop_next_off',False):
            self.drop_next_off=False
            self.events.append(('fcs',0,self.lidar_inhibited));self.off_requests+=1
            return
        if raw[:2]==b'\xef\x03':
            tilt,pan,padding=struct.unpack_from('!hhH',raw,6)
            if getattr(self,'drop_next_point_reply',False) and padding==0:
                self.drop_next_point_reply=False
                self.points.append((tilt,pan));self.target_elevation,self.target_azimuth=tilt/10,pan/10
                return
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
                # Calibration output is a dedicated, bounded EF04 pulse. Malformed
                # requests must never reach either optical device.
                event_count=len(board.events)
                for kwargs in ({'target':target},{'xyz':(.1,0,0)},{'rdr':0},{'fcs':0}):
                    assert gui.command(1103,**kwargs)==1,(kwargs,gui.last_response)
                assert not any(e[:2]==('fcs',1) for e in board.events[event_count:]),board.events[event_count:]

                pulse_events=len(board.events);off_requests=board.off_requests
                assert gui.command(1103)==0,gui.last_response
                assert board.lidar_inhibited
                assert any(e[:2]==('fcs',1) and e[2] for e in board.events[pulse_events:]),board.events[pulse_events:]
                # The GUI can keep the indication continuously visible by renewing
                # the same bounded pulse. Renewal must not repeat the RDR stop handshake.
                rdr_stops=sum(e[:2]==('rdr',0) for e in board.events)
                fcs_ons=sum(e[:2]==('fcs',1) for e in board.events)
                time.sleep(3.2)
                assert gui.command(1103)==0,gui.last_response
                assert sum(e[:2]==('rdr',0) for e in board.events)==rdr_stops,board.events[pulse_events:]
                assert sum(e[:2]==('fcs',1) for e in board.events)==fcs_ons+1,board.events[pulse_events:]
                time.sleep(2.2)
                assert board.off_requests==off_requests,board.events[pulse_events:]
                wait_for(lambda:board.off_requests>off_requests and not board.lidar_inhibited,timeout=9)
                relevant=[e[:2] for e in board.events[pulse_events:]]
                assert relevant.index(('rdr',0)) < relevant.index(('fcs',1)) < relevant.index(('fcs',0)) < relevant.index(('rdr',1)),relevant
                assert gui.command(1104)==0,gui.last_response # OFF is idempotent after automatic expiry.
                wait_for(lambda:gui.command(1102)==0)

                # A lost ON reply has an unknown output state. The command must fail,
                # issue best-effort OFF immediately and leave the optical interlock recoverable.
                board.drop_on_reply=True;off_requests=board.off_requests
                assert gui.command(1103)!=0,gui.last_response
                board.drop_on_reply=False
                wait_for(lambda:board.off_requests>off_requests and board.laser_until<=time.monotonic(),timeout=6)
                wait_for(lambda:not board.lidar_inhibited,timeout=6)
                wait_for(lambda:gui.command(1101,target=target)==0)
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
                # Zeroing is interactive: successive absolute goals are accepted against
                # the last accepted goal without waiting for physical arrival or completion.
                rapid_pan,rapid_tilt=actual;count=len(board.points)
                rapid_goals=[((rapid_pan+step)%360,rapid_tilt) for step in (5,10,15)]
                for goal_pan,goal_tilt in rapid_goals:
                    assert gui.command(1100,xyz=(goal_pan,goal_tilt,0))==0,(goal_pan,gui.last_response)
                assert len(board.points)==count+len(rapid_goals),board.points[count:]
                assert gui.command(1100,xyz=((rapid_goals[-1][0]+30.1)%360,rapid_tilt,0))!=0
                assert len(board.points)==count+len(rapid_goals)
                wait_for(lambda:gui.command(1102)==0)
                reached=angles(1102)
                assert abs(reached[0]-rapid_goals[-1][0])<.11 and abs(reached[1]-rapid_tilt)<.11,reached
                # A lost EF03 reply has an unknown outcome. Recovery waits for a
                # post-command status instead of chaining from an unconfirmed goal.
                board.drop_next_point_reply=True;count=len(board.points)
                assert gui.command(1100,xyz=((reached[0]+1)%360,reached[1],0))!=0
                assert len(board.points)==count+1
                wait_for(lambda:gui.command(1102)==0)
                recovered=angles(1102)
                assert gui.command(1100,xyz=((recovered[0]+.1)%360,recovered[1],0))==0,gui.last_response
                wait_for(lambda:gui.command(1102)==0)
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
                assert gui.command(1103)!=0 # calibration output cannot overlap an assignment
                assert gui.command(1104)==0 # safety OFF remains available during any workflow
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
                # Graceful shutdown must clear an active bounded pulse before it
                # starts waiting for the RDR to stop.
                assert gui.command(1103)==0,gui.last_response
                shutdown_events=len(board.events);off_requests=board.off_requests
                board.drop_next_off=True
                (runtime/'stop').touch()
                wait_for(lambda:board.off_requests>=off_requests+2 and board.laser_until<=time.monotonic(),timeout=2)
                wait_for(lambda:any(event[:2]==('rdr_start',0) for event in board.events[shutdown_events:]),timeout=2)
                shutdown=board.events[shutdown_events:]
                off_indices=[i for i,event in enumerate(shutdown) if event[:2]==('fcs',0)]
                rdr_stop=next(i for i,event in enumerate(shutdown) if event[:2]==('rdr_start',0))
                assert len(off_indices)>=2 and max(off_indices)<rdr_stop,shutdown
            assert board.error is None,board.error
        print('PASS: calibration laser pulse/off recovery, baseline independence, raw feedback, bounded motion, malformed command rejection, assignment exclusion, save without motion')
    finally:board.close()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--console',type=Path,required=True);args=parser.parse_args()
    ctypes.windll.winmm.timeBeginPeriod(1)
    try:run(args.console.resolve())
    finally:ctypes.windll.winmm.timeEndPeriod(1)
