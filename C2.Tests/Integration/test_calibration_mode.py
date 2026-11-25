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
    def __init__(self):
        # Command 1103 is the sole low-power overlap exception. The base probe
        # keeps enforcing physical RDR inhibition for every other FCS output.
        self.allow_lidar_overlap=True
        super().__init__()

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
                rdr_commands=sum(event[0]=='rdr' for event in board.events)
                assert board.mode==2 and board.reporting and not board.lidar_inhibited
                assert gui.command(1103)==0,gui.last_response
                assert board.mode==2 and board.reporting and not board.lidar_inhibited
                assert any(e[:2]==('fcs',1) and not e[2] for e in board.events[pulse_events:]),board.events[pulse_events:]
                assert sum(event[0]=='rdr' for event in board.events)==rdr_commands,board.events[pulse_events:]
                # An active calibration lease permits rapid pan/tilt commands and
                # read-only feedback without extinguishing the alignment indication.
                motion_pan,motion_tilt=board.azimuth,board.elevation
                motion_goals=[((motion_pan+step)%360,motion_tilt) for step in (5,10,15)]
                point_count=len(board.points);move_off_requests=board.off_requests
                for goal_pan,goal_tilt in motion_goals:
                    assert gui.command(1100,xyz=(goal_pan,goal_tilt,0))==0,(goal_pan,gui.last_response)
                assert len(board.points)==point_count+3,board.points[point_count:]
                wait_for(lambda:gui.command(1102)==0 and
                    abs(board.azimuth-board.target_azimuth)>.05,timeout=1)
                assert board.laser_until>time.monotonic()
                moving_ons=sum(e[:2]==('fcs',1) for e in board.events)
                assert gui.command(1103)==0,gui.last_response # Renewal is allowed while moving.
                assert sum(e[:2]==('fcs',1) for e in board.events)==moving_ons+1
                assert board.laser_until>time.monotonic() and board.off_requests==move_off_requests
                assert not any(event[0]=='rdr' for event in board.events[pulse_events:]),board.events[pulse_events:]
                # The GUI can keep the indication continuously visible by renewing
                # the same bounded lease while RDR surveillance remains live. Neither
                # movement, renewal nor OFF may issue any ED07 command.
                fcs_ons=sum(e[:2]==('fcs',1) for e in board.events)
                time.sleep(3.2)
                assert board.mode==2 and board.reporting and not board.lidar_inhibited
                assert valid_count(gui.wait(lambda frame:valid_count(frame)>0))>0
                assert gui.command(1103)==0,gui.last_response
                assert sum(event[0]=='rdr' for event in board.events)==rdr_commands,board.events[pulse_events:]
                assert sum(e[:2]==('fcs',1) for e in board.events)==fcs_ons+1,board.events[pulse_events:]
                time.sleep(2.2)
                assert board.mode==2 and board.reporting and not board.lidar_inhibited
                assert valid_count(gui.wait(lambda frame:valid_count(frame)>0))>0
                assert board.off_requests==off_requests,board.events[pulse_events:]
                wait_for(lambda:board.off_requests>off_requests,timeout=9)
                relevant=[e[:2] for e in board.events[pulse_events:]]
                assert not any(event[0]=='rdr' for event in board.events[pulse_events:]),board.events[pulse_events:]
                assert relevant.index(('fcs',1)) < relevant.index(('fcs',0)),relevant
                assert board.mode==2 and board.reporting and not board.lidar_inhibited
                assert gui.command(1104)==0,gui.last_response # OFF is idempotent after automatic expiry.
                assert sum(event[0]=='rdr' for event in board.events)==rdr_commands,board.events[pulse_events:]
                wait_for(lambda:gui.command(1102)==0)

                # Optional ED07 state can be unknown while RDR mode 2 reporting
                # remains live. That transient diagnostic gap must not suppress
                # the separate low-power calibration indication.
                board.boot_unknown=True
                wait_for(lambda:gui.command(1103)==0,timeout=4)
                assert board.mode==2 and board.reporting and not board.lidar_inhibited
                assert gui.command(1104)==0,gui.last_response
                board.boot_unknown=False
                wait_for(lambda:gui.command(1102)==0)

                # A lost ON reply has an unknown output state. The command must fail,
                # issue best-effort OFF immediately and leave the optical interlock recoverable.
                failure_events=len(board.events);board.drop_on_reply=True;off_requests=board.off_requests
                assert gui.command(1103)!=0,gui.last_response
                board.drop_on_reply=False
                wait_for(lambda:board.off_requests>off_requests and board.laser_until<=time.monotonic(),timeout=6)
                assert not any(event[0]=='rdr' for event in board.events[failure_events:]),board.events[failure_events:]
                assert board.mode==2 and board.reporting and not board.lidar_inhibited

                # Losing the live RDR condition while an overlap lease is active
                # fails closed immediately at FCS without trying to toggle ED07.
                invariant_events=len(board.events);off_requests=board.off_requests
                assert gui.command(1103)==0,gui.last_response
                with board.lock: board.reporting=False
                wait_for(lambda:board.off_requests>off_requests and board.laser_until<=time.monotonic(),timeout=3)
                assert not any(event[0]=='rdr' for event in board.events[invariant_events:]),board.events[invariant_events:]
                with board.lock: board.reporting=True
                wait_for(lambda:gui.command(1103)==0,timeout=4)
                assert gui.command(1104)==0,gui.last_response
                assert not any(event[0]=='rdr' for event in board.events[invariant_events:]),board.events[invariant_events:]
                assert board.mode==2 and board.reporting and not board.lidar_inhibited
                # Keep the remaining bounded-movement checks away from the mock's
                # deliberate 120..240 degree EF03 acceptance edge. The overlap
                # movement above already exercised three live commands.
                with board.lock:
                    board.azimuth=board.target_azimuth=180
                    board.elevation=board.target_elevation=10
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
                laser_ons=sum(event[:2]==('fcs',1) for event in board.events)
                assert gui.command(1103)!=0 # An accepted-but-not-arrived move is not stationary.
                assert sum(event[:2]==('fcs',1) for event in board.events)==laser_ons
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
                        assert gui.command(1100,xyz=(p+sign*step,t,0))==0,(step,sign,p,t,gui.last_response)
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
                assert board.mode==2 and board.reporting and not board.lidar_inhibited
                shutdown_events=len(board.events);off_requests=board.off_requests
                board.drop_next_off=True
                (runtime/'stop').touch()
                wait_for(lambda:board.off_requests>=off_requests+2 and board.laser_until<=time.monotonic(),timeout=2)
                wait_for(lambda:any(event[:2]==('rdr_start',0) for event in board.events[shutdown_events:]),timeout=2)
                shutdown=board.events[shutdown_events:]
                off_indices=[i for i,event in enumerate(shutdown) if event[:2]==('fcs',0)]
                rdr_stop=next(i for i,event in enumerate(shutdown) if event[:2]==('rdr_start',0))
                assert len(off_indices)>=2 and max(off_indices)<rdr_stop,shutdown
                assert not any(event[0]=='rdr' for event in shutdown),shutdown
            assert board.error is None,board.error
        print('PASS: calibration laser/RDR overlap lease, renewal/auto-off/failure/shutdown recovery, baseline independence, raw feedback, bounded motion, malformed command rejection, assignment exclusion, save without motion')
    finally:board.close()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--console',type=Path,required=True);args=parser.parse_args()
    ctypes.windll.winmm.timeBeginPeriod(1)
    try:run(args.console.resolve())
    finally:ctypes.windll.winmm.timeEndPeriod(1)
