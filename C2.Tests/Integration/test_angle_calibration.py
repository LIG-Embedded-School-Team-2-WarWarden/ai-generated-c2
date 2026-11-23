"""Independent loopback checks for both aiming paths; never uses physical devices."""
import argparse
import ctypes
import json
import math
from pathlib import Path
import struct
import tempfile
import time
from test_lidar_interlock import Probe, wait_for
from test_installation_persistence import running, ROOT
from test_station import target_records, valid_count
from test_camera_settings import read


class AngleProbe(Probe):
    def __init__(self):
        self.points = []
        super().__init__()

    def _fcs_request(self, raw, peer, now):
        if raw[:2] == b'\xef\x03':
            self.points.append(struct.unpack_from('!hh', raw, 6))
        super()._fcs_request(raw, peer, now)


def run(exe):
    board = AngleProbe()
    with board.lock: board.targets = [{"id":1001,"xyz":(-1.4,2.8,1.35)}]
    try:
        with tempfile.TemporaryDirectory(prefix='angle-calibration-') as folder:
            runtime = Path(folder)
            config = json.loads((ROOT/'configs/team-demo.json').read_text())
            for name, port in [('rdr', board.rdr_port), ('fcs', board.fcs_port)]:
                config[name]['local'] = {'ip':'127.0.0.1','port':0}
                config[name]['peer'] = {'ip':'127.0.0.1','port':port}
            config['rdr']['gui_kalman'] = {'enabled':False}
            path = runtime/'config.json'
            path.write_text(json.dumps(config))
            with running(exe, runtime, path) as gui:
                wait_for(lambda: gui.command(6, target=1) == 0)
                rows = target_records(gui.wait(lambda f: valid_count(f)>0))
                target = rows[0][0]
                wait_for(lambda: gui.command(7) == 0)
                assert gui.command(5,rdr=0,fcs=1,xyz=(.1,-.2,.3)) == 0
                original = next(r[4:7] for r in rows if r[0]==target)

                def point(manual, offsets):
                    count = len(board.points)
                    xyz = next(r[4:7] for r in target_records(gui.frame()) if r[0]==target)
                    assert all(abs(a-b)<1e-5 for a,b in zip(xyz,original))
                    e,n,u = (a-b for a,b in zip(xyz,(.1,-.2,.3)))
                    expected = (round((math.degrees(math.atan2(u,math.hypot(e,n)))+offsets[1])*10),
                                round((180+math.degrees(math.atan2(e,n))+offsets[0])*10)%3600)
                    if manual:
                        assert gui.command(1001,target=target)==0
                        assert gui.command(1003,target=target)==0
                        # Changes during an active manual sequence must be rejected.
                        assert gui.command(9,target=4,rdr=0,fcs=0,xyz=(4,5,0))!=0
                    else:
                        result=gui.command(1,target=target)
                        assert result==0,(result,gui.last_response,(runtime/"events.log").read_text(errors="replace")[-7000:])
                    assert len(board.points)==count+1
                    assert board.points[-1]==expected,(board.points[-1],expected)
                    def aligned():
                        gui.command(0)
                        return gui.last_response[7] == (103 if manual else 2)
                    wait_for(aligned)
                    if manual:
                        # An already-aimed row can be corrected immediately. Marker 1
                        # identifies the GUI values as an operator absolute override.
                        count = len(board.points)
                        result = gui.command(1003,target=target,xyz=(135,10,1))
                        assert result==0,(result,gui.last_response,(runtime/"events.log").read_text(errors="replace")[-4000:])
                        assert len(board.points)==count+1 and board.points[-1]==(100,1800)
                        wait_for(aligned)
                        count = len(board.points)
                        result = gui.command(1003,target=target)
                        assert result==0,(result,gui.last_response)
                        assert len(board.points)==count+1 and board.points[-1]==expected
                        wait_for(aligned)

                for offsets in [(2.5,-3.5,0),(2.5,-3.5,0),(-4,5,0),(0,0,0)]:
                    count=len(board.points)
                    assert gui.command(9,target=4,rdr=0,fcs=0,xyz=offsets)==0
                    assert read(gui,4)==offsets
                    time.sleep(.1)
                    assert len(board.points)==count # Saving is never motion.
                    for manual in [False,True]:
                        point(manual,offsets)
                        assert gui.command(2,target=target)==0
                        wait_for(lambda: gui.command(7)==0)

                point(False,(0,0,0))
                assert gui.command(9,target=4,rdr=0,fcs=0,xyz=(1,2,0))==0
                gui.command(0)
                assert gui.last_response[7]==3 # Previous alignment invalidated.
                count=len(board.points)
                saved=json.loads(path.read_text())
                saved['installation']['camera']['fcs_angle_offset_deg']=[-2,3,0]
                path.write_text(json.dumps(saved))
                wait_for(lambda: read(gui,4)==(-2,3,0))
                assert len(board.points)==count
                assert gui.command(8,target=1,rdr=0,fcs=0)==1
            assert board.error is None,board.error
        print('PASS: automatic zero offsets, manual absolute correction/re-aim/reset, installation position, repeated save/reset, alignment, busy rejection, reload')
    finally:
        board.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--console',type=Path,required=True)
    args=parser.parse_args()
    ctypes.windll.winmm.timeBeginPeriod(1)
    try: run(args.console.resolve())
    finally: ctypes.windll.winmm.timeEndPeriod(1)
