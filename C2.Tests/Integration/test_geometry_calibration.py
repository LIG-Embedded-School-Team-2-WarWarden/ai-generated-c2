"""3D transform transaction, persistence and both pointing paths. Loopback only."""
import argparse
import ctypes
import json
import math
from pathlib import Path
import struct
import tempfile
from test_angle_calibration import AngleProbe
from test_lidar_interlock import wait_for
from test_installation_persistence import running, ROOT
from test_station import target_records, valid_count
from test_camera_settings import read

def transform(point,t,r):
    theta=math.sqrt(sum(x*x for x in r))
    a=math.sin(theta)/theta if theta>1e-8 else 1
    b=(1-math.cos(theta))/theta**2 if theta>1e-8 else .5
    def cross(x,y): return (x[1]*y[2]-x[2]*y[1],x[2]*y[0]-x[0]*y[2],x[0]*y[1]-x[1]*y[0])
    c=cross(r,point);cc=cross(r,c)
    return tuple(point[i]+a*c[i]+b*cc[i]+t[i] for i in range(3))

def run(exe,fixture):
    fitted=json.loads(fixture.read_text())['Model']
    translation=fitted['Translation'];rotation=fitted['Rotation'];offsets=(2,-3,0)
    board=AngleProbe()
    with board.lock: board.targets=[{'id':1001,'xyz':(-1.4,2.8,1.35)}]
    try:
        with tempfile.TemporaryDirectory(prefix='geometry-calibration-') as directory:
            runtime=Path(directory);config=json.loads((ROOT/'configs/team-demo.json').read_text())
            for name,port in [('rdr',board.rdr_port),('fcs',board.fcs_port)]:
                config[name]['local']={'ip':'127.0.0.1','port':0};config[name]['peer']={'ip':'127.0.0.1','port':port}
            config['rdr']['gui_kalman']={'enabled':False};path=runtime/'config.json';path.write_text(json.dumps(config))
            def read_model(gui,token):
                out=[]
                for cmd in (1110,1111):
                    assert gui.command(cmd,target=token)==0,gui.last_response
                    out.append(struct.unpack('<3f',struct.pack('<3I',*gui.last_response[5:8])))
                return out,gui.last_response[8]
            def stage(gui,token,t=translation,r=rotation):
                assert gui.command(1112,target=token,xyz=t)==0,gui.last_response
                assert gui.command(1113,target=token,xyz=r)==0,gui.last_response
            with running(exe,runtime,path) as gui:
                wait_for(lambda:gui.command(6,target=1)==0)
                gui.wait(lambda f:valid_count(f)>0);wait_for(lambda:gui.command(1102)==0)
                assert gui.command(9,target=4,rdr=0,fcs=0,xyz=offsets)==0
                assert read_model(gui,10)[1]==0
                saved=path.read_bytes();count=len(board.points)
                assert gui.command(1112,target=10,xyz=translation)==0
                assert gui.command(1114,target=10,xyz=(1,0,0))!=0 # incomplete transaction
                assert path.read_bytes()==saved and len(board.points)==count
                assert gui.command(1113,target=10,xyz=rotation)==0
                assert gui.command(1114,target=11,xyz=(1,0,0))!=0 # wrong owner token
                assert path.read_bytes()==saved
                assert gui.command(1114,target=10,xyz=(1,0,0))==0
                assert len(board.points)==count and read(gui,4)==offsets
                assert json.loads(path.read_text())['installation']['camera']['fcs_geometry']['version']==2
                actual,enabled=read_model(gui,20);assert enabled==1
                for a,b in zip(actual,(translation,rotation)):assert all(abs(x-y)<1e-6 for x,y in zip(a,b))
                saved=path.read_bytes();stage(gui,20,r=(4,0,0))
                assert gui.command(1114,target=20,xyz=(1,0,0))!=0 and path.read_bytes()==saved
                assert gui.command(1112,target=20,xyz=(float('nan'),0,0))!=0
                # Once geometry is active, changing additive trim would apply a
                # second calibration. Reject it without mutating the transaction.
                read_model(gui,30);stage(gui,30)
                protected=path.read_bytes()
                assert gui.command(9,target=4,rdr=0,fcs=0,xyz=(3,-3,0))!=0
                assert read(gui,4)==offsets and path.read_bytes()==protected
                assert gui.command(9,target=4,rdr=0,fcs=0,xyz=offsets)==0 # idempotent write
                assert gui.command(1114,target=30,xyz=(1,0,0))==0
                for distance in (3,7,12):
                    with board.lock:board.targets=[{'id':1001,'xyz':(-distance*.2,distance,distance*.3)}]
                    rows=target_records(gui.wait(lambda f:any(abs(r[4]+.2*distance)<.2 for r in target_records(f))))
                    target=rows[0][0]
                    for manual in (False,True):
                        point=next(r[4:7] for r in target_records(gui.frame()) if r[0]==target)
                        q=transform(point,translation,rotation)
                        expected=(round((math.degrees(math.atan2(q[2],math.hypot(q[0],q[1])))+offsets[1])*10),round((180+math.degrees(math.atan2(q[0],q[1]))+offsets[0])*10)%3600)
                        count=len(board.points)
                        assert gui.command(1001 if manual else 1,target=target)==0,gui.last_response
                        if manual:assert gui.command(1003,target=target)==0,gui.last_response
                        assert len(board.points)==count+1 and all(abs(a-b)<=1 for a,b in zip(board.points[-1],expected)),(board.points[-1],expected,point)
                        def aligned():
                            gui.command(0);return gui.last_response[7]==(103 if manual else 2)
                        wait_for(aligned)
                        read_model(gui,40)
                        assert gui.command(1112,target=40,xyz=translation)!=0 # active assignment
                        assert gui.command(2,target=target)==0
                        wait_for(lambda:gui.command(1102)==0)
                before=read_model(gui,50)
            with running(exe,runtime,path) as gui:
                wait_for(lambda:gui.command(1102)==0)
                assert read_model(gui,60)==before # restart persists
                # Restore/disable is also atomic and never moves hardware.
                count=len(board.points);stage(gui,60)
                assert gui.command(1114,target=60,xyz=(0,0,0))==0
                assert read_model(gui,70)[1]==0 and len(board.points)==count
                stage(gui,70);assert gui.command(1114,target=70,xyz=(1,0,0))==0
                wait_for(lambda:gui.command(6,target=1)==0)
                rows=target_records(gui.wait(lambda f:valid_count(f)>0));target=rows[0][0]
                def aim():
                    count=len(board.points);assert gui.command(1,target=target)==0,gui.last_response
                    assert len(board.points)==count+1
                    wait_for(lambda:gui.command(0)==0 and gui.last_response[7]==2)
                    value=board.points[-1]
                    assert gui.command(2,target=target)==0
                    wait_for(lambda:gui.command(1102)==0)
                    return value
                before_install=aim()
                assert gui.command(5,rdr=1,fcs=0,xyz=(3,-2,1))==0
                assert gui.command(5,rdr=0,fcs=1,xyz=(-4,5,-1))==0
                assert read_model(gui,80)[1]==1 # absolute installation never disables RDR-local geometry
                after_install=aim()
                assert after_install==before_install,(before_install,after_install)
            assert board.error is None,board.error
        print('PASS: RDR-local 3D geometry across distances and pointing paths, v2 persistence, atomic commit, trim double-apply protection, restart/disable, installation independence')
    finally:board.close()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--console',type=Path,required=True)
    parser.add_argument('--fixture',type=Path,default=ROOT/'C2.Tests/Fixtures/fcs_geometry.json');args=parser.parse_args()
    ctypes.windll.winmm.timeBeginPeriod(1)
    try:run(args.console.resolve(),args.fixture)
    finally:ctypes.windll.winmm.timeEndPeriod(1)
