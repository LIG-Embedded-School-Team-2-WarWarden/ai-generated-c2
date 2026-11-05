"""Custom world-target dummy through the real C2 UDP/GUI IPC path."""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
from test_station import Gui, FcsBoard, target_records, valid_count
from test_simulator import eventually

ROOT=Path(__file__).resolve().parents[2]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--console',type=Path,default=ROOT/'x64/Release/C2.Console.exe')
    parser.add_argument('--external-fcs',action='store_true',help='test real-FCS routing against an independent loopback board')
    parser.add_argument('--unit-north',action='store_true',help='reproduce requested world point (0,1,0)')
    args=parser.parse_args()
    initial_xyz=[0,1,0] if args.unit_north else [1,3,.5]
    initial_pan=180 if args.unit_north else 198.4
    with tempfile.TemporaryDirectory(prefix='c2-target-dummy-') as directory:
        runtime=Path(directory)
        external=FcsBoard() if args.external_fcs else None
        arguments=[sys.executable,str(ROOT/'simulators/run.py'),'--headless','--isolated-ipc',
            '--console',str(args.console.resolve()),'--runtime',str(runtime),'--target','7',*[str(v) for v in initial_xyz]]
        if external:
            source=json.loads((ROOT/'configs/team-demo.json').read_text(encoding='utf-8'))
            source['fcs']['peer']['port']=external.port
            source['fcs']['enabled']=True
            source_file=runtime/'source.json';source_file.write_text(json.dumps(source),encoding='utf-8')
            arguments += ['--real-fcs','--fcs-config',str(source_file)]
        process=subprocess.Popen(arguments,
            stdout=subprocess.PIPE,stderr=subprocess.PIPE,creationflags=subprocess.CREATE_NO_WINDOW)
        gui=None
        try:
            def state():
                try:return json.loads((runtime/'simulator-state.json').read_text(encoding='utf-8'))
                except (OSError,json.JSONDecodeError):return {}
            ready=eventually(state,lambda s:s.get('rdr_connected'))
            if external:
                assert ready['fcs_mode']=='real' and not ready['mock_fcs_enabled']
                assert ready['received']['FCS']==0 and ready['fcs_azimuth_deg'] is None
                assert json.loads((runtime/'config.json').read_text(encoding='utf-8'))['fcs']==source['fcs']
            def pan(s):return external.azimuth/10 if external else s.get('fcs_azimuth_deg',-999)
            gui=Gui(ready['ipc_suffix'])
            assert gui.command(6,target=1)==0
            def at(xyz):
                def check(raw):
                    rows=target_records(raw)
                    return len(rows)==1 and rows[0][0]==7 and rows[0][9] and all(
                        abs(a-b)<1e-5 for a,b in zip(rows[0][4:7],xyz))
                return check
            first=gui.wait(at(initial_xyz))
            first_pose=struct.unpack_from('<2f',first,48)
            gui.wait(lambda raw:at(initial_xyz)(raw) and
                abs(struct.unpack_from('<f',raw,48)[0]-first_pose[0])>.2)
            assert gui.command(1,target=7)==0
            eventually(state,lambda s:abs(pan(s)-initial_pan)<.11)
            assert gui.command(2,target=7)==0
            # GUI installation edits must not shift a target entered in world
            # coordinates. The dummy follows ECS's RDR origin automatically.
            assert gui.command(5,rdr=1,fcs=0,xyz=(.5,0,0))==0
            gui.wait(lambda raw:valid_count(raw)==0)
            restored=gui.wait(at(initial_xyz))
            synced=eventually(state,lambda s:s.get('rdr_world')==[.5,0,0])
            assert synced['targets'][0]['xyz']==initial_xyz
            print(json.dumps({'input_world_xyz':initial_xyz,'ecs_rdr_world_xyz':[.5,0,0],
                'synchronized_dummy_rdr_world_xyz':synced['rdr_world'],
                'actual_gui_target_xyz':next(t[4:7] for t in target_records(restored) if t[0]==7)}))
            # Updating an existing ID emits LOST before its new observation;
            # this resets C2's fixed-world cache without restarting C2.
            control={'scenario':'normal','target_count':1,'targets':[{'id':7,'xyz':[-1,3,.75]}]}
            temporary=runtime/'control.update'
            temporary.write_text(json.dumps(control),encoding='utf-8')
            temporary.replace(runtime/'control.json')
            gui.wait(lambda raw:valid_count(raw)==0)
            gui.wait(at((-1,3,.75)))
            assert gui.command(1,target=7)==0
            eventually(state,lambda s:abs(pan(s)-161.6)<.11)
            assert gui.command(2,target=7)==0
            print('PASS: selected target ID/world XYZ, mirrored lidar X, pose invariance, live reposition/reacquisition, right/left FCS commands')
            print('PASS: GUI RDR position change is synchronized; target world XYZ stays unchanged')
            if external:
                assert external.error is None
                assert any(p[:2]==b'\xef\x03' for p in external.wire)
                print('PASS: lidar-only dummy, original FCS config preserved, no mock FCS socket, EF03 delivered to external board')
        finally:
            if gui:gui.close()
            (runtime/'stop.txt').touch()
            try:out,error=process.communicate(timeout=12)
            except subprocess.TimeoutExpired:
                process.kill();out,error=process.communicate();raise AssertionError('dummy did not stop')
            if external:external.close()
            assert process.returncode==0,(out,error)

if __name__=='__main__':main()
