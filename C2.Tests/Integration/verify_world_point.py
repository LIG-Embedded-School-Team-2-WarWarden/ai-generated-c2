"""Read ECS computed XYZ and actual GUI shared-memory XYZ for one dummy point."""
import argparse
import hashlib
import json
from pathlib import Path
import queue
import struct
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from test_station import Gui, target_records

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'simulators'))
from devices import Devices
from run import station_config

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--console',type=Path,default=ROOT/'x64/Release/C2.Console.exe')
    parser.add_argument('--xyz',type=float,nargs=3,default=[0,1,0])
    parser.add_argument('--report',type=Path,default=ROOT/'logs/world-point-verification.json')
    args=parser.parse_args()
    devices=Devices()
    process=gui=None
    try:
        devices.configure(targets=[{'id':7,'xyz':args.xyz}])
        with tempfile.TemporaryDirectory(prefix='c2-world-point-') as directory:
            config_path=Path(directory)/'config.json'
            config_path.write_text(json.dumps(station_config(devices.rdr_port,devices.fcs_port)),encoding='utf-8')
            suffix='_verify_'+uuid.uuid4().hex
            process=subprocess.Popen([str(args.console.resolve()),'--config',str(config_path),'--ipc-suffix',suffix],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,
                creationflags=subprocess.CREATE_NO_WINDOW)
            output=queue.Queue()
            def reader():
                for line in process.stdout:
                    try:output.put(json.loads(line))
                    except json.JSONDecodeError:pass
            threading.Thread(target=reader,daemon=True).start()
            assert output.get(timeout=8)['ready']
            def ask(command):
                process.stdin.write(command+'\n');process.stdin.flush()
                return output.get(timeout=8)
            deadline=time.monotonic()+8
            while not ask('rdr status')['connected']:
                assert time.monotonic()<deadline,'RDR did not connect'
                time.sleep(.05)
            gui=Gui(suffix)
            assert gui.command(6,target=1)==0
            def valid(raw):return any(t[0]==7 and t[9] for t in target_records(raw))
            gui.wait(valid)
            samples=[]
            for _ in range(3):
                rows=ask('rdr targets')
                computed=next(t for t in rows if t['id']==7)
                raw=gui.wait(valid)
                record=next(t for t in target_records(raw) if t[0]==7)
                actual=list(record[4:7])
                assert actual==computed['gui_xyz'],(actual,computed)
                assert max(abs(a-b) for a,b in zip(actual,args.xyz))<1e-6
                assert max(abs(a-b) for a,b in zip(computed['world_xyz'],args.xyz))<1e-6
                samples.append({'frame_sequence':struct.unpack_from('<I',raw,12)[0],
                    'timestamp_ms':struct.unpack_from('<Q',raw,16)[0],
                    'observation_pan_tilt_wire':[computed['observation_pan'],computed['observation_tilt']],
                    'raw_xyz':[computed[k] for k in ('raw_x','raw_y','raw_z')],
                    'ecs_world_xyz':computed['world_xyz'],'ecs_gui_xyz':computed['gui_xyz'],
                    'actual_gui_shared_memory_xyz':actual})
                time.sleep(.6)
            assert samples[0]['observation_pan_tilt_wire']!=samples[-1]['observation_pan_tilt_wire']
            assert all(s['actual_gui_shared_memory_xyz']==samples[0]['actual_gui_shared_memory_xyz'] for s in samples)
            report={'input_world_xyz':args.xyz,'rdr_world_xyz':[0,0,0],
                'console':str(args.console.resolve()),'console_sha256':hashlib.sha256(args.console.read_bytes()).hexdigest(),
                'samples':samples,'max_world_error_m':max(abs(a-b) for s in samples for a,b in zip(s['ecs_world_xyz'],args.xyz)),
                'max_gui_error_m':max(abs(a-b) for s in samples for a,b in zip(s['actual_gui_shared_memory_xyz'],args.xyz)),
                'computed_gui_matches_actual_shared_memory':True,'world_position_held_across_pose_changes':True}
            args.report.parent.mkdir(parents=True,exist_ok=True)
            args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
            print(json.dumps(report,ensure_ascii=False,indent=2))
            ask('rdr stop')
            process.stdin.write('quit\n');process.stdin.flush()
            process.wait(timeout=10)
            assert process.returncode==0,process.stderr.read()
    finally:
        if gui:gui.close()
        if process and process.poll() is None:
            process.kill();process.wait(timeout=5)
        devices.close()

if __name__=='__main__':main()
