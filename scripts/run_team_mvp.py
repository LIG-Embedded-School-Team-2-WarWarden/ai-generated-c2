"""Run from the repository or packaged folder; no GUI needed."""
import argparse
import json
import pathlib
import shlex
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from team_mvp.cloud import Detector
from team_mvp.engine import Engine


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config",default=str(ROOT/"configs/team-demo.json"))
    parser.add_argument("--bind",default="127.0.0.1")
    parser.add_argument("--asset-port",type=int,default=5000)
    parser.add_argument("--detector",default=str(ROOT/"build/live/Release/c2_detector_worker.exe"))
    args = parser.parse_args()
    with open(args.config,encoding="utf-8-sig") as file:
        config = json.load(file)
    engine = Engine(config,Detector(args.detector),args.bind,args.asset_port)
    print("Team ICD 1.0 console. Commands: status, targets, point 201 PAN TILT, aim 201 [object UUID], stop 201, collect start|stop, obs point PAN TILT, obs stop, obs status, events, clock-load 101 file.json, clock-profile 101 profile.json, quit",flush=True)
    try:
        while True:
            try:
                line = input("c2> ")
            except EOFError:
                break
            words = shlex.split(line,posix=False)
            if not words:
                continue
            try:
                cmd = words[0]
                if cmd in ("quit","exit"):
                    break
                if cmd in ("status","targets"):
                    state = engine.snapshot()
                    print(json.dumps(state if cmd=="status" else {k:state[k] for k in ("observation","frame","objects")},ensure_ascii=False),flush=True)
                elif cmd=="aim" and len(words) in (2,3):
                    engine.aim(words[1],words[2] if len(words)==3 else None)
                    print("OPENING: waiting for CONTROL_RESULT; continuous updates begin after confirmation.",flush=True)
                elif cmd=="point" and len(words)==4:
                    engine.point(words[1],float(words[2]),float(words[3]))
                    print("OPENING: one manual move after CONTROL_RESULT; inspect status.",flush=True)
                elif cmd=="stop" and len(words)==2:
                    engine.stop(words[1])
                    print("STOP requested; status distinguishes STOPPING and confirmed STOPPED.",flush=True)
                elif cmd=="collect" and len(words)==2 and words[1] in ("start","stop"):
                    engine.collect(words[1]=="start")
                    print("Measurement request sent; inspect status.",flush=True)
                elif cmd=="obs" and len(words)==4 and words[1]=="point":
                    engine.obs_point(float(words[2]),float(words[3]))
                    print("OBS: stop EFF/collection -> move -> confirm stopped -> collect. Inspect obs status.",flush=True)
                elif cmd=="obs" and words[1:]==["stop"]:
                    engine.obs_stop()
                    print("OBS STOP requested; inspect obs status.",flush=True)
                elif cmd=="obs" and words[1:]==["status"]:
                    state=engine.snapshot()
                    print(json.dumps(dict(device=state["devices"].get(engine.obs_id),operation=state["observer_action"],observation=state["observation"])),flush=True)
                elif cmd=="events":
                    with engine.lock:
                        print(json.dumps(list(engine.events)),flush=True)
                elif cmd=="clock-profile" and len(words)==3:
                    with open(words[2].strip('"'),encoding="utf-8-sig") as file:
                        engine.trust_acquisition_profile(words[1],json.load(file))
                    print("Verified acquisition profile bound to current boot/session; inspect targets.",flush=True)
                elif cmd=="clock-load" and len(words)==3:
                    with open(words[2].strip('"'),encoding="utf-8-sig") as file:
                        engine.trust_clock(words[1],json.load(file))
                    print("Clock evidence bound to current boot/session.",flush=True)
                else:
                    print("Unknown command or arguments.",flush=True)
            except (ValueError,KeyError,TypeError,OSError) as error:
                print("REJECTED:",error,flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        engine.close()
        print(json.dumps(engine.snapshot()),flush=True)


if __name__=="__main__":
    main()
