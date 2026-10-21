import argparse
import pathlib
import sys
import time
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from tests.dummies.peers import Peer

parser = argparse.ArgumentParser(description="Synthetic loopback OBS/EFF using team ICD 1.0")
parser.add_argument("role",choices=["OBS","EFF"])
parser.add_argument("--c2-port",type=int,default=5000)
args = parser.parse_args()
peer = Peer(args.role,("127.0.0.1",args.c2_port))
print("Synthetic "+args.role+" running; Ctrl+C to stop.",flush=True)
try:
    while True:
        time.sleep(.5)
except KeyboardInterrupt:
    pass
finally:
    peer.close()
