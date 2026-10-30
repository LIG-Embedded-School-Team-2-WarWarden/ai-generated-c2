"""Optional loopback equipment double; the C2 runtime itself is native C++."""
import argparse
import pathlib
import sys
import time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from tests.dummies.gui_devices import GuiObserver, GuiEffector

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("role", choices=["OBS", "EFF"])
parser.add_argument("--c2-port", type=int, default=5000)
parser.add_argument("--fixed-view", action="store_true")
args = parser.parse_args()
address = ("127.0.0.1", args.c2_port)
peer = GuiObserver(address, sweep=not args.fixed_view) if args.role == "OBS" else GuiEffector(address)
print("Loopback fake", args.role, "ready", flush=True)
try:
    while True:
        time.sleep(.5)
except KeyboardInterrupt:
    pass
finally:
    peer.close()
