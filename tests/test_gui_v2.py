"""Native C++ process + real UDP doubles + unmodified GitHub GUI C# IPC sources."""
import argparse
import json
import pathlib
import subprocess
import sys
import time
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tests.dummies.gui_devices import GuiObserver, GuiEffector

parser = argparse.ArgumentParser()
parser.add_argument("--console", default=str(ROOT / "build/gui-v2/Release/c2_mvp_console.exe"))
parser.add_argument("--probe", default=str(ROOT / "build/gui-probe/GuiProbe.dll"))
args, remaining = parser.parse_known_args()


class GuiV2(unittest.TestCase):
    def test_native_roundtrip(self):
        stop = ROOT / "build/gui-v2-test.stop"
        stop.unlink(missing_ok=True)
        c2 = subprocess.Popen([args.console, "--config", str(ROOT / "configs/team-demo.json"),
            "--asset-port", "0", "--gui", "--headless", "--stop-file", str(stop)],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        obs = eff = probe = None
        try:
            ready = json.loads(c2.stdout.readline())
            self.assertEqual(ready["runtime"], "C++")
            self.assertEqual(ready["gui_shared_memory_version"], 2)
            address = ("127.0.0.1", ready["port"])
            obs, eff = GuiObserver(address), GuiEffector(address)
            probe = subprocess.Popen(["dotnet", args.probe], stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.assertTrue(json.loads(probe.stdout.readline())["ready"])

            def query(line):
                probe.stdin.write(line + "\n"); probe.stdin.flush()
                value = probe.stdout.readline()
                if not value:
                    raise AssertionError(probe.stderr.read())
                return json.loads(value)

            def frame():
                return query("frame")["payload"]

            def wait(predicate, timeout=10):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    value = predicate()
                    if value:
                        return value
                    time.sleep(.05)
                raise AssertionError("GUI condition timed out")

            def command(code, target=0):
                return query(json.dumps(dict(command=code, target=target)))

            p = wait(lambda: (p if p["targetCount"] and p["fcsAssets"][0]["isEnabled"] else None)
                     if (p := frame()) else None)
            self.assertEqual(p["rdrAssets"][0]["azimuthDeg"], 135)
            self.assertAlmostEqual(p["fcsAssets"][0]["positionX"], -.1, places=6)
            t = p["targets"][0]
            self.assertAlmostEqual(t["lidarX"], -.2, delta=.03)
            self.assertAlmostEqual(t["lidarY"], 3, places=5)
            target = t["targetId"]

            # Exact boundary values traverse UDP, C++ conversion, shared memory,
            # checksum validation and the actual C# reader from pinned GUI main.
            with obs.lock:
                obs.frames_enabled = False
            for pan, gui_angle in ((-135,270),(-90,225),(0,135),(90,45),(135,0),(0,135)):
                with obs.lock:
                    obs.pan = pan
                    obs.status()
                wait(lambda: frame()["rdrAssets"][0]["azimuthDeg"] == gui_angle)
            with obs.lock:
                obs.frames_enabled = True
            wait(lambda: frame()["targetCount"] > 0)
            target = frame()["targets"][0]["targetId"]
            wait(lambda: command(1, target)["ResultCode"] == 0)
            wait(lambda: command(0)["AssignmentState"] == 2)
            ready_fcs = frame()["fcsAssets"][0]
            self.assertEqual(ready_fcs["isOperating"], 1, "GUI v2 requires operating even for a stopped/ready motor")
            self.assertEqual(command(3, target)["ResultCode"], 0)
            wait(lambda: command(0)["FireState"] == 1)
            wait(lambda: frame()["fcsAssets"][0]["laserOn"] == 1)
            self.assertEqual(eff.laser_commands, 1)
            self.assertEqual(command(3, target)["ResultCode"], 0)
            self.assertEqual(eff.laser_commands, 1)
            wait(lambda: frame()["fcsAssets"][0]["laserOn"] == 0)
            self.assertTrue(all(t["status"] != 1 for t in frame()["targets"]), "laser ON must not fabricate HIT")
            self.assertEqual(command(2, target)["ResultCode"], 0)
            wait(lambda: command(0)["AssignmentState"] == 0)
            wait(lambda: frame()["targetCount"] > 0)
            target = frame()["targets"][0]["targetId"]
            wait(lambda: command(1, target)["ResultCode"] == 0)
            wait(lambda: command(0)["AssignmentState"] == 2)
            eff.drop_laser_results = True
            self.assertEqual(command(3, target)["ResultCode"], 0)
            failed = wait(lambda: (s if s["AssignmentState"] == 3 else None)
                          if (s := command(0)) else None)
            self.assertEqual(failed["FireState"], 0, "unconfirmed ON must not report completion")
            eff.drop_laser_results = False
            self.assertEqual(command(2, target)["ResultCode"], 0)
            wait(lambda: command(0)["AssignmentState"] == 0)
            probe.stdin.write("quit\n"); probe.stdin.flush(); probe.wait(timeout=5)
            for stream in (probe.stdin, probe.stdout, probe.stderr): stream.close()
            probe = subprocess.Popen(["dotnet", args.probe], stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.assertTrue(json.loads(probe.stdout.readline())["ready"])
            self.assertEqual(command(0)["ResultCode"], 0, "pipe reconnect")
            self.assertEqual(command(99)["ResultCode"], 5)
            self.assertEqual(command(4, 91)["ResultCode"], 1)
            self.assertEqual(command(4, 15)["ResultCode"], 0)
            wait(lambda: frame()["rdrAssets"][0]["elevationDeg"] == 15)
            eff.close()
            wait(lambda: frame()["fcsAssets"][0]["isConnected"] == 0)
            self.assertEqual(command(0)["IsConnected"], 0)
            print("PASS: C++ runtime, GUI v2 C# reader, six angles, map coordinates, assignment, laser ON/OFF, missing ACK, cancel, elevation, reconnect, device loss")
        finally:
            if probe:
                if probe.poll() is None:
                    try:
                        probe.stdin.write("quit\n"); probe.stdin.flush()
                    except (OSError, BrokenPipeError):
                        pass
                try:
                    probe.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    probe.kill(); probe.wait()
                for stream in (probe.stdin, probe.stdout, probe.stderr):
                    stream.close()
            stop.write_text("stop")
            try:
                c2.wait(timeout=8)
            finally:
                if c2.poll() is None:
                    c2.kill(); c2.wait()
                if obs: obs.close()
                if eff: eff.close()
                stop.unlink(missing_ok=True)
            error = c2.stderr.read()
            c2.stdout.close(); c2.stderr.close()
            self.assertEqual(c2.returncode, 0, error)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0], *remaining])
