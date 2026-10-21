"""Black-box tests: native C++ C2 process against Python equipment doubles only."""
import argparse
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tests.dummies.contract import pb, envelope, encode, uid
from tests.dummies.peers import Peer

parser = argparse.ArgumentParser()
parser.add_argument("--console", required=True)
args, rest = parser.parse_known_args()


def wait(predicate, timeout=5):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("condition timed out")


class Console:
    def __init__(self):
        # An empty PATH demonstrates that C2 doesn't locate/spawn a Python runtime.
        env = dict(os.environ, PATH="")
        self.process = subprocess.Popen(
            [str(Path(args.console).resolve()), "--config", str(ROOT / "configs/team-demo.json"),
             "--asset-port", "0"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, encoding="utf-8", env=env)
        self.lines = queue.Queue()
        self.output = []
        def read():
            for line in self.process.stdout:
                self.output.append(line)
                self.lines.put(line)
        self.reader = threading.Thread(target=read, daemon=True)
        self.reader.start()
        started = self.read()
        assert started["runtime"] == "C++", started
        self.address = ("127.0.0.1", started["port"])

    def read(self):
        try:
            return json.loads(self.lines.get(timeout=5))
        except (queue.Empty, json.JSONDecodeError) as error:
            raise AssertionError(f"C2 response failed: {self.output!r}") from error

    def command(self, line):
        self.process.stdin.write(line + "\n")
        self.process.stdin.flush()
        return self.read()

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.write("quit\n")
            self.process.stdin.flush()
            self.process.wait(timeout=6)
        self.reader.join(timeout=1)
        self.process.stdin.close()
        self.process.stdout.close()
        assert self.process.returncode == 0, self.output


class NativeIntegration(unittest.TestCase):
    def setUp(self):
        self.c2 = Console()
        self.obs = Peer("OBS", self.c2.address)
        self.eff = Peer("EFF", self.c2.address)
        wait(self.ready)

    def tearDown(self):
        try:
            events = self.c2.command("events")
            self.assertFalse([e for e in events if e["event"] == "runtime_error"], events)
            self.c2.close()
        finally:
            self.obs.close()
            self.eff.close()
        self.assertFalse(self.obs.errors, list(self.obs.errors))
        self.assertFalse(self.eff.errors, list(self.eff.errors))

    def state(self):
        return self.c2.command("status")

    def ready(self):
        s = self.state()
        return s["devices"].get("201", {}).get("installed") and s["observation"] == "CURRENT"

    def control(self):
        return self.state()["controls"].get("201", {})

    def start(self):
        def begin():
            response = self.c2.command("aim 201")
            if "accepted" in response:
                return True
            self.assertIn("observation", response["rejected"])
            return False
        wait(begin)
        wait(lambda: self.eff.point_applications >= 1)

    def stop(self):
        self.c2.command("stop 201")
        wait(lambda: self.control().get("state") == "STOPPED")

    def test_raw_cloud_targets_continuous_aim_and_stop(self):
        targets = self.c2.command("targets")
        self.assertEqual(targets["objects"][0]["count"], 48)
        self.start()
        wait(lambda: self.eff.point_applications >= 4)
        points = [c for c in self.eff.commands if c.operation == pb.PointingCommand.OPERATION_POINT]
        self.assertGreater(len({c.source_frame_id for c in points}), 2)
        self.assertEqual(len({c.object_id for c in points}), 1)
        for command in points:
            left = .1 + .02 * math.sin(command.source_frame_id * .1)
            expected = math.degrees(math.atan2(.1, math.hypot(3, left)))
            self.assertAlmostEqual(command.tilt_deg, expected, places=3)
            self.assertEqual(command.source_connection_session_id, self.obs.session)
            self.assertTrue(command.HasField("expires_at"))
        self.stop()
        count = self.eff.point_applications
        time.sleep(.3)
        self.assertEqual(self.eff.point_applications, count)
        stop = self.eff.commands[-1]
        self.assertFalse(stop.HasField("pan_deg"))
        self.assertFalse(stop.HasField("source_frame_id"))
        self.assertGreater(stop.command_sequence, points[-1].command_sequence)

    def test_retry_keeps_exact_packet_and_applies_once(self):
        self.eff.drop_first_point_result = True
        self.start()
        def duplicated():
            seen = set()
            for key, _ in list(self.eff.received_packets):
                if key in seen:
                    return True
                seen.add(key)
            return False
        wait(duplicated)
        packets = {}
        for key, packet in list(self.eff.received_packets):
            if key in packets:
                self.assertEqual(packet, packets[key])
            packets[key] = packet
        ids = [c.request_id for c in self.eff.commands]
        self.assertEqual(len(ids), len(set(ids)))
        self.stop()

    def test_manual_point_without_observation_fields(self):
        self.assertIn("accepted", self.c2.command("point 201 20 10"))
        wait(lambda: self.control().get("reason") == "POINT_COMPLETED")
        q = self.eff.commands[-1]
        self.assertEqual((q.pan_deg, q.tilt_deg), (20, 10))
        self.assertEqual(q.origin, pb.CONTROL_MODE_MANUAL)
        self.assertFalse(q.HasField("object_id"))
        self.assertFalse(q.HasField("source_frame_id"))
        self.stop()

    def test_invalid_manual_angle_rejected(self):
        self.assertIn("rejected", self.c2.command("point 201 999 0"))
        self.assertIn("rejected", self.c2.command("point 201 nan 0"))
        self.assertEqual(self.eff.point_applications, 0)

    def test_observer_move_display_only_return_zero_and_stop(self):
        self.assertIn("accepted", self.c2.command("obs point 10 5"))
        wait(lambda: self.state()["observer_action"]["state"] == "COMPLETED")
        wait(lambda: self.state()["observation"] == "DISPLAY_ONLY")
        self.assertIn("rejected", self.c2.command("aim 201"))
        self.assertIn("accepted", self.c2.command("obs point 0 0"))
        wait(lambda: self.state()["observer_action"]["state"] == "COMPLETED")
        wait(self.ready)
        self.c2.command("obs stop")
        wait(lambda: self.state()["observer_action"]["state"] == "COMPLETED")
        self.assertFalse(self.state()["devices"]["101"]["collecting"])

    def test_observer_move_stops_active_effector(self):
        self.start()
        self.c2.command("obs point 0 0")
        wait(lambda: self.state()["observer_action"]["state"] == "COMPLETED")
        self.assertEqual(self.control()["state"], "STOPPED")
        count = self.eff.point_applications
        time.sleep(.3)
        self.assertEqual(self.eff.point_applications, count)

    def test_empty_detection_stops_without_resuming(self):
        self.start()
        self.obs.empty = True
        wait(lambda: self.control().get("state") == "STOPPED")
        self.assertEqual(self.c2.command("targets")["objects"], [])
        count = self.eff.point_applications
        self.obs.empty = False
        wait(self.ready)
        time.sleep(.3)
        self.assertEqual(self.eff.point_applications, count)

    def test_missing_clock_mapping_blocks_aim(self):
        self.obs.frame_mutator = lambda c: c.ClearField("clock_mapping_id")
        wait(lambda: self.state()["observation"] == "DISPLAY_ONLY")
        self.assertIn("rejected", self.c2.command("aim 201"))

    def test_installation_change_stops(self):
        self.start()
        with self.eff.lock:
            self.eff.revision = 2
            self.eff.status()
        wait(lambda: self.control().get("state") == "STOPPING")
        self.assertFalse(self.state()["devices"]["201"]["angles_current"])
        with self.eff.lock:
            self.eff.revision = 1
            self.eff.status()
        wait(lambda: self.control().get("state") == "STOPPED")

    def test_incomplete_new_frame_invalidates_old_target(self):
        self.start()
        with self.obs.lock:
            self.obs.frames_enabled = False
            c = pb.PointCloudChunk()
            c.CopyFrom(self.obs.last_chunks[0])
            c.frame_id += 1
            self.obs.send(c)
        wait(lambda: self.state()["observation"] == "UPDATING")
        self.assertEqual(self.c2.command("targets")["objects"], [])
        wait(lambda: self.control().get("state") == "STOPPED")

    def test_conflicting_chunk_stops_and_old_session_rejected(self):
        self.start()
        with self.obs.lock:
            self.obs.frames_enabled = False
            c = pb.PointCloudChunk()
            c.CopyFrom(self.obs.last_chunks[0])
            c.points[0].sdk_depth_raw += 1
            self.obs.send(c)
        wait(lambda: self.control().get("state") == "STOPPED")
        self.assertEqual(self.state()["observation"], "FRAME_INVALID")
        self.obs.sock.sendto(encode(envelope("101", "C2", c, uid())), self.c2.address)
        wait(lambda: any(e.get("reason") == "unknown session/source" for e in self.c2.command("events")))

    def test_stop_result_required_for_confirmation(self):
        self.start()
        self.eff.drop_stop_results = True
        self.c2.command("stop 201")
        wait(lambda: self.control().get("reason") == "STOP_UNCONFIRMED")
        self.assertEqual(self.control()["state"], "STOPPING")
        self.eff.drop_stop_results = False
        self.stop()

    def test_accepted_does_not_mean_completed(self):
        self.eff.accept_only = True
        self.start()
        wait(lambda: self.control().get("state") == "STOPPED")
        self.assertEqual(self.eff.point_applications, 1)
        self.assertEqual(self.control()["reason"], "POINT_UNCONFIRMED")

    def test_observer_restart_stops_without_resuming(self):
        self.start()
        old_session = self.obs.session
        self.obs.close()
        self.obs = Peer("OBS", self.c2.address)
        wait(lambda: self.obs.session and self.obs.session != old_session)
        wait(lambda: self.control().get("state") == "STOPPED")
        wait(self.ready)
        count = self.eff.point_applications
        time.sleep(.3)
        self.assertEqual(self.eff.point_applications, count)

    def test_collect_stop_start_keeps_effector_stopped(self):
        self.start()
        self.c2.command("collect stop")
        wait(lambda: self.state()["devices"]["101"]["measurement_result"] == "COMPLETED")
        wait(lambda: self.control().get("state") == "STOPPED")
        self.c2.command("collect start")
        wait(self.ready)
        self.assertEqual(self.control()["state"], "STOPPED")


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]] + rest, verbosity=2)
