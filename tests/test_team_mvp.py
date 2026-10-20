"""Real UDP + generated shared Protobuf + unmodified upstream C++ detector."""
import argparse
import json
import math
import pathlib
import subprocess
import sys
import time
import unittest
import socket
import queue
import threading

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from team_mvp.contract import pb, decode, encode, envelope, uid, utc
from team_mvp.cloud import Detector
from team_mvp.engine import Engine
from team_mvp.peers import Peer
from team_mvp.geometry import angles

parser = argparse.ArgumentParser()
parser.add_argument("--detector",required=True)
args, rest = parser.parse_known_args()


def wait(predicate,timeout=4):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.015)
    raise AssertionError("condition timed out")


class Integration(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT/"configs/team-demo.json").read_text())
        self.engine = Engine(self.config,Detector(args.detector),port=0)
        self.obs = Peer("OBS",self.engine.address)
        self.eff = Peer("EFF",self.engine.address)
        wait(lambda: self.ready())

    def ready(self):
        s = self.engine.snapshot()
        return s["devices"].get("201",{}).get("installed") and s["observation"]=="CURRENT"

    def tearDown(self):
        self.engine.close()
        self.obs.close()
        self.eff.close()
        self.assertFalse(self.obs.errors,list(self.obs.errors))
        self.assertFalse(self.eff.errors,list(self.eff.errors))
        self.assertFalse([e for e in self.engine.events if e["event"]=="runtime_error"],list(self.engine.events))

    def control(self):
        return self.engine.snapshot()["controls"].get("201",{})

    def start(self):
        # Latest-frame invalidation can briefly report UPDATING; caller retries selection.
        wait(self.ready)
        self.engine.aim("201")
        wait(lambda: self.eff.point_applications >= 1)

    def stop(self):
        self.engine.stop("201")
        wait(lambda: self.control().get("state")=="STOPPED")

    def test_raw_detector_continuous_point_stop(self):
        self.assertEqual(self.engine.snapshot()["objects"][0]["count"],48)
        self.start()
        wait(lambda: self.eff.point_applications >= 4)
        points = [c for c in self.eff.commands if c.operation==pb.PointingCommand.OPERATION_POINT]
        self.assertGreater(len({c.source_frame_id for c in points}),2)
        self.assertEqual(len({c.object_id for c in points}),1)
        self.assertTrue(all(c.source_connection_session_id==self.obs.session for c in points))
        self.assertTrue(all(c.origin==pb.CONTROL_MODE_AUTO and c.HasField("expires_at") for c in points))
        for command in points:
            left = .1 + .02*math.sin(command.source_frame_id*.1)
            expected = math.degrees(math.atan2(.1,math.hypot(3,left)))
            self.assertAlmostEqual(command.tilt_deg,expected,places=3)
        times = [__import__("datetime").datetime.fromisoformat(c.issued_at.replace("Z","+00:00")).timestamp() for c in points]
        self.assertTrue(all(b-a >= .199 for a,b in zip(times,times[1:])))
        self.stop()
        count = self.eff.point_applications
        time.sleep(.4)
        self.assertEqual(self.eff.point_applications,count)
        stop = self.eff.commands[-1]
        self.assertEqual(stop.operation,pb.PointingCommand.OPERATION_STOP)
        self.assertFalse(stop.HasField("pan_deg"))
        self.assertFalse(stop.HasField("source_frame_id"))
        self.assertGreater(stop.command_sequence,points[-1].command_sequence)

    def test_retry_keeps_exact_packet_and_applies_once(self):
        self.eff.drop_first_point_result = True
        self.start()
        wait(lambda: len(self.eff.received_packets)>=5)
        wait(lambda: any(sum(k==key for k,_ in self.eff.received_packets)>1 for key,_ in self.eff.received_packets))
        packets = {}
        for key,packet in list(self.eff.received_packets):
            if key in packets:
                self.assertEqual(packet,packets[key])
            packets[key] = packet
        ids = [c.request_id for c in self.eff.commands]
        self.assertEqual(len(ids),len(set(ids)))
        self.stop()

    def test_duplicate_and_reordered_heartbeat_status_do_not_refresh(self):
        with self.engine.lock:
            p = self.engine.peers["201"]
            old_alive,old_received = p["alive"],p["status_received"]
            sample,hb = p["sample"],p["hb"]
            status = pb.DeviceStatus()
            status.CopyFrom(p["status"])
            status.angles.pan_deg = 80
            self.engine.receive(encode(envelope("201","C2",status,self.eff.session)),self.eff.sock.getsockname())
            self.engine.receive(encode(envelope("201","C2",pb.Heartbeat(boot_id=self.eff.boot,sequence=hb),self.eff.session)),self.eff.sock.getsockname())
            status.angles.sample_sequence = max(0,sample-1)
            self.engine.receive(encode(envelope("201","C2",status,self.eff.session)),self.eff.sock.getsockname())
            self.assertEqual(p["alive"],old_alive)
            self.assertEqual(p["status_received"],old_received)
            self.assertNotEqual(p["status"].angles.pan_deg,80)
            large = 2**40
            packet = encode(envelope("201","C2",pb.Heartbeat(boot_id=self.eff.boot,sequence=large),self.eff.session))
            self.engine.receive(packet,self.eff.sock.getsockname())
            self.assertEqual(p["hb"],large)
            fresh = p["alive"]
            self.engine.receive(packet,self.eff.sock.getsockname())
            self.assertEqual(p["alive"],fresh)

    def test_observer_restart_stops_without_resuming(self):
        self.start()
        previous_session = self.obs.session
        self.obs.close()
        self.obs = Peer("OBS",self.engine.address)
        wait(lambda: self.obs.session and self.obs.session != previous_session)
        wait(lambda: self.control().get("state")=="STOPPED")
        wait(self.ready)
        count = self.eff.point_applications
        time.sleep(.3)
        self.assertEqual(self.eff.point_applications,count)

    def test_empty_detection_stops_and_does_not_resume(self):
        self.start()
        self.obs.empty = True
        wait(lambda: self.control().get("state")=="STOPPED")
        self.assertEqual(self.engine.snapshot()["objects"],[])
        count = self.eff.point_applications
        self.obs.empty = False
        wait(self.ready)
        time.sleep(.3)
        self.assertEqual(self.eff.point_applications,count)

    def test_missing_clock_evidence_blocks_aim(self):
        self.obs.frame_mutator = lambda c: c.ClearField("clock_mapping_id")
        wait(lambda: self.engine.snapshot()["observation"]=="DISPLAY_ONLY")
        with self.assertRaises(ValueError):
            self.engine.aim("201")
        self.assertEqual(self.eff.point_applications,0)

    def test_installation_revision_change_blocks_further_points(self):
        self.start()
        with self.eff.lock:
            self.eff.revision = 2
            self.eff.status()
        wait(lambda: self.control().get("state")=="STOPPING")
        count = self.eff.point_applications
        time.sleep(.3)
        self.assertEqual(self.eff.point_applications,count)
        self.assertFalse(self.engine.snapshot()["devices"]["201"]["angles_current"])
        with self.eff.lock:
            self.eff.revision = 1
            self.eff.status()
        wait(lambda: self.control().get("state")=="STOPPED")

    def test_manual_point_has_no_observation_fields(self):
        self.engine.point("201",20,10)
        wait(lambda: self.control().get("reason")=="POINT_COMPLETED")
        command = self.eff.commands[-1]
        self.assertEqual(command.origin,pb.CONTROL_MODE_MANUAL)
        self.assertFalse(command.HasField("source_frame_id"))
        self.assertFalse(command.HasField("object_id"))
        self.assertEqual((command.pan_deg,command.tilt_deg),(20,10))
        self.stop()

    def test_incomplete_frame_invalidates_old_result(self):
        self.start()
        with self.obs.lock:
            self.obs.frames_enabled = False
            c = pb.PointCloudChunk()
            c.CopyFrom(self.obs.last_chunks[0])
            c.frame_id += 1
            self.obs.send(c)
        wait(lambda: self.engine.snapshot()["observation"]=="UPDATING")
        self.assertEqual(self.engine.snapshot()["objects"],[])
        wait(lambda: self.control().get("state")=="STOPPED")
        self.assertNotEqual(self.engine.snapshot()["observation"],"CURRENT")

    def test_stop_without_result_remains_unconfirmed(self):
        self.start()
        self.eff.drop_stop_results = True
        self.engine.stop("201")
        wait(lambda: self.control().get("reason")=="STOP_UNCONFIRMED")
        self.assertEqual(self.control()["state"],"STOPPING")
        count = self.eff.point_applications
        time.sleep(.3)
        self.assertEqual(self.eff.point_applications,count)
        self.eff.drop_stop_results = False

    def test_accepted_is_not_completed(self):
        self.eff.accept_only = True
        self.start()
        wait(lambda: self.control().get("state")=="STOPPED")
        self.assertEqual(self.eff.point_applications,1)
        self.assertEqual(self.control()["reason"],"POINT_UNCONFIRMED")

    def test_conflicting_chunk_and_old_session(self):
        self.start()
        with self.obs.lock:
            self.obs.frames_enabled = False
            c = pb.PointCloudChunk()
            c.CopyFrom(self.obs.last_chunks[0])
            c.points[0].sdk_depth_raw += 1
            self.obs.send(c)
        wait(lambda: self.control().get("state")=="STOPPED")
        self.assertEqual(self.engine.snapshot()["observation"],"FRAME_INVALID")
        with self.engine.lock:
            before = self.engine.cloud.generation
            with self.assertRaises(ValueError):
                self.engine.receive(encode(envelope("101","C2",c,uid())),self.obs.sock.getsockname())
            self.assertEqual(self.engine.cloud.generation,before)


class Contract(unittest.TestCase):
    def test_optional_invalid_xyz_and_oversized_packet(self):
        p = pb.Point(pixel_index=0,sdk_depth_raw=964000,xyz_valid=False)
        self.assertFalse(p.HasField("x"))
        c = pb.PointCloudChunk(points=[p]*500)
        with self.assertRaises(ValueError):
            encode(envelope("101","C2",c,uid()))
        old = pb.Envelope(version_major=3,sender_id="101",recipient_id="C2",message_id=uid(),sent_at=utc())
        with self.assertRaises(ValueError):
            decode(old.SerializeToString(),"C2")

    def test_simple_relative_position_math(self):
        cfg = json.loads((ROOT/"configs/team-demo.json").read_text())
        pan,tilt = angles(dict(x=.2,y=.1,z=3),cfg["devices"]["101"],cfg["devices"]["201"])
        self.assertAlmostEqual(pan,math.degrees(math.atan2(.1,3)))
        self.assertAlmostEqual(tilt,math.degrees(math.atan2(.1,math.hypot(3,.1))))


class Console(unittest.TestCase):
    def test_executable_console_scenario(self):
        probe = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1",0))
        port = probe.getsockname()[1]
        probe.close()
        executable = pathlib.Path(args.detector).resolve().with_name("c2_mvp_console.exe")
        command = [str(executable)] if sys.platform=="win32" else [sys.executable,str(ROOT/"scripts/run_team_mvp.py"),"--detector",args.detector]
        process = subprocess.Popen(command+["--asset-port",str(port)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
            text=True,encoding="utf-8")
        lines = queue.Queue()
        def read():
            for line in process.stdout:
                lines.put(line)
        reader = threading.Thread(target=read,daemon=True)
        reader.start()
        peers = [subprocess.Popen([sys.executable,str(ROOT/"scripts/team_dummy.py"),role,"--c2-port",str(port)],
            stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL) for role in ("OBS","EFF")]
        def write(command):
            process.stdin.write(command+"\n")
            process.stdin.flush()
        def state():
            write("status")
            end = time.monotonic()+2
            while time.monotonic() < end:
                line = lines.get(timeout=2)
                if "{" in line:
                    parsed = json.loads(line[line.index("{"):])
                    if "devices" in parsed:
                        return parsed
            raise AssertionError("console did not respond")
        def observed_state():
            snapshot = {}
            def available():
                snapshot.update(state())
                return snapshot.get("frame") is not None
            wait(available)
            return snapshot
        def begin_aim():
            write("aim 201")
            return state()["controls"].get("201",{}).get("state") in ("OPENING","RUNNING")
        try:
            wait(lambda: state()["devices"].get("201",{}).get("installed") and state()["observation"]=="CURRENT")
            write("targets")
            wait(begin_aim)
            wait(lambda: state()["controls"].get("201",{}).get("sequence",0)>=3)
            write("stop 201")
            wait(lambda: state()["controls"]["201"]["state"]=="STOPPED")
            stopped = observed_state()
            time.sleep(.3)
            later = observed_state()
            self.assertEqual(later["controls"]["201"]["sequence"],stopped["controls"]["201"]["sequence"])
            self.assertGreater(later["frame"],stopped["frame"])
            write("collect stop")
            wait(lambda: not state()["devices"]["101"]["collecting"])
            wait(lambda: state()["devices"]["101"]["measurement_result"]=="COMPLETED")
            write("collect start")
            wait(lambda: state()["observation"]=="CURRENT")
            self.assertEqual(state()["controls"]["201"]["state"],"STOPPED")
            write("quit")
            self.assertEqual(process.wait(timeout=5),0)
            self.assertTrue(all(p.poll() is None for p in peers))
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=3)
            for peer in peers:
                peer.terminate()
                peer.wait(timeout=3)
            reader.join(timeout=1)
            process.stdin.close()
            process.stdout.close()


if __name__=="__main__":
    unittest.main(argv=[sys.argv[0]]+rest,verbosity=2)
