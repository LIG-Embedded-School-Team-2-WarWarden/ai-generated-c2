"""Deterministic domain tests: no sockets, threads, subprocesses or sleeping."""

import json
import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from team_mvp.clock import Clock
from team_mvp.transport import DatagramTransport
from team_mvp.requests import RequestTracker
from team_mvp.observer import ObserverOperation, ObserverPhase, ObserverController
from team_mvp.effector import EffectorController, EffectorSession, EffectorPhase
from team_mvp.contract import pb, uid, outcome, decode
from team_mvp.engine import Engine


class FakeClock(Clock):
    def __init__(self):
        self.wall = 1700000000.0
        self.mono = 100.0

    def time(self):
        return self.wall

    def monotonic(self):
        return self.mono

    def advance(self, seconds):
        self.wall += seconds
        self.mono += seconds


class FakeTransport(DatagramTransport):
    def __init__(self):
        self.packets = []
        self.closed = False

    @property
    def address(self):
        return ("127.0.0.1", 54321)

    def sendto(self, packet, address):
        self.packets.append((packet, address))

    def recvfrom(self, size):
        raise TimeoutError()

    def close(self):
        self.closed = True


class FakeDetector:
    def detect(self, width, height, points):
        return []

    def close(self):
        pass


class RequestTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.transport = FakeClock(), FakeTransport()
        self.events, self.results = [], []
        self.tracker = RequestTracker(
            self.clock, self.transport, lambda *a, **k: self.events.append((a, k))
        )
        self.peer = dict(id="201", session=uid(), address=("127.0.0.1", 5001))
        self.command = pb.PointingCommand(
            request_id=uid(),
            control_session_id=uid(),
            control_epoch=2,
            command_sequence=7,
            operation=pb.PointingCommand.OPERATION_POINT,
            pan_deg=10,
            tilt_deg=5,
        )

    def request(self, timeout=2):
        self.tracker.request(
            self.peer, self.command, "pointing_result", self.results.append, timeout
        )

    def reply(self, value=pb.REQUEST_RESULT_COMPLETED, sequence=7):
        return pb.PointingResult(
            outcome=outcome(self.command.request_id, value),
            control_session_id=self.command.control_session_id,
            control_epoch=2,
            command_sequence=sequence,
        )

    def test_retry_preserves_exact_bytes_and_stops_after_budget(self):
        self.request()
        for _ in range(3):
            self.clock.advance(0.201)
            self.tracker.tick({"201": self.peer})
        self.assertEqual(len(self.transport.packets), 3)
        self.assertEqual(len({packet for packet, _ in self.transport.packets}), 1)
        self.assertEqual(self.results, [None])
        self.assertFalse(self.tracker.pending)

    def test_accepted_suppresses_retry_but_does_not_complete(self):
        self.request()
        self.tracker.handle_result(
            self.peer, "pointing_result", self.reply(pb.REQUEST_RESULT_ACCEPTED)
        )
        self.clock.advance(0.8)
        self.tracker.tick({"201": self.peer})
        self.assertEqual(len(self.transport.packets), 1)
        self.assertEqual(self.results, [])
        self.clock.advance(1.3)
        self.tracker.tick({"201": self.peer})
        self.assertEqual(self.results, [None])

    def test_wrong_sequence_or_connection_cannot_complete(self):
        self.request()
        self.tracker.handle_result(self.peer, "pointing_result", self.reply(sequence=6))
        other = dict(self.peer, session=uid())
        self.tracker.handle_result(other, "pointing_result", self.reply())
        self.assertFalse(self.results)
        self.tracker.handle_result(self.peer, "pointing_result", self.reply())
        self.tracker.handle_result(self.peer, "pointing_result", self.reply())
        self.assertEqual(len(self.results), 1)

    def test_cancelled_point_never_retries_or_calls_late_callback(self):
        self.request()
        self.tracker.cancel("201", {"pointing_result"})
        self.clock.advance(5)
        self.tracker.tick({"201": self.peer})
        self.tracker.handle_result(self.peer, "pointing_result", self.reply())
        self.assertEqual(len(self.transport.packets), 1)
        self.assertFalse(self.results)

    def test_session_change_expires_old_request_without_retransmission(self):
        self.request()
        self.clock.advance(0.201)
        self.tracker.tick({"201": dict(self.peer, session=uid())})
        self.assertEqual(len(self.transport.packets), 1)
        self.assertEqual(self.results, [None])

    def test_serialization_uses_injected_wall_clock(self):
        self.request()
        e = decode(self.transport.packets[0][0], "201")
        self.assertEqual(e.sent_at, self.clock.utc())

    def test_caller_mutation_cannot_change_inflight_correlation(self):
        self.request()
        self.command.command_sequence = 999
        self.tracker.handle_result(self.peer, "pointing_result", self.reply(sequence=7))
        self.assertEqual(len(self.results), 1)

    def test_wall_clock_adjustment_does_not_advance_retry_deadline(self):
        self.request()
        self.clock.wall += 60
        self.tracker.tick({"201": self.peer})
        self.assertEqual(len(self.transport.packets), 1)
        self.assertFalse(self.results)

    def test_timeout_callback_can_schedule_a_stop(self):
        def timed_out(result):
            self.results.append(result)
            stop = pb.PointingCommand(request_id=uid(), operation=pb.PointingCommand.OPERATION_STOP)
            self.tracker.request(self.peer, stop, "pointing_result", lambda value: None)

        self.tracker.request(self.peer, self.command, "pointing_result", timed_out, 0.1)
        self.clock.advance(0.11)
        self.tracker.tick({"201": self.peer})
        self.assertEqual(self.results, [None])
        self.assertEqual(len(self.tracker.pending), 1)


class ObserverStateTests(unittest.TestCase):
    def test_valid_move_workflow_reaches_completed(self):
        a = ObserverOperation(phase=ObserverPhase.WAIT_EFF_STOP, session=uid(), started=0)
        for phase in (
            ObserverPhase.WAIT_COLLECTION_STOP,
            ObserverPhase.OPENING,
            ObserverPhase.MOVING,
            ObserverPhase.COLLECTING,
            ObserverPhase.COMPLETED,
        ):
            a.transition(phase)
        self.assertEqual(a.snapshot()["state"], "COMPLETED")

    def test_collection_cannot_skip_stopped_confirmation(self):
        a = ObserverOperation(phase=ObserverPhase.WAIT_EFF_STOP, session=uid(), started=0)
        with self.assertRaises(ValueError):
            a.transition(ObserverPhase.COLLECTING)

    def test_terminal_operation_cannot_be_resurrected_by_late_event(self):
        a = ObserverOperation(phase=ObserverPhase.STOPPING, session=uid(), started=0)
        a.transition(ObserverPhase.FAILED)
        with self.assertRaises(ValueError):
            a.transition(ObserverPhase.MOVING)


class WorkflowHost:
    """In-memory application port used by real OBS/EFF controllers."""

    def __init__(self):
        self.clock, self.transport = FakeClock(), FakeTransport()
        self.requests = RequestTracker(self.clock, self.transport)
        self.obs_id = "101"
        self.config = {"devices": {"101": {"revision": 1}, "201": {"revision": 1}}}
        cap = pb.CapabilityReport(
            absolute_pointing=True,
            measured_angle_feedback=True,
            stop_supported=True,
            drain_supported=True,
            point_completion_timeout_ms=200,
            stop_completion_timeout_ms=200,
            pan_min_deg=-90,
            pan_max_deg=90,
            tilt_min_deg=-45,
            tilt_max_deg=45,
        )
        self.peers = {
            key: dict(
                id=key,
                session=uid(),
                address=("127.0.0.1", 5001 if key == "101" else 5002),
                installed=True,
                mode=True,
                cap=cap,
                sample=1,
                epoch=0,
                obs_epoch=1,
                obs_sequence=0,
                obs_session=uid(),
                mode_revision=1,
                alive=self.clock.monotonic(),
                collecting=True,
                measurement_result="COMPLETED",
                status=pb.DeviceStatus(
                    motion_state=pb.MOTION_STATE_STOPPED,
                    measurement_state=pb.MEASUREMENT_STATE_COLLECTING,
                ),
            )
            for key in ("101", "201")
        }
        self.effectors_ready = False
        self.collections = []
        self.stop_reasons = []
        self.result = dict(
            objects=[{"id": "object-1"}],
            meta=pb.PointCloudChunk(
                frame_id=1, collection_session_id=uid(), installation_revision=1
            ),
        )

    def status_valid(self, p, stopped=False):
        return not stopped or p["status"].motion_state == pb.MOTION_STATE_STOPPED

    def stop_all(self, reason):
        self.stop_reasons.append(reason)

    def effectors_stopped(self):
        return self.effectors_ready

    def invalidate_observation(self, reason):
        pass

    def collect(self, start, _observer=False):
        self.collections.append(start)
        self.peers["101"]["collecting"] = False
        self.peers["101"]["measurement_result"] = "PENDING"
        self.peers["101"]["status"].measurement_state = (
            pb.MEASUREMENT_STATE_STARTING if start else pb.MEASUREMENT_STATE_STOPPING
        )

    def request(self, *args):
        self.requests.request(*args)

    def latest(self):
        return self.result

    def observation_progress(self):
        return "CURRENT", self.clock.monotonic()

    def log(self, *args, **kwargs):
        pass

    def last(self):
        return decode(
            self.transport.packets[-1][0],
            "101" if self.transport.packets[-1][1][1] == 5001 else "201",
        )

    def motion_reply(self, q):
        return pb.ObsMotionResult(
            outcome=outcome(q.request_id),
            obs_id="101",
            obs_control_session_id=q.obs_control_session_id,
            control_epoch=q.control_epoch,
            command_sequence=q.command_sequence,
            actual_motion_state=pb.MOTION_STATE_STOPPED,
        )


class ObserverWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.host = WorkflowHost()
        self.controller = ObserverController(self.host)
        self.p = self.host.peers["101"]

    def moving(self):
        self.controller.point(10, 5)
        self.controller.tick(self.host.clock.monotonic())
        self.assertFalse(self.host.collections)  # EFF stop has not been confirmed.
        self.host.effectors_ready = True
        self.controller.tick(self.host.clock.monotonic())
        self.assertEqual(self.host.collections, [False])
        self.controller.tick(self.host.clock.monotonic())
        self.assertEqual(self.controller.action.phase, ObserverPhase.WAIT_COLLECTION_STOP)
        self.p["measurement_result"] = "COMPLETED"
        self.p["status"].measurement_state = pb.MEASUREMENT_STATE_STOPPED
        self.controller.tick(self.host.clock.monotonic())
        q = self.host.last().obs_mode_command
        self.host.requests.handle_result(
            self.p,
            "obs_mode_result",
            pb.ObsModeResult(
                outcome=outcome(q.request_id),
                obs_id="101",
                observation_mode_revision=q.observation_mode_revision,
                mode=pb.OBSERVATION_MODE_FIXED_VIEW,
                actual_state=pb.MOTION_STATE_STOPPED,
                mode_applied_at=self.host.clock.utc(),
            ),
        )
        self.assertEqual(self.controller.action.phase, ObserverPhase.MOVING)
        return self.host.last().obs_motion_command

    def test_collection_waits_for_result_and_fresh_stopped_feedback(self):
        q = self.moving()
        self.host.requests.handle_result(self.p, "obs_motion_result", self.host.motion_reply(q))
        self.controller.tick(self.host.clock.monotonic())
        self.assertEqual(self.host.collections, [False])  # Result alone is insufficient.
        self.p["sample"] += 1
        self.controller.tick(self.host.clock.monotonic())
        self.assertEqual(self.host.collections, [False, True])
        self.assertEqual(self.controller.action.phase, ObserverPhase.COLLECTING)
        self.p["collecting"] = True
        self.p["measurement_result"] = "COMPLETED"
        self.p["status"].measurement_state = pb.MEASUREMENT_STATE_COLLECTING
        self.controller.tick(self.host.clock.monotonic())
        self.assertEqual(self.controller.action.phase, ObserverPhase.COMPLETED)

    def test_stop_ignores_late_completion_of_replaced_move(self):
        q = self.moving()
        callback = self.host.requests.pending[q.request_id]["callback"]
        self.controller.stop()
        stop = self.host.last().obs_motion_command
        self.assertGreater(stop.command_sequence, q.command_sequence)
        callback(self.host.motion_reply(q))
        self.assertEqual(self.controller.action.phase, ObserverPhase.STOPPING)
        self.assertEqual(self.host.collections, [False])
        self.assertNotIn(q.request_id, self.host.requests.pending)

    def test_lost_completion_issues_stop_without_resuming_collection(self):
        q = self.moving()
        self.host.clock.advance(0.21)
        self.host.requests.tick(self.host.peers)
        stop = self.host.last().obs_motion_command
        self.assertEqual(stop.operation, pb.ObsMotionCommand.OPERATION_STOP)
        self.assertGreater(stop.command_sequence, q.command_sequence)
        self.host.requests.handle_result(self.p, "obs_motion_result", self.host.motion_reply(stop))
        self.p["sample"] += 1
        self.controller.tick(self.host.clock.monotonic())
        self.controller.tick(self.host.clock.monotonic())
        self.assertEqual(self.controller.action.phase, ObserverPhase.FAILED)
        self.assertEqual(self.controller.action.reason, "OBS_MOTION_UNCONFIRMED")
        self.assertEqual(self.host.collections, [False])


class PointingStrategyTests(unittest.TestCase):
    def test_controller_uses_injected_solver_and_waits_for_terminal_result(self):
        class Solver:
            def solve(self, obj, obs, eff):
                return 13, 7

        host = WorkflowHost()
        controller = EffectorController(host, Solver())
        controller.sessions["201"] = EffectorSession(
            session=uid(),
            epoch=1,
            state=EffectorPhase.RUNNING,
            object="object-1",
            last_observation=host.clock.monotonic(),
        )
        controller.tick(host.clock.monotonic())
        q = host.last().pointing_command
        self.assertEqual((q.pan_deg, q.tilt_deg), (13, 7))
        host.clock.advance(0.201)
        host.result["meta"].frame_id = 2
        controller.tick(host.clock.monotonic())
        self.assertEqual(len(host.transport.packets), 1)
        host.requests.handle_result(
            host.peers["201"],
            "pointing_result",
            pb.PointingResult(
                outcome=outcome(q.request_id),
                control_session_id=q.control_session_id,
                control_epoch=q.control_epoch,
                command_sequence=q.command_sequence,
            ),
        )
        controller.tick(host.clock.monotonic())
        self.assertEqual(len(host.transport.packets), 2)
        self.assertEqual(host.last().pointing_command.source_frame_id, 2)

    def test_replacement_solver_cannot_bypass_reported_limits(self):
        class InvalidSolver:
            def solve(self, obj, obs, eff):
                return 999, 0

        host = WorkflowHost()
        controller = EffectorController(host, InvalidSolver())
        controller.sessions["201"] = EffectorSession(
            session=uid(),
            epoch=1,
            state=EffectorPhase.RUNNING,
            object="object-1",
            last_observation=host.clock.monotonic(),
        )
        controller.tick(host.clock.monotonic())
        self.assertEqual(host.last().pointing_command.operation, pb.PointingCommand.OPERATION_STOP)
        self.assertEqual(controller.sessions["201"].reason, "OUT_OF_RANGE")


class EngineCompositionTests(unittest.TestCase):
    def test_engine_runs_one_cycle_without_network_or_worker_threads(self):
        config = json.loads((ROOT / "configs/team-demo.json").read_text(encoding="utf-8"))
        clock, transport = FakeClock(), FakeTransport()
        engine = Engine(config, FakeDetector(), clock=clock, transport=transport, autostart=False)
        try:
            self.assertIsNone(engine.thread)
            self.assertIsNone(engine.cloud.worker)
            self.assertEqual(engine.address, transport.address)
            clock.advance(1)
            engine.tick()
            self.assertEqual(engine.snapshot()["pending"], 0)
        finally:
            engine.close()
        self.assertTrue(transport.closed)


class DeviceFreshnessTests(unittest.TestCase):
    def setUp(self):
        config = json.loads((ROOT / "configs/team-demo.json").read_text(encoding="utf-8"))
        self.clock = FakeClock()
        self.engine = Engine(
            config, FakeDetector(), clock=self.clock, transport=FakeTransport(), autostart=False
        )
        self.addCleanup(self.engine.close)
        at = self.clock.utc()
        status = pb.DeviceStatus(
            device_id="201",
            reported_at=at,
            device_state=pb.DEVICE_STATE_IDLE,
            motion_state=pb.MOTION_STATE_STOPPED,
            clock_state=pb.CLOCK_STATE_SYNCED,
            clock_uncertainty_ms=1,
            estop_latch_state=pb.EMERGENCY_LATCH_STATE_RELEASED,
            angles=pb.AngleSample(
                sample_sequence=1,
                pan_deg=0,
                tilt_deg=0,
                pan_valid=True,
                tilt_valid=True,
                pan_measured_at=at,
                tilt_measured_at=at,
            ),
        )
        self.peer = dict(
            id="201",
            boot=uid(),
            hb=5,
            alive=self.clock.monotonic(),
            sample=1,
            status=status,
            status_received=self.clock.monotonic(),
            cap=None,
            install_result=False,
            collecting=False,
        )
        self.engine.peers["201"] = self.peer

    def test_duplicate_status_does_not_extend_measured_angle_freshness(self):
        self.assertTrue(self.engine.status_valid(self.peer, True))
        self.clock.advance(0.8)
        self.engine.devices.status(self.peer, self.peer["status"])
        self.clock.advance(0.21)
        self.assertFalse(self.engine.status_valid(self.peer, True))

    def test_duplicate_heartbeat_does_not_renew_liveness(self):
        old = self.peer["alive"]
        self.clock.advance(1)
        self.engine.devices.heartbeat(
            self.peer, pb.Heartbeat(boot_id=self.peer["boot"], sequence=5)
        )
        self.assertEqual(self.peer["alive"], old)
        self.engine.devices.heartbeat(
            self.peer, pb.Heartbeat(boot_id=self.peer["boot"], sequence=6)
        )
        self.assertEqual(self.peer["alive"], self.clock.monotonic())


if __name__ == "__main__":
    unittest.main(verbosity=2)
