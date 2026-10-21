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
from team_mvp.observer import ObserverOperation, ObserverPhase
from team_mvp.contract import pb, uid, outcome, decode
from team_mvp.engine import Engine


class FakeClock(Clock):
    def __init__(self):
        self.wall = 1700000000.0
        self.mono = 100.0
    def time(self): return self.wall
    def monotonic(self): return self.mono
    def advance(self, seconds):
        self.wall += seconds
        self.mono += seconds


class FakeTransport(DatagramTransport):
    def __init__(self):
        self.packets = []
        self.closed = False
    @property
    def address(self): return ('127.0.0.1', 54321)
    def sendto(self, packet, address): self.packets.append((packet, address))
    def recvfrom(self, size): raise TimeoutError()
    def close(self): self.closed = True


class FakeDetector:
    def detect(self, width, height, points): return []
    def close(self): pass


class RequestTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.transport = FakeClock(), FakeTransport()
        self.events, self.results = [], []
        self.tracker = RequestTracker(self.clock, self.transport, lambda *a, **k: self.events.append((a,k)))
        self.peer = dict(id='201', session=uid(), address=('127.0.0.1', 5001))
        self.command = pb.PointingCommand(request_id=uid(), control_session_id=uid(), control_epoch=2,
            command_sequence=7, operation=pb.PointingCommand.OPERATION_POINT, pan_deg=10, tilt_deg=5)
    def request(self, timeout=2):
        self.tracker.request(self.peer, self.command, 'pointing_result', self.results.append, timeout)
    def reply(self, value=pb.REQUEST_RESULT_COMPLETED, sequence=7):
        return pb.PointingResult(outcome=outcome(self.command.request_id,value),
            control_session_id=self.command.control_session_id,control_epoch=2,command_sequence=sequence)
    def test_retry_preserves_exact_bytes_and_stops_after_budget(self):
        self.request()
        for _ in range(3):
            self.clock.advance(.201)
            self.tracker.tick({'201':self.peer})
        self.assertEqual(len(self.transport.packets),3)
        self.assertEqual(len({packet for packet,_ in self.transport.packets}),1)
        self.assertEqual(self.results,[None])
        self.assertFalse(self.tracker.pending)
    def test_accepted_suppresses_retry_but_does_not_complete(self):
        self.request()
        self.tracker.handle_result(self.peer,'pointing_result',self.reply(pb.REQUEST_RESULT_ACCEPTED))
        self.clock.advance(.8);self.tracker.tick({'201':self.peer})
        self.assertEqual(len(self.transport.packets),1)
        self.assertEqual(self.results,[])
        self.clock.advance(1.3);self.tracker.tick({'201':self.peer})
        self.assertEqual(self.results,[None])
    def test_wrong_sequence_or_connection_cannot_complete(self):
        self.request()
        self.tracker.handle_result(self.peer,'pointing_result',self.reply(sequence=6))
        other=dict(self.peer,session=uid())
        self.tracker.handle_result(other,'pointing_result',self.reply())
        self.assertFalse(self.results)
        self.tracker.handle_result(self.peer,'pointing_result',self.reply())
        self.tracker.handle_result(self.peer,'pointing_result',self.reply())
        self.assertEqual(len(self.results),1)
    def test_cancelled_point_never_retries_or_calls_late_callback(self):
        self.request()
        self.tracker.cancel('201',{'pointing_result'})
        self.clock.advance(5);self.tracker.tick({'201':self.peer})
        self.tracker.handle_result(self.peer,'pointing_result',self.reply())
        self.assertEqual(len(self.transport.packets),1)
        self.assertFalse(self.results)
    def test_session_change_expires_old_request_without_retransmission(self):
        self.request()
        self.clock.advance(.201)
        self.tracker.tick({'201':dict(self.peer,session=uid())})
        self.assertEqual(len(self.transport.packets),1)
        self.assertEqual(self.results,[None])
    def test_serialization_uses_injected_wall_clock(self):
        self.request()
        e=decode(self.transport.packets[0][0],'201')
        self.assertEqual(e.sent_at,self.clock.utc())
    def test_caller_mutation_cannot_change_inflight_correlation(self):
        self.request()
        self.command.command_sequence=999
        self.tracker.handle_result(self.peer,'pointing_result',self.reply(sequence=7))
        self.assertEqual(len(self.results),1)

    def test_timeout_callback_can_schedule_a_stop(self):
        def timed_out(result):
            self.results.append(result)
            stop=pb.PointingCommand(request_id=uid(),operation=pb.PointingCommand.OPERATION_STOP)
            self.tracker.request(self.peer,stop,'pointing_result',lambda value:None)
        self.tracker.request(self.peer,self.command,'pointing_result',timed_out,.1)
        self.clock.advance(.11);self.tracker.tick({'201':self.peer})
        self.assertEqual(self.results,[None])
        self.assertEqual(len(self.tracker.pending),1)


class ObserverStateTests(unittest.TestCase):
    def test_valid_move_workflow_reaches_completed(self):
        a=ObserverOperation(phase=ObserverPhase.WAIT_EFF_STOP,session=uid(),started=0)
        for phase in (ObserverPhase.WAIT_COLLECTION_STOP,ObserverPhase.OPENING,ObserverPhase.MOVING,
                      ObserverPhase.COLLECTING,ObserverPhase.COMPLETED):
            a.transition(phase)
        self.assertEqual(a.snapshot()['state'],'COMPLETED')
    def test_collection_cannot_skip_stopped_confirmation(self):
        a=ObserverOperation(phase=ObserverPhase.WAIT_EFF_STOP,session=uid(),started=0)
        with self.assertRaises(ValueError):a.transition(ObserverPhase.COLLECTING)
    def test_terminal_operation_cannot_be_resurrected_by_late_event(self):
        a=ObserverOperation(phase=ObserverPhase.STOPPING,session=uid(),started=0)
        a.transition(ObserverPhase.FAILED)
        with self.assertRaises(ValueError):a.transition(ObserverPhase.MOVING)


class EngineCompositionTests(unittest.TestCase):
    def test_engine_runs_one_cycle_without_network_or_worker_threads(self):
        config=json.loads((ROOT/'configs/team-demo.json').read_text(encoding='utf-8'))
        clock,transport=FakeClock(),FakeTransport()
        engine=Engine(config,FakeDetector(),clock=clock,transport=transport,autostart=False)
        try:
            self.assertIsNone(engine.thread)
            self.assertIsNone(engine.cloud.worker)
            self.assertEqual(engine.address,transport.address)
            clock.advance(1);engine.tick()
            self.assertEqual(engine.snapshot()['pending'],0)
        finally:engine.close()
        self.assertTrue(transport.closed)


if __name__=='__main__':unittest.main(verbosity=2)
