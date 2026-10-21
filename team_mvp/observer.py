"""Explicit OBS workflow states; illegal shortcuts cannot resume collection."""

from dataclasses import dataclass, asdict
from enum import Enum
from typing import Protocol
from .clock import Clock
from .requests import RequestTracker
from .contract import pb, uid, stamp, finite


class ObserverPhase(str, Enum):
    WAIT_EFF_STOP = "WAIT_EFF_STOP"
    WAIT_COLLECTION_STOP = "WAIT_COLLECTION_STOP"
    OPENING = "OPENING"
    MOVING = "MOVING"
    STOPPING = "STOPPING"
    COLLECTING = "COLLECTING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


_ALLOWED = {
    ObserverPhase.WAIT_EFF_STOP: {ObserverPhase.WAIT_COLLECTION_STOP, ObserverPhase.FAILED},
    ObserverPhase.WAIT_COLLECTION_STOP: {
        ObserverPhase.OPENING,
        ObserverPhase.COMPLETED,
        ObserverPhase.FAILED,
    },
    ObserverPhase.OPENING: {ObserverPhase.MOVING, ObserverPhase.FAILED},
    ObserverPhase.MOVING: {ObserverPhase.COLLECTING, ObserverPhase.STOPPING, ObserverPhase.FAILED},
    ObserverPhase.STOPPING: {ObserverPhase.WAIT_COLLECTION_STOP, ObserverPhase.FAILED},
    ObserverPhase.COLLECTING: {ObserverPhase.COMPLETED, ObserverPhase.FAILED},
    ObserverPhase.COMPLETED: set(),
    ObserverPhase.FAILED: set(),
}


@dataclass
class ObserverOperation:
    phase: ObserverPhase
    session: str
    started: float
    pan: float = 0
    tilt: float = 0
    reason: str = ""
    operation: str = "POINT"
    sample: int = 0
    ack: bool = False
    failed: bool = False

    def transition(self, phase: ObserverPhase):
        if phase == self.phase:
            return
        if phase not in _ALLOWED[self.phase]:
            raise ValueError(f"illegal OBS transition: {self.phase.value} -> {phase.value}")
        self.phase = phase

    @property
    def terminal(self):
        return self.phase in (ObserverPhase.COMPLETED, ObserverPhase.FAILED)

    def snapshot(self):
        value = asdict(self)
        value["state"] = value.pop("phase").value
        return value


class ObserverHost(Protocol):
    """Application services required by the OBS workflow, independent of Engine."""

    clock: Clock
    requests: RequestTracker
    peers: dict
    config: dict
    obs_id: str

    def status_valid(self, peer, stopped=False) -> bool: ...
    def stop_all(self, reason): ...
    def effectors_stopped(self) -> bool: ...
    def collect(self, start, _observer=False): ...
    def request(self, peer, body, result_kind, callback, timeout=2): ...
    def invalidate_observation(self, reason): ...


class ObserverController:
    def __init__(self, host: ObserverHost):
        self.host = host
        self.action: ObserverOperation | None = None

    @property
    def busy(self):
        return self.action is not None and not self.action.terminal

    def snapshot(self):
        return self.action.snapshot() if self.action else None

    def point(self, pan, tilt):
        """Stop EFF/collection, move OBS, confirm stopped feedback, then collect."""
        p = self.host.peers.get(self.host.obs_id)
        if not p or not p["installed"] or not p["mode"] or not self.host.status_valid(p, True):
            raise ValueError("OBS not ready with current stopped feedback")
        cap = p["cap"]
        if (
            not finite(pan, tilt)
            or not cap.pan_min_deg <= pan <= cap.pan_max_deg
            or not cap.tilt_min_deg <= tilt <= cap.tilt_max_deg
        ):
            raise ValueError("OBS angles outside reported limits")
        if (
            not cap.HasField("point_completion_timeout_ms")
            or not 0 < cap.point_completion_timeout_ms <= 60000
        ):
            raise ValueError("OBS completion timeout missing")
        if self.action and self.action.phase not in ("COMPLETED", "FAILED"):
            raise ValueError("OBS operation pending")
        if self.host.requests.has_pending(self.host.obs_id):
            raise ValueError("OBS request pending")
        self.host.stop_all("OBS_MOVE")
        self.host.invalidate_observation("OBS_MOVING")
        self.action = ObserverOperation(
            phase=ObserverPhase.WAIT_EFF_STOP,
            session=p["session"],
            pan=pan,
            tilt=tilt,
            started=self.host.clock.monotonic(),
        )

    def stop(self):
        p = self.host.peers.get(self.host.obs_id)
        if not p or not p["installed"] or not p["mode"]:
            raise ValueError("OBS not installed/mode ready")
        self.host.stop_all("OBS_STOP")
        self.host.requests.cancel(self.host.obs_id, {"obs_motion_result", "obs_mode_result"})
        self.host.invalidate_observation("OBS_STOP")
        self.action = ObserverOperation(
            phase=ObserverPhase.STOPPING,
            session=p["session"],
            started=self.host.clock.monotonic(),
            operation="STOP",
        )
        self._motion(p, self.action, False)

    def _motion(self, p, a, point):
        p["obs_sequence"] += 1
        q = pb.ObsMotionCommand(
            request_id=uid(),
            obs_control_session_id=p["obs_session"],
            control_epoch=p["obs_epoch"],
            command_sequence=p["obs_sequence"],
            operation=(
                pb.ObsMotionCommand.OPERATION_POINT if point else pb.ObsMotionCommand.OPERATION_STOP
            ),
            issued_at=self.host.clock.utc(),
        )
        if point:
            q.pan_deg, q.tilt_deg = a.pan, a.tilt
            q.purpose = pb.ObsMotionCommand.PURPOSE_FIXED_POSITION
            q.installation_revision = self.host.config["devices"][self.host.obs_id]["revision"]
            q.observation_mode_revision = p["mode_revision"]
            q.expires_at = self.host.clock.utc(after=1)
        a.sample, a.ack = p["sample"], False
        a.transition(ObserverPhase.MOVING if point else ObserverPhase.STOPPING)

        def completed(result):
            if self.action is not a or a.phase not in ("MOVING", "STOPPING"):
                return
            if (
                result
                and result.outcome.result == pb.REQUEST_RESULT_COMPLETED
                and result.actual_motion_state == pb.MOTION_STATE_STOPPED
            ):
                a.ack = True
            elif point:
                # A lost final result must never let an earlier POINT retry outlive STOP.
                a.operation, a.failed, a.reason = "STOP", True, "OBS_MOTION_UNCONFIRMED"
                a.started = self.host.clock.monotonic()
                self._motion(p, a, False)
            else:
                a.transition(ObserverPhase.FAILED)
                a.reason = "OBS_STOP_UNCONFIRMED"

        field = "point_completion_timeout_ms" if point else "stop_completion_timeout_ms"
        self.host.request(
            p, q, "obs_motion_result", completed, max(0.2, getattr(p["cap"], field) / 1000)
        )

    def tick(self, now):
        a = self.action
        if not a or a.phase in ("COMPLETED", "FAILED"):
            return
        p = self.host.peers.get(self.host.obs_id)
        if not p or p["session"] != a.session or now - p["alive"] > 2:
            a.transition(ObserverPhase.FAILED)
            a.reason = "OBS_DISCONNECTED"
            self.host.requests.cancel(self.host.obs_id, {"obs_mode_result", "obs_motion_result"})
            return
        if now - a.started > max(10, p["cap"].point_completion_timeout_ms / 1000 + 10):
            a.transition(ObserverPhase.FAILED)
            a.reason = "OBS_OPERATION_TIMEOUT"
            self.host.requests.cancel(self.host.obs_id, {"obs_mode_result", "obs_motion_result"})
            return
        state = a.phase
        if state == "WAIT_EFF_STOP":
            if not self.host.effectors_stopped():
                return
            a.sample = p["sample"]
            if p["collecting"]:
                self.host.collect(False, _observer=True)
            a.transition(ObserverPhase.WAIT_COLLECTION_STOP)
        elif state == "WAIT_COLLECTION_STOP":
            if self.host.requests.has_pending(self.host.obs_id, "measurement_result"):
                return
            if p.get("measurement_result") == "UNCONFIRMED":
                a.transition(ObserverPhase.FAILED)
                a.reason = "COLLECTION_STOP_UNCONFIRMED"
                return
            if (
                not self.host.status_valid(p, True)
                or p["status"].measurement_state != pb.MEASUREMENT_STATE_STOPPED
            ):
                return
            if a.operation == "STOP":
                a.transition(ObserverPhase.FAILED if a.failed else ObserverPhase.COMPLETED)
                return
            p["obs_epoch"] += 1
            p["mode_revision"] += 1
            p["obs_sequence"] = 0
            p["obs_session"] = uid()
            q = pb.ObsModeCommand(
                request_id=uid(),
                obs_control_session_id=p["obs_session"],
                control_epoch=p["obs_epoch"],
                observation_mode_revision=p["mode_revision"],
                mode=pb.OBSERVATION_MODE_FIXED_VIEW,
                installation_revision=self.host.config["devices"][self.host.obs_id]["revision"],
            )
            a.transition(ObserverPhase.OPENING)

            def opened(result):
                if self.action is not a or a.phase != "OPENING":
                    return
                if (
                    not result
                    or result.outcome.result != pb.REQUEST_RESULT_COMPLETED
                    or result.obs_id != self.host.obs_id
                    or result.observation_mode_revision != p["mode_revision"]
                    or result.mode != pb.OBSERVATION_MODE_FIXED_VIEW
                    or result.actual_state != pb.MOTION_STATE_STOPPED
                ):
                    a.transition(ObserverPhase.FAILED)
                    a.reason = "OBS_MODE_UNCONFIRMED"
                    return
                p["mode_applied"] = stamp(result.mode_applied_at)
                self._motion(p, a, True)

            self.host.request(p, q, "obs_mode_result", opened)
        elif state in ("MOVING", "STOPPING"):
            if not a.ack or p["sample"] <= a.sample or not self.host.status_valid(p, True):
                return
            if state == "STOPPING":
                if self.host.requests.has_pending(self.host.obs_id, "measurement_result"):
                    return
                if p["status"].measurement_state != pb.MEASUREMENT_STATE_STOPPED:
                    self.host.collect(False, _observer=True)
                a.transition(ObserverPhase.WAIT_COLLECTION_STOP)
            else:
                self.host.collect(True, _observer=True)
                a.transition(ObserverPhase.COLLECTING)
        elif state == "COLLECTING":
            if p.get("measurement_result") == "UNCONFIRMED":
                a.transition(ObserverPhase.FAILED)
                a.reason = "COLLECTION_START_UNCONFIRMED"
            elif (
                p["collecting"] and p["status"].measurement_state == pb.MEASUREMENT_STATE_COLLECTING
            ):
                a.transition(ObserverPhase.COMPLETED)

    def collect(self, start, _observer=False):
        if not _observer and self.busy:
            raise ValueError("OBS operation pending; use obs stop to cancel")
        p = self.host.peers.get(self.host.obs_id)
        if not p or not p["installed"] or not p["mode"]:
            raise ValueError("OBS installation/mode not ready")
        if self.host.requests.has_pending(self.host.obs_id, "measurement_result"):
            raise ValueError("measurement request pending")
        if start and p["collecting"]:
            return
        if start:
            if not self.host.status_valid(p, True):
                raise ValueError("OBS must be stopped with current measured angles")
            p["collection"] = uid()
            # frameId high-water is connection-scoped; do not reset it here.
            self.host.invalidate_observation("NO_DATA")
        else:
            self.host.stop_all("COLLECTION_STOP")
            # Never report measurement complete on transmission alone.
            p["collecting"] = False
            self.host.invalidate_observation("NO_DATA")
        body = pb.MeasurementCommand(
            request_id=uid(),
            operation=(
                pb.MeasurementCommand.OPERATION_START
                if start
                else pb.MeasurementCommand.OPERATION_STOP
            ),
            collection_session_id=p["collection"],
            installation_revision=self.host.config["devices"][self.host.obs_id]["revision"],
        )
        p["measurement_result"] = "PENDING"

        def applied(result):
            expected = pb.MEASUREMENT_STATE_COLLECTING if start else pb.MEASUREMENT_STATE_STOPPED
            p["collecting"] = bool(
                start
                and result
                and result.outcome.result == pb.REQUEST_RESULT_COMPLETED
                and result.collection_session_id == p["collection"]
                and result.actual_state == expected
            )
            p["measurement_result"] = (
                "COMPLETED"
                if result
                and result.outcome.result == pb.REQUEST_RESULT_COMPLETED
                and result.actual_state == expected
                else "UNCONFIRMED"
            )

        self.host.request(p, body, "measurement_result", applied)
