"""EFF control sessions, stop fences and pointing policy; no socket ownership."""

from dataclasses import dataclass, asdict
from enum import Enum
from typing import Protocol
from .clock import Clock
from .requests import RequestTracker
from .pointing import PointingSolver, RelativePointingSolver
from .contract import pb, uid, finite


class EffectorPhase(str, Enum):
    OPENING = "OPENING"
    RUNNING = "RUNNING"
    MANUAL = "MANUAL"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"


@dataclass
class EffectorSession:
    session: str
    epoch: int
    state: EffectorPhase
    sequence: int = 0
    object: str | None = None
    last_send: float = 0
    last_frame: int = 0
    reason: str = ""
    stop_ack: bool = False
    last_observation: float = 0
    manual: tuple[float, float] | None = None
    stop_sample: int = 0

    def snapshot(self):
        value = asdict(self)
        value["state"] = self.state.value
        return value


class EffectorHost(Protocol):
    clock: Clock
    config: dict
    peers: dict
    obs_id: str
    requests: RequestTracker

    def status_valid(self, peer, stopped=False) -> bool: ...
    def request(self, peer, body, result_kind, callback, timeout=2): ...
    def latest(self) -> dict: ...
    def observation_progress(self) -> tuple[str, float]: ...
    def log(self, event, **values): ...


class EffectorController:
    def __init__(self, host: EffectorHost, solver: PointingSolver | None = None):
        self.host = host
        self.solver = solver if solver is not None else RelativePointingSolver()
        self.sessions: dict[str, EffectorSession] = {}

    def snapshot(self):
        return {k: v.snapshot() for k, v in self.sessions.items()}

    @property
    def stopped(self):
        return all(c.state == EffectorPhase.STOPPED for c in self.sessions.values())

    def observed(self, objects, now):
        if len(objects) == 1:
            for session in self.sessions.values():
                if objects[0]["id"] == session.object:
                    session.last_observation = now

    def tick(self, now):
        for device, session in list(self.sessions.items()):
            self._update(device, session, now)

    def aim(self, device, object_id=None):
        self.begin(device, object_id)

    def point(self, device, pan, tilt):
        if not finite(pan, tilt):
            raise ValueError("finite angles required")
        self.begin(device, manual=(pan, tilt))

    def begin(self, device, object_id=None, manual=None):
        p = self.host.peers[device]
        cfg = self.host.config["devices"][device]
        cap = p["cap"]
        if cfg["role"] != "EFF" or not p["installed"] or not self.host.status_valid(p, True):
            raise ValueError("EFF not ready/stopped with current measured angles")
        if not cap or not all(
            (
                cap.absolute_pointing,
                cap.measured_angle_feedback,
                cap.stop_supported,
                cap.drain_supported,
            )
        ):
            raise ValueError("EFF capabilities missing")
        for field in (
            "point_completion_timeout_ms",
            "stop_completion_timeout_ms",
            "pan_min_deg",
            "pan_max_deg",
            "tilt_min_deg",
            "tilt_max_deg",
        ):
            if not cap.HasField(field) or not finite(getattr(cap, field)):
                raise ValueError("EFF timeouts/limits missing")
        if (
            not 0 < cap.point_completion_timeout_ms <= 60000
            or not 0 < cap.stop_completion_timeout_ms <= 60000
            or cap.pan_min_deg >= cap.pan_max_deg
            or cap.tilt_min_deg >= cap.tilt_max_deg
        ):
            raise ValueError("invalid EFF timeouts/limits")
        old = self.sessions.get(device)
        if old and old.state != "STOPPED":
            raise ValueError("stop and confirm previous control first")
        selected = None
        if manual is None:
            r = self.host.latest()
            if len(r["objects"]) != 1 or (object_id and r["objects"][0]["id"] != object_id):
                raise ValueError("MVP requires one unambiguous object")
            selected = r["objects"][0]["id"]
        elif (
            not cap.pan_min_deg <= manual[0] <= cap.pan_max_deg
            or not cap.tilt_min_deg <= manual[1] <= cap.tilt_max_deg
        ):
            raise ValueError("OUT_OF_RANGE")
        p["epoch"] += 1
        control = EffectorSession(
            session=uid(),
            epoch=p["epoch"],
            state=EffectorPhase.OPENING,
            object=selected,
            last_observation=self.host.clock.monotonic(),
            manual=manual,
        )
        self.sessions[device] = control
        body = pb.ControlOpen(
            request_id=uid(),
            control_session_id=control.session,
            control_epoch=control.epoch,
            installation_revision=cfg["revision"],
            mode=pb.CONTROL_MODE_AUTO if manual is None else pb.CONTROL_MODE_MANUAL,
        )

        def opened(result):
            if control.state != "OPENING":
                return
            if result and result.outcome.result == pb.REQUEST_RESULT_COMPLETED:
                control.state = EffectorPhase.RUNNING if manual is None else EffectorPhase.MANUAL
                if manual is not None:
                    self._send_manual(p, control)
            else:
                self.stop(device, "CONTROL_UNCONFIRMED")

        self.host.request(p, body, "control_result", opened)

    def _send_manual(self, p, c):
        if not self.host.status_valid(p):
            self.stop(p["id"], "DEVICE_UNAVAILABLE")
            return
        c.sequence += 1
        expiry = self.host.clock.utc(after=1)
        body = pb.PointingCommand(
            request_id=uid(),
            control_session_id=c.session,
            control_epoch=c.epoch,
            command_sequence=c.sequence,
            operation=pb.PointingCommand.OPERATION_POINT,
            issued_at=self.host.clock.utc(),
            expires_at=expiry,
            pan_deg=c.manual[0],
            tilt_deg=c.manual[1],
            effector_installation_revision=self.host.config["devices"][p["id"]]["revision"],
            origin=pb.CONTROL_MODE_MANUAL,
        )

        def pointed(result):
            if not result or result.outcome.result != pb.REQUEST_RESULT_COMPLETED:
                self.stop(p["id"], "POINT_UNCONFIRMED")
            elif c.state == "MANUAL":
                c.reason = "POINT_COMPLETED"

        self.host.request(
            p, body, "pointing_result", pointed, p["cap"].point_completion_timeout_ms / 1000
        )

    def stop(self, device, reason="OPERATOR_STOP"):
        c = self.sessions.get(device)
        if (
            not c
            or c.state == "STOPPED"
            or (c.state == "STOPPING" and c.reason != "STOP_UNCONFIRMED")
        ):
            return
        p = self.host.peers[device]
        c.state, c.reason = EffectorPhase.STOPPING, reason
        c.stop_ack = False
        self.host.requests.cancel(device, {"control_result", "pointing_result"})
        c.sequence += 1
        body = pb.PointingCommand(
            request_id=uid(),
            control_session_id=c.session,
            control_epoch=c.epoch,
            command_sequence=c.sequence,
            operation=pb.PointingCommand.OPERATION_STOP,
            issued_at=self.host.clock.utc(),
        )
        c.stop_sample = p["sample"]

        def stopped(result):
            if (
                result
                and result.outcome.result == pb.REQUEST_RESULT_COMPLETED
                and result.actual_motion_state == pb.MOTION_STATE_STOPPED
            ):
                c.stop_ack = True
            else:
                c.reason = "STOP_UNCONFIRMED"

        timeout = p["cap"].stop_completion_timeout_ms / 1000 if p["cap"] else 2
        self.host.request(p, body, "pointing_result", stopped, max(0.2, timeout))

    def stop_all(self, reason):
        for device in list(self.sessions):
            self.stop(device, reason)

    def _update(self, device, c, now):
        p = self.host.peers[device]
        if c.state == "STOPPING":
            if c.stop_ack and p["sample"] > c.stop_sample and self.host.status_valid(p, True):
                c.state = EffectorPhase.STOPPED
            return
        if c.manual is not None and c.state in ("MANUAL", "OPENING"):
            if not self.host.status_valid(p):
                self.stop(device, "DEVICE_UNAVAILABLE")
            return
        if c.state not in ("RUNNING", "OPENING"):
            return
        obs = self.host.peers.get(self.host.obs_id)
        if (
            not self.host.status_valid(p)
            or not obs
            or not obs["collecting"]
            or self.host.clock.monotonic() - obs["alive"] > 2
        ):
            self.stop(device, "DEVICE_UNAVAILABLE")
            return
        state, started = self.host.observation_progress()
        if state in ("UPDATING", "PROCESSING") and now - started < 1:
            if now - c.last_observation >= 1:
                self.stop(device, "OBSERVATION_STALE")
            return
        try:
            r = self.host.latest()
            if len(r["objects"]) != 1 or r["objects"][0]["id"] != c.object:
                raise ValueError("OBJECT_LOST_OR_AMBIGUOUS")
            pan, tilt = self.solver.solve(
                r["objects"][0],
                self.host.config["devices"][self.host.obs_id],
                self.host.config["devices"][device],
            )
            cap = p["cap"]
            if (
                not cap.pan_min_deg <= pan <= cap.pan_max_deg
                or not cap.tilt_min_deg <= tilt <= cap.tilt_max_deg
            ):
                raise ValueError("OUT_OF_RANGE")
        except ValueError as error:
            self.stop(device, str(error))
            return
        if c.state != "RUNNING" or now - c.last_send < 0.2 or r["meta"].frame_id == c.last_frame:
            return
        if self.host.requests.has_pending(device, "pointing_result"):
            return
        c.sequence += 1
        expires = self.host.clock.utc(after=1)
        m = r["meta"]
        body = pb.PointingCommand(
            request_id=uid(),
            control_session_id=c.session,
            control_epoch=c.epoch,
            command_sequence=c.sequence,
            operation=pb.PointingCommand.OPERATION_POINT,
            issued_at=self.host.clock.utc(),
            expires_at=expires,
            pan_deg=pan,
            tilt_deg=tilt,
            effector_installation_revision=self.host.config["devices"][device]["revision"],
            origin=pb.CONTROL_MODE_AUTO,
            collection_session_id=m.collection_session_id,
            object_id=c.object,
            source_device_id=self.host.obs_id,
            source_connection_session_id=obs["session"],
            source_frame_id=m.frame_id,
            observation_installation_revision=m.installation_revision,
        )

        def pointed(result):
            if not result or result.outcome.result != pb.REQUEST_RESULT_COMPLETED:
                self.stop(device, "POINT_UNCONFIRMED")

        self.host.request(
            p, body, "pointing_result", pointed, p["cap"].point_completion_timeout_ms / 1000
        )
        c.last_send, c.last_frame = now, m.frame_id
        self.host.log("point", device=device, pan=pan, tilt=tilt, frame=m.frame_id)
