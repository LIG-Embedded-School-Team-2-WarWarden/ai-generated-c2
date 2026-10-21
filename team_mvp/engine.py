"""No-GUI, fixed-OBS educational pointer runtime using the shared ICD 1.0."""

import collections
import ipaddress
import threading
import time
import uuid
from google.protobuf.message import DecodeError
from .contract import pb, uid, decode
from .observation import ObservationService
from .clock import Clock, SystemClock
from .transport import DatagramTransport, UdpTransport
from .detection import PointDetector
from .pointing import PointingSolver
from .requests import RequestTracker
from .observer import ObserverController
from .effector import EffectorController
from .devices import DeviceSessions
from .geometry import validate_installation


class Engine:
    def __init__(
        self,
        config,
        detector: PointDetector,
        bind="127.0.0.1",
        port=5000,
        *,
        clock: Clock | None = None,
        transport: DatagramTransport | None = None,
        autostart=True,
        pointing_solver: PointingSolver | None = None,
    ):
        self.clock = clock if clock is not None else SystemClock()
        self.config = config
        self.obs_id = config["observer"]
        for value in config["devices"].values():
            validate_installation(value)
        if config["max_clock_uncertainty_ms"] <= 0:
            raise ValueError("deployment clock uncertainty limit required")
        if config.get("simulation") and not ipaddress.ip_address(bind).is_loopback:
            raise ValueError("synthetic clock profile requires loopback bind")
        self.boot = uid()
        self.lock = threading.RLock()
        self.events = collections.deque(maxlen=256)
        self.transport = transport if transport is not None else UdpTransport(bind, port)
        self.requests = RequestTracker(self.clock, self.transport, self.log)
        self.observation = ObservationService(self, detector, autostart)
        self.devices = DeviceSessions(self)
        self.effector = EffectorController(self, pointing_solver)
        self.observer = ObserverController(self)
        self.address = self.transport.address
        self.closed = False
        self.running = True
        self.last_hb = 0
        self.hb_sequence = 0
        self.last_clock = (self.clock.time(), self.clock.monotonic())
        self.thread = threading.Thread(target=self.run, daemon=True) if autostart else None
        if self.thread is not None:
            self.thread.start()

    def log(self, event, **values):
        self.events.append(dict(event=event, **values))

    @property
    def pending(self):
        # Compatibility view for diagnostic tools; the service owns lifecycle rules.
        return self.requests.pending

    def send(self, peer, body):
        return self.requests.send(peer, body)

    def request(self, peer, body, result_kind, callback, timeout=2):
        return self.requests.request(peer, body, result_kind, callback, timeout)

    @property
    def peers(self):
        return self.devices.peers

    def register(self, envelope, address):
        self.devices.register(envelope, address)

    def receive(self, data, address):
        e = decode(data, "C2")
        kind = e.WhichOneof("body")
        if kind == "register_request":
            self.register(e, address)
            return
        p = self.peers.get(e.sender_id)
        if not p or p["session"] != e.connection_session_id or p["address"][0] != address[0]:
            if p and p["address"][0] == address[0]:
                self.send(p, pb.SessionRequired(device_id=p["id"], c2_boot_id=self.boot))
            raise ValueError("unknown session/source")
        b = getattr(e, kind)
        if kind == "heartbeat":
            self.devices.heartbeat(p, b)
        elif kind == "device_status":
            self.devices.status(p, b)
        elif kind == "capability_report":
            self.devices.capability(p, b)
        elif kind.endswith("_result"):
            self.result(p, kind, b)
        elif kind == "obs_pose" and p["id"] == self.obs_id:
            uuid.UUID(b.pose_id)
            old = p["poses"].get(b.pose_id)
            if old is not None and old != b:
                self.cloud.fail("POSE_CONFLICT")
                self.stop_all("POSE_CONFLICT")
                return
            p["poses"][b.pose_id] = b
            while len(p["poses"]) > 64:
                del p["poses"][next(iter(p["poses"]))]
        elif kind == "point_cloud_chunk" and p["id"] == self.obs_id:
            if (
                not p["installed"]
                or not p["collecting"]
                or b.collection_session_id != p["collection"]
            ):
                return
            verified = self.observation.verify_frame(p, b)
            self.cloud.ingest(p["session"], b, verified)
        elif kind == "error_report":
            self.stop_all("DEVICE_ERROR")

    def status_valid(self, peer, stopped=False):
        return self.devices.status_valid(peer, stopped)

    def collect(self, start, _observer=False):
        with self.lock:
            self.observer.collect(start, _observer)

    @property
    def observer_action(self):
        return self.observer.snapshot()

    def obs_point(self, pan, tilt):
        with self.lock:
            self.observer.point(pan, tilt)

    def obs_stop(self):
        with self.lock:
            self.observer.stop()

    @property
    def cloud(self):
        return self.observation.cloud

    def invalidate_observation(self, reason, reset=False):
        self.observation.invalidate(reason, reset)

    def forget_control(self, device):
        self.effector.sessions.pop(device, None)

    def effectors_stopped(self):
        return self.effector.stopped

    def trust_clock(self, device, mapping):
        with self.lock:
            self.observation.trust_clock(device, mapping)

    def trust_acquisition_profile(self, device, profile):
        with self.lock:
            self.observation.trust_acquisition_profile(device, profile)

    def result(self, peer, kind, body):
        self.requests.handle_result(peer, kind, body)

    def publish(self, generation, session, meta, objects, verified, error):
        with self.lock:
            self.observation.publish(generation, session, meta, objects, verified, error)

    def on_observation(self, objects, now):
        self.effector.observed(objects, now)

    def latest(self):
        return self.observation.latest()

    @property
    def controls(self):
        return self.effector.sessions

    def aim(self, device, object_id=None):
        with self.lock:
            self.effector.aim(device, object_id)

    def point(self, device, pan, tilt):
        with self.lock:
            self.effector.point(device, pan, tilt)

    def stop(self, device, reason="OPERATOR_STOP"):
        with self.lock:
            self.effector.stop(device, reason)

    def stop_all(self, reason):
        self.effector.stop_all(reason)

    def observation_progress(self):
        return self.cloud.state, self.cloud.started

    def tick(self):
        now = self.clock.monotonic()
        wall = self.clock.time()
        if abs((wall - self.last_clock[0]) - (now - self.last_clock[1])) > 0.25:
            for p in self.peers.values():
                p["mapping"] = None
            self.invalidate_observation("CLOCK_CHANGED")
            self.stop_all("CLOCK_CHANGED")
        self.last_clock = wall, now
        if now - self.last_hb >= 0.5:
            self.last_hb = now
            self.hb_sequence += 1
            for p in self.peers.values():
                self.send(p, pb.Heartbeat(boot_id=self.boot, sequence=self.hb_sequence))
        self.requests.tick(self.peers)
        self.devices.expire_registrations(now)
        if self.cloud.frame and now - self.cloud.started >= 1:
            self.cloud.fail("FRAME_INCOMPLETE")
        self.effector.tick(now)
        self.observer.tick(now)

    def snapshot(self):
        with self.lock:
            result = self.cloud.result
            state = self.cloud.state
            if state == "CURRENT":
                try:
                    self.latest()
                except ValueError:
                    state = "STALE"
            return dict(
                devices={
                    key: dict(
                        session=p["session"],
                        boot=p["boot"],
                        installed=p["installed"],
                        connected=self.clock.monotonic() - p["alive"] <= 2,
                        sample=p["sample"],
                        heartbeat=p["hb"],
                        angles_current=self.status_valid(p),
                        collecting=p["collecting"],
                        measurement_result=p.get("measurement_result", "PENDING"),
                        pan=(
                            p["status"].angles.pan_deg
                            if p["status"] and p["status"].angles.HasField("pan_deg")
                            else None
                        ),
                        tilt=(
                            p["status"].angles.tilt_deg
                            if p["status"] and p["status"].angles.HasField("tilt_deg")
                            else None
                        ),
                    )
                    for key, p in self.peers.items()
                },
                observation=state,
                frame=result["meta"].frame_id if result else None,
                objects=(
                    result["objects"] if result and state in ("CURRENT", "DISPLAY_ONLY") else []
                ),
                controls=self.effector.snapshot(),
                observer_action=dict(self.observer_action) if self.observer_action else None,
                pending=len(self.pending),
            )

    def run(self):
        while self.running:
            try:
                data, address = self.transport.recvfrom(65536)
                with self.lock:
                    self.receive(data, address)
            except TimeoutError:
                pass
            except (ValueError, DecodeError, KeyError) as error:
                with self.lock:
                    self.log("rejected", reason=str(error))
            except OSError as error:
                if self.running:
                    self.log("socket_error", reason=str(error))
            try:
                with self.lock:
                    self.tick()
            except Exception as error:
                # Expose runtime faults and stop updates; never silently continue aiming.
                with self.lock:
                    self.log("runtime_error", reason=repr(error))
                    self.stop_all("RUNTIME_ERROR")

    def close(self):
        if self.closed:
            return
        with self.lock:
            if self.observer_action and self.observer_action["state"] not in (
                "COMPLETED",
                "FAILED",
            ):
                self.obs_stop()
            self.stop_all("SHUTDOWN")
        # Shutdown waits concern real background I/O, not simulated domain time.
        deadline = time.monotonic() + 2
        while self.thread is not None and time.monotonic() < deadline:
            with self.lock:
                if self.effector.stopped:
                    break
            time.sleep(0.02)
        with self.lock:
            p = self.peers.get(self.obs_id)
            if p and p["collecting"]:
                self.collect(False, _observer=True)
        deadline = time.monotonic() + 1
        while self.thread is not None and self.pending and time.monotonic() < deadline:
            time.sleep(0.02)
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2)
        self.transport.close()
        self.cloud.close()
        self.closed = True
