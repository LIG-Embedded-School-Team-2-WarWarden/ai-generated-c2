"""Device registration, capability/installation negotiation and sample freshness."""

import collections
import ipaddress
import uuid
from typing import Protocol
from .clock import Clock
from .requests import RequestTracker
from .transport import DatagramTransport
from .contract import pb, uid, stamp, envelope, encode, current_angles, finite


class DeviceHost(Protocol):
    clock: Clock
    config: dict
    obs_id: str
    boot: str
    requests: RequestTracker
    transport: DatagramTransport

    def stop_all(self, reason): ...
    def forget_control(self, device): ...
    def invalidate_observation(self, reason, reset=False): ...
    def request(self, peer, body, result_kind, callback, timeout=2): ...
    def collect(self, start, _observer=False): ...


class DeviceSessions:
    def __init__(self, host: DeviceHost):
        self.host = host
        self.peers = {}
        self.registrations = {}
        self.retired_boots = collections.defaultdict(set)

    def expire_registrations(self, now):
        self.registrations = {k: v for k, v in self.registrations.items() if now - v[1] < 60}

    def heartbeat(self, peer, body):
        if body.boot_id != peer["boot"] or body.sequence <= peer["hb"]:
            return
        peer["hb"], peer["alive"] = body.sequence, self.host.clock.monotonic()

    def status(self, peer, body):
        if body.device_id != peer["id"] or body.angles.sample_sequence <= peer["sample"]:
            return
        stamp(body.reported_at)
        peer["sample"], peer["status"], peer["status_received"] = (
            body.angles.sample_sequence,
            body,
            self.host.clock.monotonic(),
        )
        peer["alive"] = self.host.clock.monotonic()
        if (
            peer["install_result"]
            and body.active_installation_revision
            == self.host.config["devices"][peer["id"]]["revision"]
        ):
            peer["installed"] = True
        self.configure_peer(peer)

    def capability(self, peer, body):
        if body.device_id != peer["id"]:
            raise ValueError("capability device mismatch")
        if peer["cap"] is not None and peer["cap"] != body:
            self.host.stop_all("CAPABILITY_CHANGED")
            raise ValueError("re-register after capability change")
        peer["cap"] = body
        self.configure_peer(peer)

    def register(self, e, address):
        r = e.register_request
        uuid.UUID(r.request_id)
        uuid.UUID(r.boot_id)
        cfg = self.host.config["devices"].get(r.device_id)
        if not cfg or e.sender_id != r.device_id or r.role != pb.Role.Value("ROLE_" + cfg["role"]):
            raise ValueError("unlisted device/role")
        if not 1 <= r.receive_port <= 65535:
            raise ValueError("invalid receive port")
        if self.host.config.get("simulation") and not ipaddress.ip_address(address[0]).is_loopback:
            raise ValueError("simulation allows loopback peers only")
        key = (r.device_id, r.request_id)
        raw = r.SerializeToString(deterministic=True)
        cached = self.registrations.get(key)
        if cached:
            if cached[0] != raw:
                from .contract import outcome

                reply = pb.RegisterResult(
                    outcome=outcome(
                        r.request_id, pb.REQUEST_RESULT_REJECTED, pb.REASON_CODE_ID_CONFLICT
                    ),
                    device_id=r.device_id,
                    boot_id=r.boot_id,
                    c2_boot_id=self.host.boot,
                )
                self.host.transport.sendto(
                    encode(envelope("C2", r.device_id, reply, sent_at=self.host.clock.utc())),
                    (address[0], r.receive_port),
                )
                return
        p = self.peers.get(r.device_id)
        if r.boot_id in self.retired_boots[r.device_id]:
            raise ValueError("retired boot registration")
        if p and p["boot"] == r.boot_id:
            if p["address"] != (address[0], r.receive_port):
                raise ValueError("same boot changed endpoint")
        else:
            if p:
                self.retired_boots[r.device_id].add(p["boot"])
                self.host.stop_all("DEVICE_RESTART")
                self.host.requests.cancel(r.device_id)
                if r.device_id == self.host.obs_id:
                    self.host.invalidate_observation("DEVICE_RESTART", reset=True)
                self.host.forget_control(r.device_id)
            p = dict(
                id=r.device_id,
                boot=r.boot_id,
                session=uid(),
                address=(address[0], r.receive_port),
                hb=0,
                alive=self.host.clock.monotonic(),
                sample=0,
                status=None,
                status_received=0,
                cap=None,
                installed=False,
                install_result=False,
                install_sent=False,
                mode_sent=False,
                mode=False,
                mode_revision=1,
                obs_epoch=1,
                obs_sequence=0,
                obs_session=None,
                collection=None,
                collecting=False,
                poses={},
                mapping=None,
                epoch=0,
                disconnected=False,
            )
            self.peers[r.device_id] = p
        from .contract import outcome

        reply = pb.RegisterResult(
            outcome=outcome(r.request_id),
            device_id=r.device_id,
            boot_id=r.boot_id,
            connection_session_id=p["session"],
            c2_boot_id=self.host.boot,
        )
        self.host.transport.sendto(
            encode(envelope("C2", r.device_id, reply, sent_at=self.host.clock.utc())), p["address"]
        )
        self.registrations[key] = (raw, self.host.clock.monotonic())

    def status_valid(self, p, stopped=False):
        s = p["status"]
        limit = self.host.config["max_clock_uncertainty_ms"]
        if (
            not s
            or self.host.clock.monotonic() - p["status_received"] > 1
            or self.host.clock.monotonic() - p["alive"] > 2
        ):
            return False
        try:
            return (
                s.device_state in (pb.DEVICE_STATE_IDLE, pb.DEVICE_STATE_BUSY)
                and (
                    not p["install_result"]
                    or s.active_installation_revision
                    == self.host.config["devices"][p["id"]]["revision"]
                )
                and not s.HasField("fault_code")
                and s.estop_latch_state == pb.EMERGENCY_LATCH_STATE_RELEASED
                and (not s.HasField("estop_latched") or not s.estop_latched)
                and s.clock_state == pb.CLOCK_STATE_SYNCED
                and s.HasField("clock_uncertainty_ms")
                and finite(s.clock_uncertainty_ms)
                and 0 <= s.clock_uncertainty_ms <= limit
                and 0 <= self.host.clock.time() - stamp(s.reported_at) <= 1
                and current_angles(s.angles, self.host.clock.time())
                and (not stopped or s.motion_state == pb.MOTION_STATE_STOPPED)
            )
        except ValueError:
            return False

    def configure_peer(self, p):
        cap, cfg = p["cap"], self.host.config["devices"][p["id"]]
        if not cap or not self.status_valid(p, True):
            return
        if cap.calibration_id != cfg["calibration_id"]:
            return
        if cfg["role"] == "OBS" and (
            not cap.organized_frame_supported
            or cap.sensor_data_profile_id != cfg["profile"]
            or not cap.pose_feedback_supported
            or not cap.lidar_measurement_stop_supported
            or pb.OBSERVATION_MODE_FIXED_VIEW not in cap.supported_observation_modes
        ):
            return
        if not p["install_sent"]:
            p["install_sent"] = True
            body = pb.InstallationSet(
                request_id=uid(),
                device_id=p["id"],
                installation_revision=cfg["revision"],
                mount_rotation=cfg["rotation"],
                axis_calibration_id=cfg["calibration_id"],
                position_m=pb.Position(**dict(zip(("x", "y", "z"), cfg["position"]))),
                sensor_offset_m=pb.Position(**dict(zip(("x", "y", "z"), cfg["sensor_offset"]))),
            )

            def installed(result):
                if (
                    result
                    and result.outcome.result == pb.REQUEST_RESULT_COMPLETED
                    and result.device_id == p["id"]
                    and result.active_installation_revision == cfg["revision"]
                ):
                    p["install_result"] = True
                    # A later DEVICE_STATUS, never an earlier one, activates it.

            self.host.request(p, body, "installation_result", installed)
        if p["installed"] and cfg["role"] == "OBS" and not p["mode_sent"]:
            p["mode_sent"] = True
            body = pb.ObsModeCommand(
                request_id=uid(),
                obs_control_session_id=uid(),
                control_epoch=1,
                observation_mode_revision=1,
                mode=pb.OBSERVATION_MODE_FIXED_VIEW,
                installation_revision=cfg["revision"],
            )
            p["obs_session"] = body.obs_control_session_id

            def mode(result):
                if (
                    result
                    and result.outcome.result == pb.REQUEST_RESULT_COMPLETED
                    and result.obs_id == p["id"]
                    and result.observation_mode_revision == 1
                    and result.mode == pb.OBSERVATION_MODE_FIXED_VIEW
                    and result.actual_state == pb.MOTION_STATE_STOPPED
                ):
                    p["mode"] = True
                    p["mode_applied"] = stamp(result.mode_applied_at)
                    self.host.collect(True)

            self.host.request(p, body, "obs_mode_result", mode)
