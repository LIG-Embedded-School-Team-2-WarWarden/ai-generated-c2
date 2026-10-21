"""Observation acceptance, trusted acquisition time and detected object continuity."""

import math
from typing import Protocol
from .clock import Clock
from .cloud import Cloud
from .contract import pb, stamp, finite


class ObservationHost(Protocol):
    clock: Clock
    peers: dict
    config: dict
    obs_id: str

    def publish(self, generation, session, meta, objects, verified, error): ...
    def on_observation(self, objects, now): ...


class ObservationService:
    def __init__(self, host: ObservationHost, detector, autostart=True):
        self.host = host
        self.previous = None
        self.cloud = Cloud(detector, host.publish, clock=host.clock, autostart=autostart)

    def invalidate(self, reason, reset=False):
        if reset:
            self.cloud.reset()
        else:
            self.cloud.fail(reason)
        self.previous = None

    def trust_clock(self, device, mapping):
        """Operator supplied, verified mapping tied to this exact connection and boot."""
        p = self.host.peers[device]
        if mapping["boot_id"] != p["boot"] or mapping["connection_session_id"] != p["session"]:
            raise ValueError("clock evidence is for another boot/session")
        if (
            not mapping["evidence"]
            or not finite(mapping["offset_seconds"], mapping["uncertainty_ms"])
            or not 0 <= mapping["uncertainty_ms"] <= self.host.config["max_clock_uncertainty_ms"]
        ):
            raise ValueError("clock evidence/uncertainty required")
        p["mapping"] = dict(mapping)

    def trust_acquisition_profile(self, device, profile):
        """Bind externally measured sensor evidence to the currently connected process."""
        p = self.host.peers[device]
        if profile["sensor_profile"] != self.host.config["devices"][device]["profile"]:
            raise ValueError("sensor profile mismatch")
        start, end = stamp(profile["valid_from"]), stamp(profile["valid_until"])
        if not start <= self.host.clock.time() < end:
            raise ValueError("acquisition evidence expired/not active")
        mapping = {
            k: profile[k]
            for k in (
                "id",
                "evidence",
                "offset_seconds",
                "uncertainty_ms",
                "sync_type",
                "sync_state",
                "frame_protocol_version",
            )
        }
        if (
            not isinstance(mapping["id"], str)
            or not 0 < len(mapping["id"]) <= 64
            or any(
                type(mapping[k]) is not int or not 0 <= mapping[k] <= 255
                for k in ("sync_type", "sync_state", "frame_protocol_version")
            )
        ):
            raise ValueError("invalid mapping ID or SDK time identifiers")
        if (
            any(
                type(mapping[k]) not in (int, float) or not finite(mapping[k])
                for k in ("offset_seconds", "uncertainty_ms")
            )
            or mapping["uncertainty_ms"] < 1
        ):
            raise ValueError("measured finite offset/uncertainty required")
        mapping.update(
            boot_id=p["boot"], connection_session_id=p["session"], valid_from=start, valid_until=end
        )
        self.trust_clock(device, mapping)

    def verify_frame(self, p, c):
        cfg = self.host.config["devices"][p["id"]]
        if (
            c.installation_revision != cfg["revision"]
            or c.observation_mode_revision != p["mode_revision"]
            or c.sensor_data_profile_id != cfg["profile"]
        ):
            raise ValueError("frame installation/mode/profile mismatch")
        pose = p["poses"].get(c.pose_id)
        try:
            if not pose or not (
                pose.obs_id == p["id"]
                and pose.collection_session_id == p["collection"]
                and pose.installation_revision == c.installation_revision
                and pose.observation_mode_revision == p["mode_revision"]
                and pose.pose_valid
                and pose.pan_measured
                and pose.tilt_measured
                and pose.HasField("pan_deg")
                and pose.HasField("tilt_deg")
                and finite(pose.pan_deg, pose.tilt_deg)
                and abs(pose.pan_deg) <= 0.001
                and abs(pose.tilt_deg) <= 0.001
                and pose.motion_state == pb.MOTION_STATE_STOPPED
                and pose.captured_at == c.captured_at
            ):
                return False
            start, at, end = [
                stamp(getattr(c, k))
                for k in ("captured_start_at", "captured_at", "captured_end_at")
            ]
            if (
                not p["mode_applied"] <= start <= at <= end <= self.host.clock.time()
                or self.host.clock.time() - start > 1
            ):
                return False
            if not c.HasField("sdk_time") or c.sdk_time.nanoseconds >= 1000000000:
                return False
            mapping = p["mapping"]
            if self.host.config.get("simulation") and cfg["profile"] == "synthetic-organized-v1":
                mapping = dict(
                    id="synthetic-clock-v1",
                    offset_seconds=0,
                    uncertainty_ms=1,
                    sync_type=2,
                    sync_state=1,
                    frame_protocol_version=3,
                )
            if not mapping or c.clock_mapping_id != mapping["id"]:
                return False
            if (
                "valid_until" in mapping
                and not mapping["valid_from"]
                <= start
                <= end
                <= self.host.clock.time()
                <= mapping["valid_until"]
            ):
                return False
            if (c.sdk_time.sync_type, c.sdk_time.sync_state, c.sdk_time.frame_protocol_version) != (
                mapping["sync_type"],
                mapping["sync_state"],
                mapping["frame_protocol_version"],
            ):
                return False
            if (
                not c.HasField("clock_uncertainty_ms")
                or not finite(c.clock_uncertainty_ms)
                or not mapping["uncertainty_ms"]
                <= c.clock_uncertainty_ms
                <= self.host.config["max_clock_uncertainty_ms"]
            ):
                return False
            mapped = c.sdk_time.seconds + c.sdk_time.nanoseconds / 1e9 + mapping["offset_seconds"]
            return abs(mapped - at) <= 0.0011
        except (ValueError, KeyError):
            return False

    def publish(self, generation, session, meta, objects, verified, error):
        if generation != self.cloud.generation or session != self.cloud.key:
            return
        if error:
            self.cloud.fail("DETECTOR_ERROR")
            return
        if (
            self.previous
            and len(objects) == 1
            and self.host.clock.monotonic() - self.previous[1] <= 1
        ):
            old = self.previous[0]
            if (
                math.dist(
                    [old[k] for k in ("x", "y", "z")], [objects[0][k] for k in ("x", "y", "z")]
                )
                <= 0.5
            ):
                objects[0]["id"] = old["id"]
        self.previous = (
            (objects[0], self.host.clock.monotonic()) if len(objects) == 1 and verified else None
        )
        self.cloud.result = dict(
            meta=meta, objects=objects, verified=verified, received=self.host.clock.monotonic()
        )
        self.cloud.state = "CURRENT" if verified else "DISPLAY_ONLY"
        if verified:
            self.host.on_observation(objects, self.host.clock.monotonic())

    def latest(self):
        r = self.cloud.result
        mapping = self.host.peers.get(self.host.obs_id, {}).get("mapping")
        if mapping and "valid_until" in mapping and self.host.clock.time() > mapping["valid_until"]:
            raise ValueError("acquisition evidence expired")
        if self.cloud.state != "CURRENT" or not r or not r["verified"]:
            raise ValueError("no verified current observation")
        if (
            self.host.clock.monotonic() - r["received"] > 1
            or not 0 <= self.host.clock.time() - stamp(r["meta"].captured_start_at) <= 1
        ):
            raise ValueError("observation stale")
        return r
