"""Synthetic loopback equipment, NOT firmware or a LiDAR SDK adapter."""

import collections
import datetime as dt
import ipaddress
import math
import socket
import threading
import time
from .contract import pb, uid, utc, stamp, envelope, encode, decode, outcome


def iso(seconds):
    return (
        dt.datetime.fromtimestamp(seconds, dt.timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


class Peer:
    def __init__(self, role, c2=("127.0.0.1", 5000), device=None):
        if not ipaddress.ip_address(c2[0]).is_loopback:
            raise ValueError("synthetic peers are loopback only")
        self.role, self.c2 = role, c2
        self.device = device or ("101" if role == "OBS" else "201")
        self.boot, self.session = uid(), None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.01)
        self.lock = threading.RLock()
        self.running = True
        self.revision = 0
        self.pan = self.tilt = 0
        self.collection = None
        self.mode_revision = 0
        self.frame = 0
        self.sample = self.hb = 0
        self.c2_hb = 0
        self.c2_alive = time.monotonic()
        self.control, self.epoch, self.sequence = None, 0, 0
        self.gate = False
        self.cache = {}
        self.commands = collections.deque(maxlen=1024)
        self.received_packets = collections.deque(maxlen=2048)
        self.errors = collections.deque(maxlen=32)
        self.frames_enabled = True
        self.empty = False
        self.reverse_chunks = True
        self.duplicate_chunks = True
        self.drop_stop_results = False
        self.drop_first_point_result = False
        self.accept_only = False
        self.frame_mutator = None
        self.point_applications = 0
        self.stop_applications = 0
        self.last_status = None
        self.last_chunks = []
        self.register = pb.RegisterRequest(
            request_id=uid(),
            device_id=self.device,
            role=pb.ROLE_OBS if role == "OBS" else pb.ROLE_EFF,
            boot_id=self.boot,
            receive_port=self.sock.getsockname()[1],
        )
        self.register_packet = encode(envelope(self.device, "C2", self.register))
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, body):
        self.sock.sendto(encode(envelope(self.device, "C2", body, self.session)), self.c2)

    def capability(self):
        return pb.CapabilityReport(
            device_id=self.device,
            absolute_pointing=True,
            measured_angle_feedback=True,
            stop_supported=True,
            drain_supported=True,
            point_completion_timeout_ms=1000,
            stop_completion_timeout_ms=1000,
            pan_min_deg=-90,
            pan_max_deg=90,
            tilt_min_deg=-45,
            tilt_max_deg=45,
            calibration_id="synthetic-" + ("obs" if self.role == "OBS" else "eff") + "-zero-v1",
            supported_observation_modes=(
                [pb.OBSERVATION_MODE_FIXED_VIEW] if self.role == "OBS" else []
            ),
            pose_feedback_supported=self.role == "OBS",
            lidar_measurement_stop_supported=self.role == "OBS",
            sensor_data_profile_id="synthetic-organized-v1" if self.role == "OBS" else "",
            organized_frame_supported=self.role == "OBS",
        )

    def status(self):
        self.sample += 1
        at = utc()
        # These are values read from a simulated plant, never claimed as hardware feedback.
        s = pb.DeviceStatus(
            device_id=self.device,
            reported_at=at,
            angles=pb.AngleSample(
                sample_sequence=self.sample,
                pan_deg=self.pan,
                tilt_deg=self.tilt,
                pan_measured_at=at,
                tilt_measured_at=at,
                pan_valid=True,
                tilt_valid=True,
            ),
            device_state=pb.DEVICE_STATE_IDLE,
            motion_state=pb.MOTION_STATE_STOPPED,
            active_installation_revision=self.revision,
            clock_state=pb.CLOCK_STATE_SYNCED,
            clock_uncertainty_ms=1,
            estop_latch_state=pb.EMERGENCY_LATCH_STATE_RELEASED,
        )
        if self.role == "OBS":
            s.measurement_state = (
                pb.MEASUREMENT_STATE_COLLECTING if self.collection else pb.MEASUREMENT_STATE_STOPPED
            )
        self.last_status = s
        self.send(s)

    def receive(self, data):
        e = decode(data, self.device)
        if e.sender_id != "C2":
            return
        kind = e.WhichOneof("body")
        b = getattr(e, kind)
        if kind == "register_result":
            if (
                b.outcome.request_id != self.register.request_id
                or b.boot_id != self.boot
                or b.outcome.result != pb.REQUEST_RESULT_COMPLETED
            ):
                return
            if self.session == b.connection_session_id:
                return
            self.session = b.connection_session_id
            self.send(self.capability())
            self.status()
            return
        if kind == "session_required":
            self.session = None
            self.collection = None
            self.gate = False
            self.control = None
            self.cache.clear()
            self.epoch = self.sequence = 0
            self.register.request_id = uid()
            self.register_packet = encode(envelope(self.device, "C2", self.register))
            return
        if not self.session or e.connection_session_id != self.session:
            return
        if kind == "heartbeat":
            if b.sequence > self.c2_hb:
                self.c2_hb = b.sequence
                self.c2_alive = time.monotonic()
            return
        if not hasattr(b, "request_id"):
            return
        self.received_packets.append((b.request_id, data))
        raw = b.SerializeToString(deterministic=True)
        if b.request_id in self.cache:
            prior, response, _ = self.cache[b.request_id]
            if prior != raw:
                conflict = type(response)()
                conflict.CopyFrom(response)
                conflict.outcome.CopyFrom(
                    outcome(b.request_id, pb.REQUEST_RESULT_REJECTED, pb.REASON_CODE_ID_CONFLICT)
                )
                self.send(conflict)
            elif not (
                kind == "pointing_command"
                and b.operation == pb.PointingCommand.OPERATION_STOP
                and self.drop_stop_results
            ):
                self.send(response)
            return
        response = None
        if kind == "installation_set":
            self.revision = b.installation_revision
            response = pb.InstallationResult(
                outcome=outcome(b.request_id),
                device_id=self.device,
                active_installation_revision=self.revision,
            )
        elif kind == "obs_mode_command" and self.role == "OBS":
            self.mode_revision = b.observation_mode_revision
            self.control, self.epoch, self.sequence, self.gate = (
                b.obs_control_session_id,
                b.control_epoch,
                0,
                True,
            )
            response = pb.ObsModeResult(
                outcome=outcome(b.request_id),
                obs_id=self.device,
                observation_mode_revision=self.mode_revision,
                mode=b.mode,
                actual_state=pb.MOTION_STATE_STOPPED,
                mode_applied_at=utc(),
            )
        elif kind == "obs_motion_command" and self.role == "OBS":
            reason = None
            if (
                b.obs_control_session_id != self.control
                or b.control_epoch != self.epoch
                or b.command_sequence <= self.sequence
            ):
                reason = pb.REASON_CODE_STALE_COMMAND
            elif b.operation == pb.ObsMotionCommand.OPERATION_POINT:
                if self.collection or not self.gate:
                    reason = pb.REASON_CODE_BUSY
                elif (
                    not b.HasField("pan_deg")
                    or not b.HasField("tilt_deg")
                    or not -90 <= b.pan_deg <= 90
                    or not -45 <= b.tilt_deg <= 45
                ):
                    reason = pb.REASON_CODE_OUT_OF_RANGE
                elif stamp(b.expires_at) <= time.time():
                    reason = pb.REASON_CODE_EXPIRED_COMMAND
            if reason is None:
                self.sequence = b.command_sequence
                if b.operation == pb.ObsMotionCommand.OPERATION_POINT:
                    self.pan, self.tilt = b.pan_deg, b.tilt_deg
                elif b.operation == pb.ObsMotionCommand.OPERATION_STOP:
                    self.gate = False
            response = pb.ObsMotionResult(
                outcome=outcome(
                    b.request_id,
                    pb.REQUEST_RESULT_REJECTED if reason else pb.REQUEST_RESULT_COMPLETED,
                    reason,
                ),
                obs_id=self.device,
                obs_control_session_id=b.obs_control_session_id,
                control_epoch=b.control_epoch,
                command_sequence=b.command_sequence,
                actual_motion_state=pb.MOTION_STATE_STOPPED,
            )
        elif kind == "measurement_command" and self.role == "OBS":
            self.collection = (
                b.collection_session_id
                if b.operation == pb.MeasurementCommand.OPERATION_START
                else None
            )
            response = pb.MeasurementResult(
                outcome=outcome(b.request_id),
                collection_session_id=b.collection_session_id,
                actual_state=(
                    pb.MEASUREMENT_STATE_COLLECTING
                    if self.collection
                    else pb.MEASUREMENT_STATE_STOPPED
                ),
            )
        elif kind == "control_open" and self.role == "EFF":
            reason = pb.REASON_CODE_STALE_COMMAND if b.control_epoch <= self.epoch else None
            if b.installation_revision != self.revision:
                reason = pb.REASON_CODE_INSTALLATION_MISMATCH
            if reason is None:
                self.control, self.epoch, self.sequence, self.gate = (
                    b.control_session_id,
                    b.control_epoch,
                    0,
                    True,
                )
            response = pb.ControlResult(
                outcome=outcome(
                    b.request_id,
                    pb.REQUEST_RESULT_REJECTED if reason else pb.REQUEST_RESULT_COMPLETED,
                    reason,
                ),
                active_control_session_id=b.control_session_id,
                control_epoch=b.control_epoch,
            )
        elif kind == "pointing_command" and self.role == "EFF":
            reason = None
            if (
                b.control_session_id != self.control
                or b.control_epoch != self.epoch
                or b.command_sequence <= self.sequence
            ):
                reason = pb.REASON_CODE_STALE_COMMAND
            elif b.operation == pb.PointingCommand.OPERATION_POINT:
                if not self.gate:
                    reason = pb.REASON_CODE_STALE_COMMAND
                elif not b.HasField("expires_at") or stamp(b.expires_at) <= time.time():
                    reason = pb.REASON_CODE_EXPIRED_COMMAND
                elif (
                    not b.HasField("pan_deg")
                    or not b.HasField("tilt_deg")
                    or not -90 <= b.pan_deg <= 90
                    or not -45 <= b.tilt_deg <= 45
                ):
                    reason = pb.REASON_CODE_OUT_OF_RANGE
            elif b.HasField("pan_deg") or b.HasField("tilt_deg"):
                reason = pb.REASON_CODE_INVALID_MESSAGE
            if reason is None:
                self.sequence = b.command_sequence
                self.commands.append(b)
                if b.operation == pb.PointingCommand.OPERATION_POINT:
                    self.pan, self.tilt = b.pan_deg, b.tilt_deg
                    self.point_applications += 1
                elif b.operation == pb.PointingCommand.OPERATION_STOP:
                    self.gate = False
                    self.stop_applications += 1
            response = pb.PointingResult(
                outcome=outcome(
                    b.request_id,
                    pb.REQUEST_RESULT_REJECTED if reason else pb.REQUEST_RESULT_COMPLETED,
                    reason,
                ),
                control_session_id=b.control_session_id,
                control_epoch=b.control_epoch,
                command_sequence=b.command_sequence,
                actual_motion_state=pb.MOTION_STATE_STOPPED,
            )
        if response is None:
            return
        if (
            self.accept_only
            and kind == "pointing_command"
            and b.operation == pb.PointingCommand.OPERATION_POINT
        ):
            response.outcome.CopyFrom(outcome(b.request_id, pb.REQUEST_RESULT_ACCEPTED))
        self.cache[b.request_id] = (raw, response, time.monotonic())
        if (
            kind == "pointing_command"
            and b.operation == pb.PointingCommand.OPERATION_STOP
            and self.drop_stop_results
        ):
            return
        if (
            kind == "pointing_command"
            and b.operation == pb.PointingCommand.OPERATION_POINT
            and self.drop_first_point_result
        ):
            self.drop_first_point_result = False
            return
        self.send(response)
        self.status()

    def raw_frame(self):
        self.frame += 1
        at = math.floor((time.time() - 0.003) * 1000) / 1000
        pose = pb.ObsPose(
            obs_id=self.device,
            pose_id=uid(),
            captured_at=iso(at),
            observation_mode_revision=self.mode_revision,
            installation_revision=self.revision,
            pan_deg=self.pan,
            tilt_deg=self.tilt,
            pan_measured=True,
            tilt_measured=True,
            motion_state=pb.MOTION_STATE_STOPPED,
            pose_valid=True,
            collection_session_id=self.collection,
        )
        self.send(pose)
        chunks = []
        for chunk in range(6):
            c = pb.PointCloudChunk(
                collection_session_id=self.collection,
                frame_id=self.frame,
                source_frame_id=self.frame,
                captured_start_at=iso(at - 0.001),
                captured_at=iso(at),
                captured_end_at=iso(at + 0.001),
                installation_revision=self.revision,
                observation_mode_revision=self.mode_revision,
                pose_id=pose.pose_id,
                width=8,
                height=6,
                sensor_data_profile_id="synthetic-organized-v1",
                chunk_index=chunk,
                chunk_count=6,
                total_points=48,
                motion_compensation=pb.PointCloudChunk.MOTION_COMPENSATION_NONE,
                sdk_time=pb.SdkFrameTime(
                    seconds=int(at),
                    nanoseconds=round((at - int(at)) * 1e9),
                    sync_type=2,
                    sync_state=1,
                    frame_protocol_version=3,
                ),
                clock_mapping_id="synthetic-clock-v1",
                clock_uncertainty_ms=1,
            )
            for i in range(chunk * 8, chunk * 8 + 8):
                p = c.points.add(
                    pixel_index=i,
                    sdk_depth_raw=3000 if not self.empty else 0,
                    xyz_valid=not self.empty,
                )
                if not self.empty:
                    p.x = 0.2 + (i % 8 - 3.5) * 0.02 + 0.02 * math.sin(self.frame * 0.1)
                    p.y = 0.1 + (i // 8 - 2.5) * 0.02
                    p.z = 3
            if self.frame_mutator:
                self.frame_mutator(c)
            chunks.append(c)
        self.last_chunks = chunks
        for c in reversed(chunks) if self.reverse_chunks else chunks:
            self.send(c)
            if self.duplicate_chunks:
                self.send(c)

    def run(self):
        last_register = last_status = last_frame = 0
        while self.running:
            try:
                data, source = self.sock.recvfrom(65536)
                if source == self.c2:
                    with self.lock:
                        self.receive(data)
            except socket.timeout:
                pass
            except ConnectionResetError:
                # Windows reports a closed remote UDP port via ICMP as 10054.
                # Liveness still expires via monotonic heartbeat timeout below.
                pass
            except Exception as error:
                self.errors.append(repr(error))
            with self.lock:
                now = time.monotonic()
                if not self.session:
                    if now - last_register >= 0.2:
                        self.sock.sendto(self.register_packet, self.c2)
                        last_register = now
                    continue
                if now - self.c2_alive > 2:
                    self.collection, self.gate = None, False
                if now - last_status >= 0.5:
                    if not self.revision:
                        self.send(self.capability())
                    self.hb += 1
                    self.send(pb.Heartbeat(boot_id=self.boot, sequence=self.hb))
                    self.status()
                    last_status = now
                if (
                    self.role == "OBS"
                    and self.collection
                    and self.frames_enabled
                    and now - last_frame >= 0.12
                ):
                    self.raw_frame()
                    last_frame = now
                self.cache = {k: v for k, v in self.cache.items() if now - v[2] < 60}

    def close(self):
        self.running = False
        self.thread.join(timeout=2)
        self.sock.close()
