"""Loopback-only equipment doubles for the native C++ GUI v2 integration test."""
import time
from .peers import Peer
from .contract import pb, decode, outcome, utc


class GuiObserver(Peer):
    def __init__(self, address, sweep=False):
        self.sweep = sweep
        self.sweep_started = None
        super().__init__("OBS", address)

    def capability(self):
        cap = super().capability()
        cap.pan_min_deg, cap.pan_max_deg = -135, 135
        return cap

    def send(self, body):
        if self.sweep and isinstance(body, pb.DeviceStatus):
            if self.collection:
                now = time.monotonic()
                if self.sweep_started is None:
                    self.sweep_started = now
                phase = (135 + (now - self.sweep_started) * 30) % 540
                self.pan = 135 - abs(phase - 270)
                body.angles.pan_deg = self.pan
                body.motion_state = pb.MOTION_STATE_MOVING
            else:
                self.sweep_started = None
        super().send(body)

    def raw_frame(self):
        if not self.sweep:
            super().raw_frame()


class GuiEffector(Peer):
    def __init__(self, address):
        self.laser_until = 0
        self.laser_sequence = 0
        self.laser_commands = 0
        self.laser_replies = {}
        self.drop_laser_results = False
        self.last_laser_request = None
        super().__init__("EFF", address, capabilities=[pb.CAPABILITY_LASER_CONTROL])

    def capability(self):
        cap = super().capability()
        cap.laser_control_supported = True
        return cap

    def laser_status(self):
        if not self.control:
            return
        on = self.gate and time.monotonic() < self.laser_until
        at = utc()
        s = pb.LaserStatus(device_id=self.device, control_session_id=self.control,
            control_epoch=self.epoch, last_accepted_sequence=self.laser_sequence,
            laser_state=pb.LASER_STATE_ON if on else pb.LASER_STATE_OFF,
            reported_at=at, state_observed_at=at)
        if on:
            s.active_request_id = self.last_laser_request
            s.remaining_duration_ms = max(1, int((self.laser_until-time.monotonic())*1000))
        super().send(s)

    def send(self, body):
        super().send(body)
        if isinstance(body, pb.DeviceStatus):
            self.laser_status()

    def receive(self, data):
        e = decode(data, self.device)
        if e.WhichOneof("body") != "laser_command":
            return super().receive(data)
        if e.sender_id != "C2" or e.connection_session_id != self.session:
            return
        q = e.laser_command
        if q.control_session_id != self.control or q.control_epoch != self.epoch:
            return
        if q.request_id not in self.laser_replies:
            if q.command_sequence <= self.laser_sequence:
                return
            self.laser_sequence = q.command_sequence
            self.laser_commands += 1
            self.last_laser_request = q.request_id
            self.laser_until = time.monotonic() + q.duration_ms/1000 if q.operation == pb.LaserCommand.OPERATION_ON else 0
            at = utc()
            self.laser_replies[q.request_id] = pb.LaserResult(outcome=outcome(q.request_id),
                control_session_id=self.control, control_epoch=self.epoch,
                command_sequence=q.command_sequence,
                actual_laser_state=pb.LASER_STATE_ON if self.laser_until else pb.LASER_STATE_OFF,
                state_observed_at=at)
        if not self.drop_laser_results:
            super().send(self.laser_replies[q.request_id])
        self.laser_status()
