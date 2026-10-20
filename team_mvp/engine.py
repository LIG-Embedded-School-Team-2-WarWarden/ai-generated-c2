"""No-GUI, fixed-OBS educational pointer runtime using the shared ICD 1.0."""
import collections
import datetime as dt
import ipaddress
import math
import socket
import threading
import time
import uuid
from google.protobuf.message import DecodeError
from .contract import pb, uid, utc, stamp, envelope, encode, decode, current_angles, finite
from .cloud import Cloud
from .geometry import validate_installation, angles


class Engine:
    def __init__(self, config, detector, bind="127.0.0.1", port=5000):
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
        self.peers, self.pending, self.controls = {}, {}, {}
        self.observer_action = None
        self.registrations = {}
        self.retired_boots = collections.defaultdict(set)
        self.events = collections.deque(maxlen=256)
        self.cloud = Cloud(detector, self.publish)
        self.previous = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind, port))
        self.sock.settimeout(.02)
        self.address = self.sock.getsockname()
        self.running = True
        self.last_hb = 0
        self.hb_sequence = 0
        self.last_clock = (time.time(),time.monotonic())
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def log(self, event, **values):
        self.events.append(dict(event=event, **values))

    def send(self, peer, body):
        packet = encode(envelope("C2", peer["id"], body, peer["session"]))
        self.sock.sendto(packet, peer["address"])
        return packet

    def request(self, peer, body, result_kind, callback, timeout=2):
        packet = self.send(peer, body)
        now = time.monotonic()
        self.pending[body.request_id] = dict(peer=peer["id"], session=peer["session"],
            body=body, kind=result_kind, callback=callback, packet=packet,
            sent=now, started=now, attempts=1, accepted=False, timeout=timeout)
        self.log("request", device=peer["id"], kind=body.DESCRIPTOR.name,
                 request=body.request_id, sequence=getattr(body,"command_sequence",0))

    def register(self, e, address):
        r = e.register_request
        uuid.UUID(r.request_id)
        uuid.UUID(r.boot_id)
        cfg = self.config["devices"].get(r.device_id)
        if not cfg or e.sender_id != r.device_id or r.role != pb.Role.Value("ROLE_"+cfg["role"]):
            raise ValueError("unlisted device/role")
        if not 1 <= r.receive_port <= 65535:
            raise ValueError("invalid receive port")
        if self.config.get("simulation") and not ipaddress.ip_address(address[0]).is_loopback:
            raise ValueError("simulation allows loopback peers only")
        key = (r.device_id, r.request_id)
        raw = r.SerializeToString(deterministic=True)
        cached = self.registrations.get(key)
        if cached:
            if cached[0] != raw:
                from .contract import outcome
                reply = pb.RegisterResult(outcome=outcome(r.request_id,pb.REQUEST_RESULT_REJECTED,pb.REASON_CODE_ID_CONFLICT),
                    device_id=r.device_id,boot_id=r.boot_id,c2_boot_id=self.boot)
                self.sock.sendto(encode(envelope("C2",r.device_id,reply)),(address[0],r.receive_port))
                return
        p = self.peers.get(r.device_id)
        if r.boot_id in self.retired_boots[r.device_id]:
            raise ValueError("retired boot registration")
        if p and p["boot"] == r.boot_id:
            if p["address"] != (address[0],r.receive_port):
                raise ValueError("same boot changed endpoint")
        else:
            if p:
                self.retired_boots[r.device_id].add(p["boot"])
                self.stop_all("DEVICE_RESTART")
                self.pending = {k:v for k,v in self.pending.items() if v["peer"] != r.device_id}
                if r.device_id == self.obs_id:
                    self.cloud.reset()
                    self.previous = None
                self.controls.pop(r.device_id, None)
            p = dict(id=r.device_id,boot=r.boot_id,session=uid(),address=(address[0],r.receive_port),
                     hb=0,alive=time.monotonic(),sample=0,status=None,status_received=0,
                     cap=None,installed=False,install_result=False,install_sent=False,
                     mode_sent=False,mode=False,mode_revision=1,obs_epoch=1,obs_sequence=0,obs_session=None,collection=None,collecting=False,poses={},
                     mapping=None,epoch=0,disconnected=False)
            self.peers[r.device_id] = p
        from .contract import outcome
        reply = pb.RegisterResult(outcome=outcome(r.request_id),device_id=r.device_id,
            boot_id=r.boot_id,connection_session_id=p["session"],c2_boot_id=self.boot)
        self.sock.sendto(encode(envelope("C2",r.device_id,reply)),p["address"])
        self.registrations[key] = (raw, time.monotonic())

    def receive(self, data, address):
        e = decode(data, "C2")
        kind = e.WhichOneof("body")
        if kind == "register_request":
            self.register(e,address)
            return
        p = self.peers.get(e.sender_id)
        if not p or p["session"] != e.connection_session_id or p["address"][0] != address[0]:
            if p and p["address"][0] == address[0]:
                self.send(p,pb.SessionRequired(device_id=p["id"],c2_boot_id=self.boot))
            raise ValueError("unknown session/source")
        b = getattr(e,kind)
        if kind == "heartbeat":
            if b.boot_id != p["boot"] or b.sequence <= p["hb"]:
                return
            p["hb"], p["alive"] = b.sequence, time.monotonic()
        elif kind == "device_status":
            if b.device_id != p["id"] or b.angles.sample_sequence <= p["sample"]:
                return
            stamp(b.reported_at)
            p["sample"], p["status"], p["status_received"] = b.angles.sample_sequence, b, time.monotonic()
            p["alive"] = time.monotonic()
            cfg = self.config["devices"][p["id"]]
            if p["install_result"] and b.active_installation_revision == cfg["revision"]:
                p["installed"] = True
            self.configure_peer(p)
        elif kind == "capability_report":
            if b.device_id != p["id"]:
                raise ValueError("capability device mismatch")
            if p["cap"] is not None and p["cap"] != b:
                self.stop_all("CAPABILITY_CHANGED")
                raise ValueError("re-register after capability change")
            p["cap"] = b
            self.configure_peer(p)
        elif kind.endswith("_result"):
            self.result(p,kind,b)
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
            if not p["installed"] or not p["collecting"] or b.collection_session_id != p["collection"]:
                return
            verified = self.verify_frame(p,b)
            self.cloud.ingest(p["session"],b,verified)
        elif kind == "error_report":
            self.stop_all("DEVICE_ERROR")

    def status_valid(self,p,stopped=False):
        s = p["status"]
        limit = self.config["max_clock_uncertainty_ms"]
        if not s or time.monotonic()-p["status_received"] > 1 or time.monotonic()-p["alive"] > 2:
            return False
        try:
            return (s.device_state in (pb.DEVICE_STATE_IDLE,pb.DEVICE_STATE_BUSY)
                and (not p["install_result"] or s.active_installation_revision == self.config["devices"][p["id"]]["revision"])
                and not s.HasField("fault_code")
                and s.estop_latch_state == pb.EMERGENCY_LATCH_STATE_RELEASED
                and (not s.HasField("estop_latched") or not s.estop_latched)
                and s.clock_state == pb.CLOCK_STATE_SYNCED
                and s.HasField("clock_uncertainty_ms") and finite(s.clock_uncertainty_ms)
                and 0 <= s.clock_uncertainty_ms <= limit
                and 0 <= time.time()-stamp(s.reported_at) <= 1
                and current_angles(s.angles,time.time())
                and (not stopped or s.motion_state == pb.MOTION_STATE_STOPPED))
        except ValueError:
            return False

    def configure_peer(self,p):
        cap, cfg = p["cap"], self.config["devices"][p["id"]]
        if not cap or not self.status_valid(p,True):
            return
        if cap.calibration_id != cfg["calibration_id"]:
            return
        if cfg["role"] == "OBS" and (not cap.organized_frame_supported
                or cap.sensor_data_profile_id != cfg["profile"]
                or not cap.pose_feedback_supported or not cap.lidar_measurement_stop_supported
                or pb.OBSERVATION_MODE_FIXED_VIEW not in cap.supported_observation_modes):
            return
        if not p["install_sent"]:
            p["install_sent"] = True
            body = pb.InstallationSet(request_id=uid(),device_id=p["id"],
                installation_revision=cfg["revision"],mount_rotation=cfg["rotation"],
                axis_calibration_id=cfg["calibration_id"],
                position_m=pb.Position(**dict(zip(("x","y","z"),cfg["position"]))),
                sensor_offset_m=pb.Position(**dict(zip(("x","y","z"),cfg["sensor_offset"]))))
            def installed(result):
                if result and result.outcome.result == pb.REQUEST_RESULT_COMPLETED and result.device_id == p["id"] and result.active_installation_revision == cfg["revision"]:
                    p["install_result"] = True
                    # A later DEVICE_STATUS, never an earlier one, activates it.
            self.request(p,body,"installation_result",installed)
        if p["installed"] and cfg["role"] == "OBS" and not p["mode_sent"]:
            p["mode_sent"] = True
            body = pb.ObsModeCommand(request_id=uid(),obs_control_session_id=uid(),
                control_epoch=1,observation_mode_revision=1,mode=pb.OBSERVATION_MODE_FIXED_VIEW,
                installation_revision=cfg["revision"])
            p["obs_session"] = body.obs_control_session_id
            def mode(result):
                if (result and result.outcome.result == pb.REQUEST_RESULT_COMPLETED and result.obs_id == p["id"]
                    and result.observation_mode_revision == 1 and result.mode == pb.OBSERVATION_MODE_FIXED_VIEW
                    and result.actual_state == pb.MOTION_STATE_STOPPED):
                    p["mode"] = True
                    p["mode_applied"] = stamp(result.mode_applied_at)
                    self.collect(True)
            self.request(p,body,"obs_mode_result",mode)

    def collect(self,start,_observer=False):
        with self.lock:
            if not _observer and self.observer_action and self.observer_action["state"] not in ("COMPLETED","FAILED"):
                raise ValueError("OBS operation pending; use obs stop to cancel")
            p = self.peers.get(self.obs_id)
            if not p or not p["installed"] or not p["mode"]:
                raise ValueError("OBS installation/mode not ready")
            if any(v["peer"] == self.obs_id and v["kind"] == "measurement_result" for v in self.pending.values()):
                raise ValueError("measurement request pending")
            if start and p["collecting"]:
                return
            if start:
                if not self.status_valid(p,True):
                    raise ValueError("OBS must be stopped with current measured angles")
                p["collection"] = uid()
                # frameId high-water is connection-scoped; do not reset it here.
                self.cloud.fail("NO_DATA")
                self.previous = None
            else:
                self.stop_all("COLLECTION_STOP")
                # Never report measurement complete on transmission alone.
                p["collecting"] = False
                self.cloud.fail("NO_DATA")
            body = pb.MeasurementCommand(request_id=uid(),operation=pb.MeasurementCommand.OPERATION_START if start else pb.MeasurementCommand.OPERATION_STOP,
                collection_session_id=p["collection"],installation_revision=self.config["devices"][self.obs_id]["revision"])
            p["measurement_result"] = "PENDING"
            def applied(result):
                expected = pb.MEASUREMENT_STATE_COLLECTING if start else pb.MEASUREMENT_STATE_STOPPED
                p["collecting"] = bool(start and result and result.outcome.result == pb.REQUEST_RESULT_COMPLETED
                    and result.collection_session_id == p["collection"] and result.actual_state == expected)
                p["measurement_result"] = "COMPLETED" if result and result.outcome.result == pb.REQUEST_RESULT_COMPLETED and result.actual_state == expected else "UNCONFIRMED"
            self.request(p,body,"measurement_result",applied)

    def obs_point(self,pan,tilt):
        """Stop EFF/collection, move OBS, confirm stopped feedback, then collect."""
        with self.lock:
            p = self.peers.get(self.obs_id)
            if not p or not p["installed"] or not p["mode"] or not self.status_valid(p,True):
                raise ValueError("OBS not ready with current stopped feedback")
            cap = p["cap"]
            if not finite(pan,tilt) or not cap.pan_min_deg <= pan <= cap.pan_max_deg or not cap.tilt_min_deg <= tilt <= cap.tilt_max_deg:
                raise ValueError("OBS angles outside reported limits")
            if not cap.HasField("point_completion_timeout_ms") or not 0 < cap.point_completion_timeout_ms <= 60000:
                raise ValueError("OBS completion timeout missing")
            if self.observer_action and self.observer_action["state"] not in ("COMPLETED","FAILED"):
                raise ValueError("OBS operation pending")
            if any(v["peer"]==self.obs_id for v in self.pending.values()):
                raise ValueError("OBS request pending")
            self.stop_all("OBS_MOVE")
            self.cloud.fail("OBS_MOVING")
            self.previous = None
            self.observer_action = dict(state="WAIT_EFF_STOP",session=p["session"],pan=pan,tilt=tilt,
                started=time.monotonic(),reason="",operation="POINT")

    def obs_stop(self):
        with self.lock:
            p = self.peers.get(self.obs_id)
            if not p or not p["installed"] or not p["mode"]:
                raise ValueError("OBS not installed/mode ready")
            self.stop_all("OBS_STOP")
            self.pending = {k:v for k,v in self.pending.items() if not (v["peer"]==self.obs_id and v["kind"] in ("obs_motion_result","obs_mode_result"))}
            self.cloud.fail("OBS_STOP")
            self.previous = None
            self.observer_action = dict(state="STOPPING",session=p["session"],started=time.monotonic(),reason="",operation="STOP")
            self._obs_motion(p,self.observer_action,False)

    def _obs_motion(self,p,a,point):
        p["obs_sequence"] += 1
        q = pb.ObsMotionCommand(request_id=uid(),obs_control_session_id=p["obs_session"],control_epoch=p["obs_epoch"],
            command_sequence=p["obs_sequence"],operation=pb.ObsMotionCommand.OPERATION_POINT if point else pb.ObsMotionCommand.OPERATION_STOP,issued_at=utc())
        if point:
            q.pan_deg,q.tilt_deg = a["pan"],a["tilt"]
            q.purpose = pb.ObsMotionCommand.PURPOSE_FIXED_POSITION
            q.installation_revision = self.config["devices"][self.obs_id]["revision"]
            q.observation_mode_revision = p["mode_revision"]
            q.expires_at = (dt.datetime.now(dt.timezone.utc)+dt.timedelta(seconds=1)).isoformat(timespec="milliseconds").replace("+00:00","Z")
        a["sample"],a["ack"] = p["sample"],False
        a["state"] = "MOVING" if point else "STOPPING"
        def completed(result):
            if self.observer_action is not a:
                return
            if result and result.outcome.result==pb.REQUEST_RESULT_COMPLETED and result.actual_motion_state==pb.MOTION_STATE_STOPPED:
                a["ack"] = True
            else:
                a["state"],a["reason"] = "FAILED","OBS_MOTION_UNCONFIRMED"
        field = "point_completion_timeout_ms" if point else "stop_completion_timeout_ms"
        self.request(p,q,"obs_motion_result",completed,max(.2,getattr(p["cap"],field)/1000))

    def update_observer(self,now):
        a = self.observer_action
        if not a or a["state"] in ("COMPLETED","FAILED"):
            return
        p = self.peers.get(self.obs_id)
        if not p or p["session"]!=a["session"] or now-p["alive"]>2:
            a["state"],a["reason"] = "FAILED","OBS_DISCONNECTED"
            return
        if now-a["started"]>max(10,p["cap"].point_completion_timeout_ms/1000+10):
            a["state"],a["reason"] = "FAILED","OBS_OPERATION_TIMEOUT"
            return
        state = a["state"]
        if state=="WAIT_EFF_STOP":
            if any(c["state"]!="STOPPED" for c in self.controls.values()):
                return
            a["sample"] = p["sample"]
            if p["collecting"]:
                self.collect(False,_observer=True)
            a["state"] = "WAIT_COLLECTION_STOP"
        elif state=="WAIT_COLLECTION_STOP":
            if any(v["peer"]==self.obs_id and v["kind"]=="measurement_result" for v in self.pending.values()):
                return
            if p.get("measurement_result")=="UNCONFIRMED":
                a["state"],a["reason"] = "FAILED","COLLECTION_STOP_UNCONFIRMED"
                return
            if not self.status_valid(p,True) or p["status"].measurement_state!=pb.MEASUREMENT_STATE_STOPPED:
                return
            if a["operation"]=="STOP":
                a["state"]="COMPLETED"
                return
            p["obs_epoch"]+=1
            p["mode_revision"]+=1
            p["obs_sequence"]=0
            p["obs_session"]=uid()
            q=pb.ObsModeCommand(request_id=uid(),obs_control_session_id=p["obs_session"],control_epoch=p["obs_epoch"],
                observation_mode_revision=p["mode_revision"],mode=pb.OBSERVATION_MODE_FIXED_VIEW,
                installation_revision=self.config["devices"][self.obs_id]["revision"])
            a["state"]="OPENING"
            def opened(result):
                if self.observer_action is not a:
                    return
                if not result or result.outcome.result!=pb.REQUEST_RESULT_COMPLETED or result.obs_id!=self.obs_id or result.observation_mode_revision!=p["mode_revision"] or result.mode!=pb.OBSERVATION_MODE_FIXED_VIEW or result.actual_state!=pb.MOTION_STATE_STOPPED:
                    a["state"],a["reason"]="FAILED","OBS_MODE_UNCONFIRMED"
                    return
                p["mode_applied"]=stamp(result.mode_applied_at)
                self._obs_motion(p,a,True)
            self.request(p,q,"obs_mode_result",opened)
        elif state in ("MOVING","STOPPING"):
            if not a["ack"] or p["sample"]<=a["sample"] or not self.status_valid(p,True):
                return
            if state=="STOPPING":
                if any(v["peer"]==self.obs_id and v["kind"]=="measurement_result" for v in self.pending.values()):
                    return
                if p["status"].measurement_state!=pb.MEASUREMENT_STATE_STOPPED:
                    self.collect(False,_observer=True)
                a["state"]="WAIT_COLLECTION_STOP"
            else:
                self.collect(True,_observer=True)
                a["state"]="COLLECTING"
        elif state=="COLLECTING":
            if p.get("measurement_result")=="UNCONFIRMED":
                a["state"],a["reason"]="FAILED","COLLECTION_START_UNCONFIRMED"
            elif p["collecting"] and p["status"].measurement_state==pb.MEASUREMENT_STATE_COLLECTING:
                a["state"]="COMPLETED"

    def trust_clock(self,device,mapping):
        """Operator supplied, verified mapping tied to this exact connection and boot."""
        with self.lock:
            p = self.peers[device]
            if mapping["boot_id"] != p["boot"] or mapping["connection_session_id"] != p["session"]:
                raise ValueError("clock evidence is for another boot/session")
            if not mapping["evidence"] or not finite(mapping["offset_seconds"],mapping["uncertainty_ms"]) or not 0 <= mapping["uncertainty_ms"] <= self.config["max_clock_uncertainty_ms"]:
                raise ValueError("clock evidence/uncertainty required")
            p["mapping"] = dict(mapping)

    def trust_acquisition_profile(self,device,profile):
        """Bind externally measured sensor evidence to the currently connected process."""
        with self.lock:
            p=self.peers[device]
            if profile["sensor_profile"]!=self.config["devices"][device]["profile"]:
                raise ValueError("sensor profile mismatch")
            start,end=stamp(profile["valid_from"]),stamp(profile["valid_until"])
            if not start<=time.time()<end:
                raise ValueError("acquisition evidence expired/not active")
            mapping={k:profile[k] for k in ("id","evidence","offset_seconds","uncertainty_ms","sync_type","sync_state","frame_protocol_version")}
            if not isinstance(mapping["id"],str) or not 0<len(mapping["id"])<=64 or any(type(mapping[k]) is not int or not 0<=mapping[k]<=255 for k in ("sync_type","sync_state","frame_protocol_version")):
                raise ValueError("invalid mapping ID or SDK time identifiers")
            if any(type(mapping[k]) not in (int,float) or not finite(mapping[k]) for k in ("offset_seconds","uncertainty_ms")) or mapping["uncertainty_ms"]<1:
                raise ValueError("measured finite offset/uncertainty required")
            mapping.update(boot_id=p["boot"],connection_session_id=p["session"],valid_from=start,valid_until=end)
            self.trust_clock(device,mapping)

    def verify_frame(self,p,c):
        cfg = self.config["devices"][p["id"]]
        if c.installation_revision != cfg["revision"] or c.observation_mode_revision != p["mode_revision"] or c.sensor_data_profile_id != cfg["profile"]:
            raise ValueError("frame installation/mode/profile mismatch")
        pose = p["poses"].get(c.pose_id)
        try:
            if not pose or not (pose.obs_id == p["id"] and pose.collection_session_id == p["collection"]
                and pose.installation_revision == c.installation_revision and pose.observation_mode_revision == p["mode_revision"]
                and pose.pose_valid and pose.pan_measured and pose.tilt_measured
                and pose.HasField("pan_deg") and pose.HasField("tilt_deg")
                and finite(pose.pan_deg,pose.tilt_deg)
                and abs(pose.pan_deg) <= .001 and abs(pose.tilt_deg) <= .001
                and pose.motion_state == pb.MOTION_STATE_STOPPED and pose.captured_at == c.captured_at):
                return False
            start,at,end = [stamp(getattr(c,k)) for k in ("captured_start_at","captured_at","captured_end_at")]
            if not p["mode_applied"] <= start <= at <= end <= time.time() or time.time()-start > 1:
                return False
            if not c.HasField("sdk_time") or c.sdk_time.nanoseconds >= 1000000000:
                return False
            mapping = p["mapping"]
            if self.config.get("simulation") and cfg["profile"] == "synthetic-organized-v1":
                mapping = dict(id="synthetic-clock-v1",offset_seconds=0,uncertainty_ms=1,
                               sync_type=2,sync_state=1,frame_protocol_version=3)
            if not mapping or c.clock_mapping_id != mapping["id"]:
                return False
            if "valid_until" in mapping and not mapping["valid_from"]<=start<=end<=time.time()<=mapping["valid_until"]:
                return False
            if (c.sdk_time.sync_type,c.sdk_time.sync_state,c.sdk_time.frame_protocol_version) != (mapping["sync_type"],mapping["sync_state"],mapping["frame_protocol_version"]):
                return False
            if not c.HasField("clock_uncertainty_ms") or not finite(c.clock_uncertainty_ms) or not mapping["uncertainty_ms"] <= c.clock_uncertainty_ms <= self.config["max_clock_uncertainty_ms"]:
                return False
            mapped = c.sdk_time.seconds+c.sdk_time.nanoseconds/1e9+mapping["offset_seconds"]
            return abs(mapped-at) <= .0011
        except (ValueError,KeyError):
            return False

    def result(self,p,kind,b):
        out = b.outcome
        pending = self.pending.get(out.request_id)
        if not pending or pending["peer"] != p["id"] or pending["session"] != p["session"] or pending["kind"] != kind:
            return
        request = pending["body"]
        if kind == "control_result" and (b.active_control_session_id != request.control_session_id or b.control_epoch != request.control_epoch):
            return
        if kind == "obs_motion_result" and (b.obs_id != p["id"] or (b.obs_control_session_id,b.control_epoch,b.command_sequence) != (request.obs_control_session_id,request.control_epoch,request.command_sequence)):
            return
        if kind == "pointing_result" and (b.control_session_id,b.control_epoch,b.command_sequence) != (request.control_session_id,request.control_epoch,request.command_sequence):
            return
        if out.HasField("detail") and len(out.detail.encode("utf-8")) > 256:
            return
        stamp(out.reported_at)
        if out.result == pb.REQUEST_RESULT_ACCEPTED:
            pending["accepted"] = True
            return
        if out.result not in (pb.REQUEST_RESULT_COMPLETED,pb.REQUEST_RESULT_REJECTED,pb.REQUEST_RESULT_FAILED,pb.REQUEST_RESULT_CANCELLED) or not out.HasField("completed_at"):
            return
        stamp(out.completed_at)
        if out.result != pb.REQUEST_RESULT_COMPLETED and not out.HasField("reason_code"):
            return
        del self.pending[out.request_id]
        self.log("result",device=p["id"],kind=kind,result=pb.RequestResult.Name(out.result))
        pending["callback"](b)

    def publish(self,generation,session,meta,objects,verified,error):
        with self.lock:
            if generation != self.cloud.generation or session != self.cloud.key:
                return
            if error:
                self.cloud.fail("DETECTOR_ERROR")
                return
            if self.previous and len(objects) == 1 and time.monotonic()-self.previous[1] <= 1:
                old = self.previous[0]
                if math.dist([old[k] for k in ("x","y","z")],[objects[0][k] for k in ("x","y","z")]) <= .5:
                    objects[0]["id"] = old["id"]
            self.previous = (objects[0],time.monotonic()) if len(objects) == 1 and verified else None
            self.cloud.result = dict(meta=meta,objects=objects,verified=verified,received=time.monotonic())
            self.cloud.state = "CURRENT" if verified else "DISPLAY_ONLY"
            if verified:
                for c in self.controls.values():
                    if len(objects)==1 and objects[0]["id"]==c["object"]:
                        c["last_observation"] = time.monotonic()

    def latest(self):
        r = self.cloud.result
        mapping=self.peers.get(self.obs_id,{}).get("mapping")
        if mapping and "valid_until" in mapping and time.time()>mapping["valid_until"]:
            raise ValueError("acquisition evidence expired")
        if self.cloud.state != "CURRENT" or not r or not r["verified"]:
            raise ValueError("no verified current observation")
        if time.monotonic()-r["received"] > 1 or not 0 <= time.time()-stamp(r["meta"].captured_start_at) <= 1:
            raise ValueError("observation stale")
        return r

    def aim(self,device,object_id=None):
        self.begin_control(device,object_id)

    def point(self,device,pan,tilt):
        if not finite(pan,tilt):
            raise ValueError("finite angles required")
        self.begin_control(device,manual=(pan,tilt))

    def begin_control(self,device,object_id=None,manual=None):
        with self.lock:
            p = self.peers[device]
            cfg = self.config["devices"][device]
            cap = p["cap"]
            if cfg["role"] != "EFF" or not p["installed"] or not self.status_valid(p,True):
                raise ValueError("EFF not ready/stopped with current measured angles")
            if not cap or not all((cap.absolute_pointing,cap.measured_angle_feedback,cap.stop_supported,cap.drain_supported)):
                raise ValueError("EFF capabilities missing")
            for field in ("point_completion_timeout_ms","stop_completion_timeout_ms","pan_min_deg","pan_max_deg","tilt_min_deg","tilt_max_deg"):
                if not cap.HasField(field) or not finite(getattr(cap,field)):
                    raise ValueError("EFF timeouts/limits missing")
            if not 0 < cap.point_completion_timeout_ms <= 60000 or not 0 < cap.stop_completion_timeout_ms <= 60000 or cap.pan_min_deg >= cap.pan_max_deg or cap.tilt_min_deg >= cap.tilt_max_deg:
                raise ValueError("invalid EFF timeouts/limits")
            old = self.controls.get(device)
            if old and old["state"] != "STOPPED":
                raise ValueError("stop and confirm previous control first")
            selected = None
            if manual is None:
                r = self.latest()
                if len(r["objects"]) != 1 or (object_id and r["objects"][0]["id"] != object_id):
                    raise ValueError("MVP requires one unambiguous object")
                selected = r["objects"][0]["id"]
            elif not cap.pan_min_deg <= manual[0] <= cap.pan_max_deg or not cap.tilt_min_deg <= manual[1] <= cap.tilt_max_deg:
                raise ValueError("OUT_OF_RANGE")
            p["epoch"] += 1
            control = dict(session=uid(),epoch=p["epoch"],sequence=0,state="OPENING",
                object=selected,last_send=0,last_frame=0,reason="",stop_ack=False,
                last_observation=time.monotonic(),manual=manual)
            self.controls[device] = control
            body = pb.ControlOpen(request_id=uid(),control_session_id=control["session"],
                control_epoch=control["epoch"],installation_revision=cfg["revision"],mode=pb.CONTROL_MODE_AUTO if manual is None else pb.CONTROL_MODE_MANUAL)
            def opened(result):
                if control["state"] != "OPENING":
                    return
                if result and result.outcome.result == pb.REQUEST_RESULT_COMPLETED:
                    control["state"] = "RUNNING" if manual is None else "MANUAL"
                    if manual is not None:
                        self.send_manual(p,control)
                else:
                    self.stop(device,"CONTROL_UNCONFIRMED")
            self.request(p,body,"control_result",opened)

    def send_manual(self,p,c):
        if not self.status_valid(p):
            self.stop(p["id"],"DEVICE_UNAVAILABLE")
            return
        c["sequence"] += 1
        expiry = (dt.datetime.now(dt.timezone.utc)+dt.timedelta(seconds=1)).isoformat(timespec="milliseconds").replace("+00:00","Z")
        body = pb.PointingCommand(request_id=uid(),control_session_id=c["session"],
            control_epoch=c["epoch"],command_sequence=c["sequence"],operation=pb.PointingCommand.OPERATION_POINT,
            issued_at=utc(),expires_at=expiry,pan_deg=c["manual"][0],tilt_deg=c["manual"][1],
            effector_installation_revision=self.config["devices"][p["id"]]["revision"],origin=pb.CONTROL_MODE_MANUAL)
        def pointed(result):
            if not result or result.outcome.result != pb.REQUEST_RESULT_COMPLETED:
                self.stop(p["id"],"POINT_UNCONFIRMED")
            elif c["state"]=="MANUAL":
                c["reason"] = "POINT_COMPLETED"
        self.request(p,body,"pointing_result",pointed,p["cap"].point_completion_timeout_ms/1000)

    def stop(self,device,reason="OPERATOR_STOP"):
        with self.lock:
            c = self.controls.get(device)
            if not c or c["state"] == "STOPPED" or (c["state"]=="STOPPING" and c["reason"]!="STOP_UNCONFIRMED"):
                return
            p = self.peers[device]
            c["state"],c["reason"] = "STOPPING",reason
            c["stop_ack"] = False
            self.pending = {k:v for k,v in self.pending.items() if not (v["peer"] == device and v["kind"] in ("control_result","pointing_result"))}
            c["sequence"] += 1
            body = pb.PointingCommand(request_id=uid(),control_session_id=c["session"],
                control_epoch=c["epoch"],command_sequence=c["sequence"],operation=pb.PointingCommand.OPERATION_STOP,issued_at=utc())
            c["stop_sample"] = p["sample"]
            def stopped(result):
                if result and result.outcome.result == pb.REQUEST_RESULT_COMPLETED and result.actual_motion_state == pb.MOTION_STATE_STOPPED:
                    c["stop_ack"] = True
                else:
                    c["reason"] = "STOP_UNCONFIRMED"
            timeout = p["cap"].stop_completion_timeout_ms/1000 if p["cap"] else 2
            self.request(p,body,"pointing_result",stopped,max(.2,timeout))

    def stop_all(self,reason):
        for device in list(self.controls):
            self.stop(device,reason)

    def update_control(self,device,c,now):
        p = self.peers[device]
        if c["state"] == "STOPPING":
            if c["stop_ack"] and p["sample"] > c["stop_sample"] and self.status_valid(p,True):
                c["state"] = "STOPPED"
            return
        if c.get("manual") is not None and c["state"] in ("MANUAL","OPENING"):
            if not self.status_valid(p):
                self.stop(device,"DEVICE_UNAVAILABLE")
            return
        if c["state"] not in ("RUNNING","OPENING"):
            return
        obs = self.peers.get(self.obs_id)
        if not self.status_valid(p) or not obs or not obs["collecting"] or time.monotonic()-obs["alive"] > 2:
            self.stop(device,"DEVICE_UNAVAILABLE")
            return
        if self.cloud.state in ("UPDATING","PROCESSING") and now-self.cloud.started < 1:
            if now-c["last_observation"] >= 1:
                self.stop(device,"OBSERVATION_STALE")
            return
        try:
            r = self.latest()
            if len(r["objects"]) != 1 or r["objects"][0]["id"] != c["object"]:
                raise ValueError("OBJECT_LOST_OR_AMBIGUOUS")
            pan,tilt = angles(r["objects"][0],self.config["devices"][self.obs_id],self.config["devices"][device])
            cap = p["cap"]
            if not cap.pan_min_deg <= pan <= cap.pan_max_deg or not cap.tilt_min_deg <= tilt <= cap.tilt_max_deg:
                raise ValueError("OUT_OF_RANGE")
        except ValueError as error:
            self.stop(device,str(error))
            return
        if c["state"] != "RUNNING" or now-c["last_send"] < .2 or r["meta"].frame_id == c["last_frame"]:
            return
        if any(v["peer"] == device and v["kind"] == "pointing_result" for v in self.pending.values()):
            return
        c["sequence"] += 1
        expires = (dt.datetime.now(dt.timezone.utc)+dt.timedelta(seconds=1)).isoformat(timespec="milliseconds").replace("+00:00","Z")
        m = r["meta"]
        body = pb.PointingCommand(request_id=uid(),control_session_id=c["session"],
            control_epoch=c["epoch"],command_sequence=c["sequence"],operation=pb.PointingCommand.OPERATION_POINT,
            issued_at=utc(),expires_at=expires,pan_deg=pan,tilt_deg=tilt,
            effector_installation_revision=self.config["devices"][device]["revision"],origin=pb.CONTROL_MODE_AUTO,
            collection_session_id=m.collection_session_id,object_id=c["object"],source_device_id=self.obs_id,
            source_connection_session_id=obs["session"],source_frame_id=m.frame_id,
            observation_installation_revision=m.installation_revision)
        def pointed(result):
            if not result or result.outcome.result != pb.REQUEST_RESULT_COMPLETED:
                self.stop(device,"POINT_UNCONFIRMED")
        self.request(p,body,"pointing_result",pointed,p["cap"].point_completion_timeout_ms/1000)
        c["last_send"],c["last_frame"] = now,m.frame_id
        self.log("point",device=device,pan=pan,tilt=tilt,frame=m.frame_id)

    def tick(self):
        now = time.monotonic()
        wall = time.time()
        if abs((wall-self.last_clock[0])-(now-self.last_clock[1])) > .25:
            for p in self.peers.values():
                p["mapping"] = None
            self.cloud.fail("CLOCK_CHANGED")
            self.previous = None
            self.stop_all("CLOCK_CHANGED")
        self.last_clock = wall,now
        if now-self.last_hb >= .5:
            self.last_hb = now
            self.hb_sequence += 1
            for p in self.peers.values():
                self.send(p,pb.Heartbeat(boot_id=self.boot,sequence=self.hb_sequence))
        for key,v in list(self.pending.items()):
            if key not in self.pending:
                continue
            if now-v["started"] >= v["timeout"] or (not v["accepted"] and v["attempts"] >= 3 and now-v["sent"] >= .2):
                del self.pending[key]
                v["callback"](None)
            elif not v["accepted"] and now-v["sent"] >= .2:
                p = self.peers[v["peer"]]
                if p["session"] == v["session"]:
                    self.sock.sendto(v["packet"],p["address"])
                v["attempts"] += 1
                v["sent"] = now
        self.registrations = {k:v for k,v in self.registrations.items() if now-v[1] < 60}
        if self.cloud.frame and now-self.cloud.started >= 1:
            self.cloud.fail("FRAME_INCOMPLETE")
        for device,c in list(self.controls.items()):
            self.update_control(device,c,now)
        self.update_observer(now)

    def snapshot(self):
        with self.lock:
            result = self.cloud.result
            state = self.cloud.state
            if state == "CURRENT":
                try:
                    self.latest()
                except ValueError:
                    state = "STALE"
            return dict(devices={key:dict(session=p["session"],boot=p["boot"],installed=p["installed"],
                connected=time.monotonic()-p["alive"] <= 2,sample=p["sample"],heartbeat=p["hb"],
                angles_current=self.status_valid(p),collecting=p["collecting"],
                measurement_result=p.get("measurement_result","PENDING"),
                pan=p["status"].angles.pan_deg if p["status"] and p["status"].angles.HasField("pan_deg") else None,
                tilt=p["status"].angles.tilt_deg if p["status"] and p["status"].angles.HasField("tilt_deg") else None)
                for key,p in self.peers.items()},
                observation=state,frame=result["meta"].frame_id if result else None,
                objects=result["objects"] if result and state in ("CURRENT","DISPLAY_ONLY") else [],
                controls={k:dict(v) for k,v in self.controls.items()},observer_action=dict(self.observer_action) if self.observer_action else None,pending=len(self.pending))

    def run(self):
        while self.running:
            try:
                data,address = self.sock.recvfrom(65536)
                with self.lock:
                    self.receive(data,address)
            except socket.timeout:
                pass
            except (ValueError,DecodeError,KeyError) as error:
                with self.lock:
                    self.log("rejected",reason=str(error))
            except OSError as error:
                if self.running:
                    self.log("socket_error",reason=str(error))
            try:
                with self.lock:
                    self.tick()
            except Exception as error:
                # Expose runtime faults and stop updates; never silently continue aiming.
                with self.lock:
                    self.log("runtime_error",reason=repr(error))
                    self.stop_all("RUNTIME_ERROR")

    def close(self):
        with self.lock:
            if self.observer_action and self.observer_action["state"] not in ("COMPLETED","FAILED"):
                self.obs_stop()
            self.stop_all("SHUTDOWN")
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            with self.lock:
                if all(c["state"] == "STOPPED" for c in self.controls.values()):
                    break
            time.sleep(.02)
        with self.lock:
            p = self.peers.get(self.obs_id)
            if p and p["collecting"]:
                self.collect(False,_observer=True)
        deadline = time.monotonic()+1
        while self.pending and time.monotonic() < deadline:
            time.sleep(.02)
        self.running = False
        self.thread.join(timeout=2)
        self.sock.close()
        self.cloud.close()
