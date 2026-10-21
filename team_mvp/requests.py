"""Request lifecycle service: exact retries, correlation and terminal callbacks.

The application serializes access; callbacks may cancel or enqueue other requests.
A local successful send is never treated as device completion.
"""
from .clock import Clock
from .transport import DatagramTransport
from .contract import pb, uid, stamp, envelope, encode


class RequestTracker:
    def __init__(self,clock:Clock,transport:DatagramTransport,log=lambda *a,**k:None):
        self.clock,self.transport,self.log=clock,transport,log
        self.pending={}

    def send(self,peer,body):
        packet=encode(envelope("C2",peer["id"],body,peer["session"],sent_at=self.clock.utc()))
        self.transport.sendto(packet,peer["address"])
        return packet

    def request(self,peer,body,result_kind,callback,timeout=2):
        # Freeze semantic identity along with the exact wire packet.
        snapshot=body.__class__()
        snapshot.CopyFrom(body)
        body=snapshot
        packet=self.send(peer,body)
        now=self.clock.monotonic()
        self.pending[body.request_id]=dict(peer=peer["id"],session=peer["session"],body=body,
            kind=result_kind,callback=callback,packet=packet,sent=now,started=now,
            attempts=1,accepted=False,timeout=timeout)
        self.log("request",device=peer["id"],kind=body.DESCRIPTOR.name,request=body.request_id,
                 sequence=getattr(body,"command_sequence",0))

    def cancel(self,peer_id,kinds=None):
        for key,value in list(self.pending.items()):
            if value["peer"]==peer_id and (kinds is None or value["kind"] in kinds):
                del self.pending[key]

    def has_pending(self,peer_id,kind=None):
        return any(v["peer"]==peer_id and (kind is None or v["kind"]==kind) for v in self.pending.values())

    def handle_result(self,peer,kind,body):
        out=body.outcome
        pending=self.pending.get(out.request_id)
        if not pending or (pending["peer"],pending["session"],pending["kind"])!=(peer["id"],peer["session"],kind):
            return
        request=pending["body"]
        if kind=="control_result" and (body.active_control_session_id,body.control_epoch)!=(request.control_session_id,request.control_epoch):
            return
        if kind=="obs_motion_result" and (body.obs_id,body.obs_control_session_id,body.control_epoch,body.command_sequence)!=(peer["id"],request.obs_control_session_id,request.control_epoch,request.command_sequence):
            return
        if kind=="pointing_result" and (body.control_session_id,body.control_epoch,body.command_sequence)!=(request.control_session_id,request.control_epoch,request.command_sequence):
            return
        if out.HasField("detail") and len(out.detail.encode("utf-8"))>256:
            return
        stamp(out.reported_at)
        if out.result==pb.REQUEST_RESULT_ACCEPTED:
            pending["accepted"]=True
            return
        if out.result not in (pb.REQUEST_RESULT_COMPLETED,pb.REQUEST_RESULT_REJECTED,pb.REQUEST_RESULT_FAILED,pb.REQUEST_RESULT_CANCELLED) or not out.HasField("completed_at"):
            return
        stamp(out.completed_at)
        if out.result!=pb.REQUEST_RESULT_COMPLETED and not out.HasField("reason_code"):
            return
        del self.pending[out.request_id]
        self.log("result",device=peer["id"],kind=kind,result=pb.RequestResult.Name(out.result))
        pending["callback"](body)

    def tick(self,peers):
        now=self.clock.monotonic()
        for key,value in list(self.pending.items()):
            if self.pending.get(key) is not value:
                continue
            peer=peers.get(value["peer"])
            expired=(not peer or peer["session"]!=value["session"] or now-value["started"]>=value["timeout"] or
                     (not value["accepted"] and value["attempts"]>=3 and now-value["sent"]>=.2))
            if expired:
                del self.pending[key]
                value["callback"](None)
            elif not value["accepted"] and now-value["sent"]>=.2:
                self.transport.sendto(value["packet"],peer["address"])
                value["attempts"]+=1
                value["sent"]=now
