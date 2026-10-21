"""Shared generated Protobuf only. No local field numbers or private wire codec."""

import datetime as dt
import pathlib
import sys
import uuid
import math
import re

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "external/warwarden-protocol/python"))
from mfs.icd.v1 import mfs_pb2 as pb


def uid():
    return str(uuid.uuid4())


def utc():
    return (
        dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
    )


def stamp(value):
    if not re.fullmatch(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z", value):
        raise ValueError("UTC milliseconds required")
    return dt.datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp()


def envelope(sender, recipient, body, session=None, *, sent_at=None):
    e = pb.Envelope(
        version_major=1,
        version_minor=0,
        sender_id=sender,
        recipient_id=recipient,
        message_id=uid(),
        sent_at=utc() if sent_at is None else sent_at,
    )
    if session:
        e.connection_session_id = session
    getattr(e, body_name(body)).CopyFrom(body)
    return e


def body_name(body):
    for field in pb.Envelope.DESCRIPTOR.oneofs_by_name["body"].fields:
        if field.message_type.full_name == body.DESCRIPTOR.full_name:
            return field.name
    raise ValueError("message not in shared Envelope")


def encode(e):
    b = e.SerializeToString(deterministic=True)
    if len(b) > 1200:
        raise ValueError("Envelope exceeds 1200 bytes")
    return b


def decode(data, recipient):
    if len(data) > 1200:
        raise ValueError("oversized UDP Envelope")
    e = pb.Envelope.FromString(data)
    if e.version_major != 1 or e.version_minor != 0 or not e.WhichOneof("body"):
        raise ValueError("unsupported version/body (including legacy v3/field 100)")
    if not e.sender_id or e.recipient_id != recipient:
        raise ValueError("invalid sender/recipient")
    uuid.UUID(e.message_id)
    stamp(e.sent_at)
    return e


def outcome(request, result=pb.REQUEST_RESULT_COMPLETED, reason=None):
    value = pb.Result(request_id=request, result=result, reported_at=utc())
    if result != pb.REQUEST_RESULT_ACCEPTED:
        value.completed_at = value.reported_at
    if reason is not None:
        value.reason_code = reason
    return value


def finite(*values):
    return all(math.isfinite(x) for x in values)


def current_angles(sample, now, limit=1.0):
    if not sample.sample_sequence or not sample.pan_valid or not sample.tilt_valid:
        return False
    if not all(
        sample.HasField(x) for x in ("pan_deg", "tilt_deg", "pan_measured_at", "tilt_measured_at")
    ):
        return False
    return finite(sample.pan_deg, sample.tilt_deg) and all(
        0 <= now - stamp(t) <= limit for t in (sample.pan_measured_at, sample.tilt_measured_at)
    )
