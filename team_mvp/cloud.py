"""ICD POINT_CLOUD_CHUNK assembly and C2-owned LiDAR detection."""
import json
import queue
import struct
import subprocess
import threading
import time
from .contract import pb, finite, stamp, uid

class Detector:
    def __init__(self, executable):
        self.process = subprocess.Popen([str(executable)], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))

    def detect(self, width, height, points):
        data = bytearray(struct.pack("<II", width, height))
        for p in points:
            data.extend(struct.pack("<IfffI", p.sdk_depth_raw, p.x if p.xyz_valid else 0,
                p.y if p.xyz_valid else 0, p.z if p.xyz_valid else 0, p.xyz_valid))
        self.process.stdin.write(data)
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        result = json.loads(line)
        if result is None:
            raise ValueError("detector failed")
        for value in result:
            value["id"] = uid()
        return result

    def close(self):
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait(timeout=3)
        for stream in (self.process.stdin,self.process.stdout):
            try:
                stream.close()
            except OSError:
                pass

class Cloud:
    def __init__(self, detector, publish):
        self.detector, self.publish = detector, publish
        self.jobs = queue.Queue(maxsize=1)
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.key = None
        self.high_frame = 0
        self.generation = 0
        self.frame = None
        self.state = "NO_DATA"
        self.result = None
        self.started = 0
        self.sealed = {}
        self.worker.start()

    def reset(self):
        self.generation += 1
        self.key, self.high_frame, self.frame, self.result = None, 0, None, None
        self.state = "NO_DATA"
        self.sealed = {}

    def fail(self, reason):
        self.generation += 1
        self.frame = self.result = None
        self.sealed = {}
        self.state = reason
        return False

    def ingest(self, session, c, verified):
        if self.key != session:
            self.reset()
            self.key = session
        if not c.frame_id or c.frame_id < self.high_frame:
            return False
        n = c.width * c.height
        if not 0 < n <= 500000 or n != c.total_points or not 1 <= c.chunk_count <= 8192 or c.chunk_index >= c.chunk_count:
            return self.fail("FRAME_INVALID")
        if not c.points or c.motion_compensation != pb.PointCloudChunk.MOTION_COMPENSATION_NONE:
            return self.fail("FRAME_INVALID")
        for p in c.points:
            xyz = [p.HasField(a) for a in ("x", "y", "z")]
            if p.pixel_index >= n or (p.xyz_valid and (not all(xyz) or not finite(p.x,p.y,p.z))) or (not p.xyz_valid and any(xyz)):
                return self.fail("FRAME_INVALID")
            if p.HasField("intensity") and (not finite(p.intensity) or not c.HasField("intensity_scale_id")):
                return self.fail("FRAME_INVALID")
        if c.frame_id > self.high_frame:
            self.sealed = {}
            self.generation += 1
            self.high_frame = c.frame_id
            self.state, self.result = "UPDATING", None
            self.started = time.monotonic()
            meta = pb.PointCloudChunk()
            meta.CopyFrom(c)
            meta.ClearField("points")
            meta.chunk_index = 0
            self.frame = dict(meta=meta, slots=[None]*n, chunks={}, bytes=0, verified=verified)
        encoded = c.SerializeToString(deterministic=True)
        if self.sealed:
            return True if self.sealed.get(c.chunk_index) == encoded else self.fail("FRAME_INVALID")
        if not self.frame or time.monotonic()-self.started >= 1:
            return self.fail("FRAME_INCOMPLETE")
        f = self.frame
        meta = pb.PointCloudChunk()
        meta.CopyFrom(c)
        meta.ClearField("points")
        meta.chunk_index = 0
        if meta != f["meta"] or verified != f["verified"]:
            return self.fail("FRAME_INVALID")
        encoded = c.SerializeToString(deterministic=True)
        if c.chunk_index in f["chunks"]:
            return True if f["chunks"][c.chunk_index] == encoded else self.fail("FRAME_INVALID")
        indices = [p.pixel_index for p in c.points]
        if len(set(indices)) != len(indices) or any(f["slots"][i] is not None for i in indices):
            return self.fail("FRAME_INVALID")
        f["bytes"] += len(encoded)
        if f["bytes"] > 8*1024*1024:
            return self.fail("FRAME_INVALID")
        f["chunks"][c.chunk_index] = encoded
        for p in c.points:
            copied = pb.Point()
            copied.CopyFrom(p)
            f["slots"][p.pixel_index] = copied
        if len(f["chunks"]) == c.chunk_count:
            if any(p is None for p in f["slots"]):
                return self.fail("FRAME_INVALID")
            job = self.generation, session, f["meta"], f["slots"], verified
            self.sealed = f["chunks"]
            self.frame = None
            self.state = "PROCESSING"
            try:
                self.jobs.get_nowait()
            except queue.Empty:
                pass
            self.jobs.put_nowait(job)
        return True

    def run(self):
        while True:
            job = self.jobs.get()
            if job is None:
                return
            generation, session, meta, slots, verified = job
            try:
                objects = self.detector.detect(meta.width, meta.height, slots)
                self.publish(generation, session, meta, objects, verified, None)
            except Exception as error:
                self.publish(generation, session, meta, [], False, str(error))

    def close(self):
        self.detector.close()
        try:
            self.jobs.get_nowait()
        except queue.Empty:
            pass
        self.jobs.put_nowait(None)
        self.worker.join(timeout=3)
