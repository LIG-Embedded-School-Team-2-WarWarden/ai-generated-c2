"""Detector port and adapter for the unchanged upstream C++ worker process."""

import json
import struct
import subprocess
from typing import Protocol
from .contract import uid


class PointDetector(Protocol):
    def detect(self, width: int, height: int, points) -> list[dict]: ...
    def close(self) -> None: ...


class SubprocessDetector:
    def __init__(self, executable):
        self.process = subprocess.Popen(
            [str(executable)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )

    def detect(self, width, height, points):
        data = bytearray(struct.pack("<II", width, height))
        for p in points:
            data.extend(
                struct.pack(
                    "<IfffI",
                    p.sdk_depth_raw,
                    p.x if p.xyz_valid else 0,
                    p.y if p.xyz_valid else 0,
                    p.z if p.xyz_valid else 0,
                    p.xyz_valid,
                )
            )
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
        for stream in (self.process.stdin, self.process.stdout):
            try:
                stream.close()
            except OSError:
                pass
