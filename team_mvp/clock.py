"""Time port: protocol UTC uses wall time; deadlines use monotonic time."""
from abc import ABC, abstractmethod
import datetime as dt
import time


class Clock(ABC):
    @abstractmethod
    def time(self) -> float: ...
    @abstractmethod
    def monotonic(self) -> float: ...
    def utc(self, after: float = 0) -> str:
        return dt.datetime.fromtimestamp(self.time()+after,dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00","Z")


class SystemClock(Clock):
    def time(self): return time.time()
    def monotonic(self): return time.monotonic()
