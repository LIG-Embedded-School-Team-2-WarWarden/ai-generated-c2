"""Replaceable pointing strategy; coordinates remain an installation concern."""

from typing import Protocol
from .geometry import angles


class PointingSolver(Protocol):
    def solve(self, obj: dict, observer: dict, effector: dict) -> tuple[float, float]: ...


class RelativePointingSolver:
    def solve(self, obj, observer, effector):
        return angles(obj, observer, effector)
