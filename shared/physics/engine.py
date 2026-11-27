"""Backend-independent attitude integration interface."""

from __future__ import annotations

from abc import ABC, abstractmethod


class AttitudeEngine(ABC):
    """Moment-driven attitude integrator.

    ``step`` takes moments in (pitch, roll, yaw) order in N·m and returns
    (pitch, roll, yaw, pitch_rate, roll_rate, yaw_rate) in rad and rad/s.
    ``reset`` restores the initial attitude and rates.
    """

    @abstractmethod
    def reset(self) -> None:
        ...

    @abstractmethod
    def step(self, moments: tuple[float, float, float]) -> tuple[float, float, float, float, float, float]:
        ...
