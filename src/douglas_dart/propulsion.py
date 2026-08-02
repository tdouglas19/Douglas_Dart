"""Mutually exclusive mode-selector abstraction."""

from __future__ import annotations

from enum import Enum

from .config import ReferenceCase
from .pulsejet import PulsejetSimulator
from .ramjet import RamjetResult, evaluate_ramjet


class PropulsionMode(str, Enum):
    OFF = "off"
    PULSEJET = "pulsejet"
    RAMJET = "ramjet"


class DualModePropulsion:
    """Own the single active intake path and prevent simultaneous operation."""

    def __init__(self, case: ReferenceCase) -> None:
        self.case = case
        self.mode = PropulsionMode.OFF
        self._pulsejet: PulsejetSimulator | None = None

    @property
    def available_intake_area_m2(self) -> float:
        return self.case.selector.available_area_m2

    def select(self, mode: PropulsionMode) -> None:
        self.mode = PropulsionMode(mode)
        if self.mode is not PropulsionMode.PULSEJET:
            self._pulsejet = None

    def start_pulsejet(self, altitude_m: float | None = None, mach: float | None = None) -> None:
        self.select(PropulsionMode.PULSEJET)
        self._pulsejet = PulsejetSimulator(
            self.case.pulsejet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            self.case.altitude_m if altitude_m is None else altitude_m,
            self.case.mach if mach is None else mach,
        )

    def pulsejet_step(self, time_step_s: float):
        if self.mode is not PropulsionMode.PULSEJET or self._pulsejet is None:
            raise RuntimeError("pulsejet intake path is not selected and initialized")
        return self._pulsejet.step(time_step_s)

    def ramjet_point(self, altitude_m: float, mach: float) -> RamjetResult:
        if self.mode is not PropulsionMode.RAMJET:
            raise RuntimeError("ramjet intake path is not selected")
        return evaluate_ramjet(
            self.case.ramjet,
            self.case.selector,
            self.case.nozzle,
            self.case.fuel,
            altitude_m,
            mach,
        )
