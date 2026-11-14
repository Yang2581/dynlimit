"""Simulation clock (nanosecond authority)."""

from __future__ import annotations


class SimulationClock:
    def __init__(self, dt_physics_s: float = 0.001) -> None:
        if dt_physics_s <= 0:
            raise ValueError("dt_physics_s must be positive")
        self._dt_physics_s = float(dt_physics_s)
        self._dt_physics_ns = int(round(dt_physics_s * 1e9))
        self.simulation_time_ns: int = 0
        self.step_index: int = 0

    @property
    def dt_physics_s(self) -> float:
        return self._dt_physics_s

    @property
    def dt_physics_ns(self) -> int:
        return self._dt_physics_ns

    def tick(self) -> None:
        self.simulation_time_ns += self._dt_physics_ns
        self.step_index += 1

    def reset(self) -> None:
        self.simulation_time_ns = 0
        self.step_index = 0
