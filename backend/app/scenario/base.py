"""Scenario Protocol."""

from __future__ import annotations

from typing import Any, Protocol

from backend.app.schemas.observation import Observation
from backend.app.schemas.vehicle import VehicleState


class Scenario(Protocol):
    def initialize(self, config: dict[str, Any]) -> None: ...

    def observation(self, t_ns: int) -> Observation: ...

    def advance_npcs(self, dt_s: float, ego: VehicleState | None = None) -> None: ...

    def check_events(self, ego: VehicleState) -> list[dict[str, Any]]: ...

    def max_sim_time_s(self) -> float | None: ...

    def track_id(self) -> str: ...

    def ego_initial_condition(self) -> dict[str, Any]: ...
