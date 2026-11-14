"""Controller Protocol."""

from __future__ import annotations

from typing import Any, Protocol

from backend.app.schemas.control import ControlCommand
from backend.app.schemas.observation import Observation
from backend.app.schemas.vehicle import VehicleState


class Controller(Protocol):
    def reset(self, config: dict[str, Any], initial_state: VehicleState) -> None: ...

    def compute_control(
        self, state: VehicleState, observation: Observation
    ) -> tuple[ControlCommand, dict[str, Any]]: ...
