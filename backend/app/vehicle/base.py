"""Vehicle dynamics adapter Protocol."""

from __future__ import annotations

from typing import Any, Protocol

from backend.app.schemas.capabilities import VehicleCapabilities
from backend.app.schemas.control import ControlCommand
from backend.app.schemas.vehicle import VehicleState


class VehicleDynamicsAdapter(Protocol):
    def initialize(self, vehicle_config: dict[str, Any], road_config: dict[str, Any]) -> None: ...

    def set_control(self, cmd: ControlCommand) -> None: ...

    def step(self, dt_s: float) -> None: ...

    def get_state(self) -> VehicleState: ...

    def read_cached_state(self) -> VehicleState: ...

    def reset(self, initial_condition: dict[str, Any]) -> None: ...

    def close(self) -> None: ...

    def capabilities(self) -> VehicleCapabilities: ...
