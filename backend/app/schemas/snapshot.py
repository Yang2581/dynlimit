"""Snapshot published to UI / observers."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field

from backend.app.schemas.vehicle import VehicleState


class SimulationSnapshot(BaseModel):
    schema_version: str = "1.0"
    sim_time_ns: int
    step_index: int
    vehicles: list[VehicleState]
    metrics: dict[str, Any] = Field(default_factory=dict)
