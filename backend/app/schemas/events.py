"""Simulation events."""

from typing import Any, Literal

from pydantic import BaseModel


EventType = Literal[
    "timeout",
    "error",
    "finished",
    "custom",
    "collision",
    "off_track",
]


class SimulationEvent(BaseModel):
    schema_version: str = "1.0"
    sim_time_ns: int
    type: EventType
    message: str
    payload: dict[str, Any] | None = None
