"""Adapter capability flags for optional vehicle channels."""

from pydantic import BaseModel, Field


class VehicleCapabilities(BaseModel):
    schema_version: str = "1.0"
    has_accel: bool = False
    has_steering_wheel: bool = False
    has_wheel_states: bool = False
    has_tire_forces: bool = False
    notes: str = Field(default="", description="Human-readable caveats")
