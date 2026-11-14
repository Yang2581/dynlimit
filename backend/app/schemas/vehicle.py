"""Platform vehicle state (SI units)."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field


class VehicleState(BaseModel):
    schema_version: str = "1.0"
    sim_time_ns: int
    vehicle_id: str
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    roll_rad: float
    pitch_rad: float
    vx_mps: float
    vy_mps: float
    yaw_rate_radps: float
    ax_mps2: float | None = None
    ay_mps2: float | None = None
    steering_wheel_rad: float | None = None
    wheel_states: dict[str, Any] | None = None
