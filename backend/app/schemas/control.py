"""Control command and actuation limits."""

from pydantic import BaseModel, Field


class ControlCommand(BaseModel):
    schema_version: str = "1.0"
    steering_cmd: float = Field(description="Road-wheel / equivalent steer [rad]")
    throttle_cmd: float = Field(ge=0.0, le=1.0)
    brake_cmd: float = Field(ge=0.0, le=1.0)
    source: str
    valid_until_ns: int | None = None


class ActuationLimits(BaseModel):
    schema_version: str = "1.0"
    max_steering_rad: float = 0.6
    max_steering_rate_radps: float = 1.0
    max_throttle: float = 1.0
    max_brake: float = 1.0
