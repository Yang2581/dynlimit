"""Typed controller observation."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field


class Observation(BaseModel):
    schema_version: str = "1.0"
    sim_time_ns: int
    track_id: str
    ego_ref: dict[str, Any] | None = None
    npcs: list[dict[str, Any]] | None = None
    extras: dict[str, Any] = Field(default_factory=dict)
