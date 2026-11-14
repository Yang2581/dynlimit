"""Lattice PID: lateral lattice / AES + Pure Pursuit tracking."""

from __future__ import annotations

from typing import Any

__all__ = ["LatticePidController", "Strategy01Controller"]


def __getattr__(name: str) -> Any:
    if name in ("LatticePidController", "Strategy01Controller"):
        from backend.app.planning.strategies.strategy01.controller import (
            LatticePidController,
            Strategy01Controller,
        )

        return LatticePidController if name == "LatticePidController" else Strategy01Controller
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
