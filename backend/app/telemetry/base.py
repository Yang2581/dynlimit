"""Recorder Protocol."""

from __future__ import annotations

from typing import Any, Protocol


class Recorder(Protocol):
    def start(self, metadata: dict[str, Any]) -> None: ...

    def enqueue(self, topic: str, t_ns: int, payload: dict[str, Any]) -> None: ...

    def finalize(self, status: str) -> None: ...
