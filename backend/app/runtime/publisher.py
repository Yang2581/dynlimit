"""Snapshot publisher Protocol, null, and latest-only fan-out."""

from __future__ import annotations

import asyncio
import threading
from typing import Any, Protocol

from backend.app.schemas.snapshot import SimulationSnapshot


class SnapshotPublisher(Protocol):
    def offer_latest(self, snapshot: SimulationSnapshot) -> None: ...


class NullPublisher:
    def offer_latest(self, snapshot: SimulationSnapshot) -> None:
        return None


class LatestOnlyPublisher:
    """Keep only the newest snapshot; slow WS clients never block the plant."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._seq = 0
        self._latest: dict[str, Any] | None = None
        self._queues: list[asyncio.Queue[dict[str, Any]]] = []
        self._loop: asyncio.AbstractEventLoop | None = None
        self.session_id: str = ""

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    def subscribe(self) -> asyncio.Queue[dict[str, Any]]:
        q: asyncio.Queue[dict[str, Any]] = asyncio.Queue(maxsize=1)
        with self._lock:
            self._queues.append(q)
            latest = self._latest
        if latest is not None:
            q.put_nowait(dict(latest))
        return q

    def unsubscribe(self, q: asyncio.Queue[dict[str, Any]]) -> None:
        with self._lock:
            if q in self._queues:
                self._queues.remove(q)

    def latest(self) -> dict[str, Any] | None:
        with self._lock:
            return dict(self._latest) if self._latest is not None else None

    def offer_latest(self, snapshot: SimulationSnapshot) -> None:
        with self._lock:
            self._seq += 1
            payload = {
                "schema_version": "1.0",
                "type": "simulation.snapshot",
                "session_id": self.session_id,
                "seq": self._seq,
                "sim_time_ns": snapshot.sim_time_ns,
                "step_index": snapshot.step_index,
                "vehicles": [
                    {
                        "id": v.vehicle_id,
                        "x_m": v.x_m,
                        "y_m": v.y_m,
                        "z_m": v.z_m,
                        "yaw_rad": v.yaw_rad,
                        "vx_mps": v.vx_mps,
                        "vy_mps": v.vy_mps,
                        "yaw_rate_radps": v.yaw_rate_radps,
                    }
                    for v in snapshot.vehicles
                ],
                "metrics": dict(snapshot.metrics),
                "trail_xy": snapshot.metrics.get("trail_xy", []),
                "controller": snapshot.metrics.get("controller"),
                "npcs": snapshot.metrics.get("npcs", []),
                "obstacles": snapshot.metrics.get("obstacles", []),
            }
            self._latest = payload
            queues = list(self._queues)
            loop = self._loop

        if loop is None:
            return

        def _push(queue: asyncio.Queue[dict[str, Any]], message: dict[str, Any]) -> None:
            while not queue.empty():
                try:
                    queue.get_nowait()
                except asyncio.QueueEmpty:
                    break
            try:
                queue.put_nowait(message)
            except asyncio.QueueFull:
                pass

        for q in queues:
            loop.call_soon_threadsafe(_push, q, payload)
