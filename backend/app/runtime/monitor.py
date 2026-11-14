"""Event monitor Protocol and composite timeout / geometry monitor."""

from __future__ import annotations

from typing import Protocol

from backend.app.schemas.events import SimulationEvent
from backend.app.schemas.vehicle import VehicleState
from backend.app.scenario.base import Scenario


class EventMonitor(Protocol):
    def evaluate(
        self, state: VehicleState, scenario: Scenario
    ) -> list[SimulationEvent]: ...


class NullEventMonitor:
    def evaluate(
        self, state: VehicleState, scenario: Scenario
    ) -> list[SimulationEvent]:
        return []


class TimeoutMonitor:
    """Emit a single timeout event when sim time exceeds scenario max."""

    def __init__(self) -> None:
        self._fired = False

    def reset(self) -> None:
        self._fired = False

    def evaluate(
        self, state: VehicleState, scenario: Scenario
    ) -> list[SimulationEvent]:
        max_s = scenario.max_sim_time_s()
        if self._fired or max_s is None:
            return []
        if state.sim_time_ns * 1e-9 >= max_s:
            self._fired = True
            return [
                SimulationEvent(
                    sim_time_ns=state.sim_time_ns,
                    type="timeout",
                    message=f"Reached max_sim_time_s={max_s}",
                )
            ]
        return []


class GeometryMonitor:
    """Collision / off-track from scenario.check_events; optional stop flags."""

    def __init__(self) -> None:
        self._seen: set[str] = set()

    def reset(self) -> None:
        self._seen.clear()

    def evaluate(
        self, state: VehicleState, scenario: Scenario
    ) -> list[SimulationEvent]:
        raw = scenario.check_events(state)
        out: list[SimulationEvent] = []
        for item in raw:
            et = str(item.get("type", "custom"))
            key = et + ":" + str((item.get("payload") or {}).get("other_id", ""))
            if key in self._seen and et in ("collision", "off_track"):
                # still allow stop decision via attributes below
                pass
            else:
                self._seen.add(key)
                out.append(
                    SimulationEvent(
                        sim_time_ns=state.sim_time_ns,
                        type=et,  # type: ignore[arg-type]
                        message=str(item.get("message", et)),
                        payload=item.get("payload"),
                    )
                )
        return out


class CompositeMonitor:
    def __init__(self, *monitors: EventMonitor) -> None:
        self._monitors = list(monitors)

    def reset(self) -> None:
        for m in self._monitors:
            if hasattr(m, "reset"):
                m.reset()  # type: ignore[attr-defined]

    def evaluate(
        self, state: VehicleState, scenario: Scenario
    ) -> list[SimulationEvent]:
        events: list[SimulationEvent] = []
        for m in self._monitors:
            events.extend(m.evaluate(state, scenario))
        return events
