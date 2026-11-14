"""Integer-step multi-rate scheduler."""

from __future__ import annotations


def _require_integer_ratio(name: str, outer_s: float, inner_s: float) -> int:
    if outer_s <= 0 or inner_s <= 0:
        raise ValueError(f"{name}: periods must be positive")
    ratio = outer_s / inner_s
    n = int(round(ratio))
    if n < 1 or abs(ratio - n) > 1e-9:
        raise ValueError(
            f"{name}: {outer_s}/{inner_s} must be an integer ratio (got {ratio})"
        )
    return n


class MultiRateScheduler:
    def __init__(
        self,
        dt_physics_s: float,
        dt_control_s: float,
        dt_publish_s: float,
    ) -> None:
        self.control_every_n = _require_integer_ratio(
            "dt_control_s", dt_control_s, dt_physics_s
        )
        self.publish_every_n = _require_integer_ratio(
            "dt_publish_s", dt_publish_s, dt_physics_s
        )

    def controller_due(self, step_index: int) -> bool:
        return step_index % self.control_every_n == 0

    def snapshot_due(self, step_index: int) -> bool:
        return step_index % self.publish_every_n == 0
