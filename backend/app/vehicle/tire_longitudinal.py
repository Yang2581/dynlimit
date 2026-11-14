"""Independent longitudinal slip force — no combined-slip coupling with Fy."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LongitudinalSlipParams:
    B: float = 12.0
    C: float = 1.65
    E: float = 0.97
    kappa_max_drive: float = 0.15
    kappa_max_brake: float = 0.2


DEFAULT_LONGITUDINAL = LongitudinalSlipParams()


def kappa_cmd_from_pedals(
    *,
    throttle: float,
    brake: float,
    params: LongitudinalSlipParams = DEFAULT_LONGITUDINAL,
) -> float:
    """Map pedals to equivalent commanded slip. Brake overrides throttle."""
    th = max(0.0, min(1.0, float(throttle)))
    br = max(0.0, min(1.0, float(brake)))
    if br > 0.0:
        return -params.kappa_max_brake * br
    return params.kappa_max_drive * th


def longitudinal_force_n(
    *,
    kappa: float,
    fz_n: float,
    mu: float,
    params: LongitudinalSlipParams = DEFAULT_LONGITUDINAL,
) -> float:
    """Return Fx [N]. Positive kappa → positive (drive) Fx."""
    fz = max(0.0, float(fz_n))
    mu_c = float(mu)
    k = float(kappa)
    d = mu_c * fz
    if d == 0.0:
        return 0.0
    bk = params.B * k
    return float(
        d * math.sin(params.C * math.atan(bk - params.E * (bk - math.atan(bk))))
    )
