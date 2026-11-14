"""Pure-lateral Pacejka (Magic Formula) lateral force — no combined slip."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PacejkaLateralParams:
    B: float = 10.0
    C: float = 1.3
    D0: float = 1.0
    E: float = 0.97


DEFAULT_PACEJKA_LATERAL = PacejkaLateralParams()


def lateral_force_n(
    *,
    alpha_rad: float,
    fz_n: float,
    mu: float,
    params: PacejkaLateralParams = DEFAULT_PACEJKA_LATERAL,
) -> float:
    """Return Fy [N]. Positive alpha → restoring (negative) Fy."""
    fz = max(0.0, float(fz_n))
    mu_c = float(mu)
    a = float(alpha_rad)
    d = mu_c * params.D0 * fz
    if d == 0.0:
        return 0.0
    ba = params.B * a
    return float(
        -d
        * math.sin(params.C * math.atan(ba - params.E * (ba - math.atan(ba))))
    )
