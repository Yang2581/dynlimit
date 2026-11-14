"""CarSim import/export ↔ platform SI conversions.

Defaults follow docs/phase0/field_unit_map.md (script-derived; calibrate before papers).
"""

from __future__ import annotations

import math
from typing import Any

from backend.app.schemas.control import ControlCommand
from backend.app.schemas.vehicle import VehicleState

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
KMH_TO_MPS = 1.0 / 3.6
MPS_TO_KMH = 3.6

DEFAULT_STEER_IMPORTS = ("IMP_STEER_L1", "IMP_STEER_R1")

# Drive / brake imports that enable platform longitudinal control on CarSim.
LONGITUDINAL_DRIVE_LABELS = frozenset(
    {
        "IMP_THROTTLE_ENGINE",
        "IMP_M_MOTOR",
        "IMP_THROTTLE",
    }
)
LONGITUDINAL_BRAKE_LABELS = frozenset(
    {
        "IMP_FBK_PDL",
        "IMP_BRAKE",
        "IMP_PCON_BK",  # master-cylinder pressure [MPa] when OPT_BK_PEDAL=0
    }
)
LONGITUDINAL_IMPORT_LABELS = LONGITUDINAL_DRIVE_LABELS | LONGITUDINAL_BRAKE_LABELS


def has_longitudinal_imports(import_labels: list[str]) -> bool:
    """True if any drive (or brake-only) channel is present for longitudinal actuation."""
    labels = set(import_labels)
    return bool(labels & LONGITUDINAL_DRIVE_LABELS) or bool(labels & LONGITUDINAL_BRAKE_LABELS)


def control_to_imports(
    cmd: ControlCommand,
    import_labels: list[str],
    *,
    steer_in_degrees: bool = True,
    motor_torque_nm: float = 200.0,
    brake_p_max_mpa: float = 10.0,
) -> dict[str, float]:
    """Map platform ControlCommand to CarSim import dict (all labels required)."""
    steer_value = cmd.steering_cmd * RAD2DEG if steer_in_degrees else cmd.steering_cmd
    out: dict[str, float] = {}
    for label in import_labels:
        if label in ("IMP_STEER_L1", "IMP_STEER_R1", "IMP_STEER_L2", "IMP_STEER_R2", "IMP_STEER_SW"):
            out[label] = float(steer_value)
        elif label in ("IMP_THROTTLE_ENGINE", "IMP_THROTTLE"):
            # CarSim throttle often 0-100 style; keep 0-1→0-100 until confirmed
            out[label] = float(cmd.throttle_cmd * 100.0)
        elif label in ("IMP_FBK_PDL", "IMP_BRAKE"):
            out[label] = float(cmd.brake_cmd)
        elif label == "IMP_PCON_BK":
            # OPT_BK_PEDAL=0: brake_cmd 0..1 → master-cylinder pressure [MPa]
            out[label] = float(cmd.brake_cmd * brake_p_max_mpa)
        elif label in ("IMP_M_MOTOR",):
            # Map throttle 0..1 → motor torque Nm (scale configurable)
            out[label] = float(cmd.throttle_cmd * motor_torque_nm)
        else:
            out[label] = 0.0
    return out


def exports_to_state(
    exports: dict[str, float],
    *,
    sim_time_ns: int,
    vehicle_id: str = "ego",
    speed_in_kmh: bool = True,
    yaw_in_degrees: bool = True,
    yaw_rate_in_degrees: bool = True,
) -> VehicleState:
    """Map CarSim export dict to platform VehicleState. Missing → None (never fake 0)."""

    def get(name: str) -> float | None:
        if name not in exports:
            return None
        return float(exports[name])

    x = get("Xo")
    y = get("Yo")
    if x is None:
        x = get("XCG_TM")
    if y is None:
        y = get("YCG_TM")

    yaw_raw = get("Yaw")
    if yaw_raw is None:
        yaw_rad = None
    else:
        yaw_rad = yaw_raw * DEG2RAD if yaw_in_degrees else yaw_raw

    vx_raw = get("Vx")
    vy_raw = get("Vy")
    if vx_raw is None:
        vx_mps = None
    else:
        vx_mps = vx_raw * KMH_TO_MPS if speed_in_kmh else vx_raw
    if vy_raw is None:
        vy_mps = None
    else:
        vy_mps = vy_raw * KMH_TO_MPS if speed_in_kmh else vy_raw

    avz_raw = get("AVz")
    if avz_raw is None:
        yaw_rate = None
    else:
        yaw_rate = avz_raw * DEG2RAD if yaw_rate_in_degrees else avz_raw

    # CarSim Ax/Ay exports in this dataset are in g (see LastRun_echo EXPORT AX/AY).
    ax_raw = get("Ax")
    ay_raw = get("Ay")
    ax = None if ax_raw is None else ax_raw * 9.80665
    ay = None if ay_raw is None else ay_raw * 9.80665

    roll_raw = get("Roll")
    if roll_raw is None:
        roll_rad = 0.0
    else:
        roll_rad = roll_raw * DEG2RAD if yaw_in_degrees else roll_raw

    pitch_raw = get("Pitch")
    if pitch_raw is None:
        pitch_rad = 0.0
    else:
        pitch_rad = pitch_raw * DEG2RAD if yaw_in_degrees else pitch_raw

    steer_sw = get("Steer_SW")
    if steer_sw is None:
        steering_wheel_rad = None
    else:
        steering_wheel_rad = steer_sw * DEG2RAD if yaw_in_degrees else steer_sw

    # Required platform fields: if CarSim did not provide, raise — caller must ensure exports
    missing = [
        name
        for name, val in (
            ("x_m", x),
            ("y_m", y),
            ("yaw_rad", yaw_rad),
            ("vx_mps", vx_mps),
            ("vy_mps", vy_mps),
            ("yaw_rate_radps", yaw_rate),
        )
        if val is None
    ]
    if missing:
        raise KeyError(
            "CarSim exports missing required fields for VehicleState: "
            + ", ".join(missing)
            + f" (got keys={sorted(exports)})"
        )

    return VehicleState(
        sim_time_ns=sim_time_ns,
        vehicle_id=vehicle_id,
        x_m=float(x),
        y_m=float(y),
        z_m=0.0,
        yaw_rad=float(yaw_rad),
        roll_rad=float(roll_rad),
        pitch_rad=float(pitch_rad),
        vx_mps=float(vx_mps),
        vy_mps=float(vy_mps),
        yaw_rate_radps=float(yaw_rate),
        ax_mps2=ax,
        ay_mps2=ay,
        steering_wheel_rad=steering_wheel_rad,
        wheel_states=_wheel_states_from_exports(exports),
    )


def _wheel_states_from_exports(exports: dict[str, float]) -> dict[str, Any] | None:
    tire_keys = [k for k in exports if k.startswith(("Fy_", "Fx_", "Fz_", "Alpha_", "Kappa_"))]
    if not tire_keys:
        return None
    return {k: float(exports[k]) for k in tire_keys}


def conversion_notes() -> dict[str, Any]:
    return {
        "steer": "platform rad → CarSim deg for IMP_STEER_*",
        "speed": "CarSim Vx/Vy treated as km/h → m/s",
        "yaw": "CarSim Yaw treated as deg → rad (configurable)",
        "yaw_rate": "CarSim AVz treated as deg/s → rad/s",
        "reference": "docs/phase0/field_unit_map.md",
    }
