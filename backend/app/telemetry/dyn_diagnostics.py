"""Dynamics-aware diagnostic helpers for offline replay / series."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

_WHEEL_IDS = ("L1", "R1", "L2", "R2")


def beta_rad(vx_mps: float, vy_mps: float) -> float:
    """Sideslip β = atan2(v_y, v_x)."""
    return math.atan2(float(vy_mps), float(vx_mps))


def flatten_tire_forces(wheel_states: dict[str, Any] | None) -> dict[str, float]:
    """Flatten CarSim flat keys or MOCK nested axle forces into Fx_*/Fy_*/Fz_*."""
    if not wheel_states:
        return {}
    out: dict[str, float] = {}
    for wid in _WHEEL_IDS:
        for prefix in ("Fx", "Fy", "Fz"):
            key = f"{prefix}_{wid}"
            if key in wheel_states and wheel_states[key] is not None:
                try:
                    out[key] = float(wheel_states[key])
                except (TypeError, ValueError):
                    pass
    if out:
        return out
    front = wheel_states.get("front") or wheel_states.get("F")
    rear = wheel_states.get("rear") or wheel_states.get("R")
    if isinstance(front, dict) and isinstance(rear, dict):
        for side, src in (("L1", front), ("R1", front), ("L2", rear), ("R2", rear)):
            if "Fx_N" in src:
                out[f"Fx_{side}"] = float(src["Fx_N"]) / 2.0
            if "Fy_N" in src:
                out[f"Fy_{side}"] = float(src["Fy_N"]) / 2.0
            if "Fz_N" in src:
                out[f"Fz_{side}"] = float(src["Fz_N"]) / 2.0
    return out


def u_tire_frame(
    wheel_states: dict[str, Any] | None,
    *,
    mu: float = 1.0,
    eps: float = 1e-6,
) -> float | None:
    """Per-frame tire utilization: max_i sqrt(Fx^2+Fy^2) / max(eps, μ Fz).

    Returns ``None`` when no usable tire forces are present.
    """
    forces = flatten_tire_forces(wheel_states if isinstance(wheel_states, dict) else None)
    if not forces:
        return None
    mu_f = max(float(mu), eps)
    vals: list[float] = []
    for wid in _WHEEL_IDS:
        fx = forces.get(f"Fx_{wid}")
        fy = forces.get(f"Fy_{wid}")
        fz = forces.get(f"Fz_{wid}")
        if fx is None or fy is None or fz is None:
            continue
        denom = max(eps, mu_f * max(float(fz), 0.0))
        vals.append(math.sqrt(float(fx) * float(fx) + float(fy) * float(fy)) / denom)
    if not vals:
        return None
    return float(max(vals))


def resolve_friction_mu(
    experiment_dir: str | Path | None = None,
    meta: dict[str, Any] | None = None,
) -> float:
    """Best-effort μ from experiment metadata / track file; default 1.0."""
    data = meta
    if data is None and experiment_dir is not None:
        meta_path = Path(experiment_dir) / "metadata.json"
        if meta_path.is_file():
            try:
                raw = json.loads(meta_path.read_text(encoding="utf-8"))
                data = raw if isinstance(raw, dict) else None
            except json.JSONDecodeError:
                data = None
    if not isinstance(data, dict):
        return 1.0

    for key in ("friction_mu", "mu"):
        if key in data and data[key] is not None:
            try:
                return float(data[key])
            except (TypeError, ValueError):
                pass

    track_path = data.get("track_path")
    if isinstance(track_path, str) and Path(track_path).is_file():
        try:
            track = json.loads(Path(track_path).read_text(encoding="utf-8"))
            if isinstance(track, dict) and track.get("friction_mu") is not None:
                return float(track["friction_mu"])
        except (json.JSONDecodeError, TypeError, ValueError, OSError):
            pass

    track_id = data.get("track_id")
    if isinstance(track_id, str) and track_id:
        try:
            from backend.app.scenario import config_store

            track = config_store.load_track(track_id)
            if track.friction_mu is not None:
                return float(track.friction_mu)
        except Exception:
            pass
    return 1.0


def slim_ctrl_diag(diag: dict[str, Any] | None) -> dict[str, Any]:
    """Keep lightweight diagnostic fields for WS snapshots."""
    if not isinstance(diag, dict):
        return {}
    keep = (
        "controller",
        "ok",
        "solve_ms",
        "best_cost",
        "deadline_miss",
        "deadline_budget_ms",
        "deadline_miss_count",
        "rollout_backend",
        "longitudinal_mode",
        "plant_speed_mps",
        "clamped_steering",
        "clamped_throttle",
        "clamped_brake",
    )
    out: dict[str, Any] = {}
    for k in keep:
        if k in diag:
            out[k] = diag[k]
    return out


def _finite_vals(xs: list[Any]) -> list[float]:
    out: list[float] = []
    for x in xs:
        if x is None:
            continue
        try:
            v = float(x)
        except (TypeError, ValueError):
            continue
        if math.isfinite(v):
            out.append(v)
    return out


def _percentile(sorted_vals: list[float], p: float) -> float:
    """Linear-interpolated percentile; ``sorted_vals`` must be non-empty sorted."""
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    p = max(0.0, min(100.0, float(p)))
    k = (len(sorted_vals) - 1) * (p / 100.0)
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    if lo == hi:
        return sorted_vals[lo]
    w = k - lo
    return sorted_vals[lo] * (1.0 - w) + sorted_vals[hi] * w


def grade_layered_flags(
    metrics: dict[str, Any] | None,
    *,
    deadline_ok: bool | None = None,
) -> dict[str, bool]:
    """Paper-style layered success annotations (not a bake-off scoreboard)."""
    m = metrics if isinstance(metrics, dict) else {}
    no_collision = not bool(m.get("collision", False))
    no_off_track = not bool(m.get("off_track", False))
    plant_fault = bool(m.get("plant_fault") or m.get("plant_error"))
    reason = str(m.get("finished_reason") or "")
    # Conservative: only explicit goal-like reasons count as goal_reached.
    goal_reached = reason in {"goal", "success", "completed", "finished"}
    if deadline_ok is None:
        miss = m.get("deadline_miss_count")
        deadline_ok = True if miss is None else int(miss) == 0
    task = goal_reached and no_collision
    physics = task and no_off_track and (not plant_fault)
    deadline = physics and bool(deadline_ok)
    return {
        "task_only_success": task,
        "physics_safe_success": physics,
        "deadline_feasible_success": deadline,
        "goal_reached": goal_reached,
        "no_collision": no_collision,
        "no_off_track": no_off_track,
        "deadline_ok": bool(deadline_ok),
    }


def summarize_series(
    series: dict[str, list[Any]],
    *,
    run_metrics: dict[str, Any] | None = None,
    u_thresh: float = 0.9,
) -> dict[str, Any]:
    """Aggregate dynamics-aware diagnostics for Replay HUD / paper-facing logs."""
    out: dict[str, Any] = {"u_tire_thresh": float(u_thresh)}

    betas = _finite_vals(list(series.get("beta_rad") or []))
    if betas:
        abs_b = sorted(abs(b) for b in betas)
        out["max_abs_beta"] = abs_b[-1]
        out["p95_abs_beta"] = _percentile(abs_b, 95.0)
        out["max_abs_beta_deg"] = abs_b[-1] * (180.0 / math.pi)
        out["p95_abs_beta_deg"] = out["p95_abs_beta"] * (180.0 / math.pi)

    us = _finite_vals(list(series.get("u_tire") or []))
    if us:
        us_sorted = sorted(us)
        out["U_tire_max"] = us_sorted[-1]
        out["U_tire_p95"] = _percentile(us_sorted, 95.0)
        out["U_tire_frac_above_thresh"] = sum(1 for u in us if u > u_thresh) / len(us)

    solves = _finite_vals(list(series.get("solve_ms") or []))
    if solves:
        solves_sorted = sorted(solves)
        out["solve_ms_max"] = solves_sorted[-1]
        out["solve_ms_p95"] = _percentile(solves_sorted, 95.0)
        out["solve_ms_median"] = _percentile(solves_sorted, 50.0)

    misses = _finite_vals(list(series.get("deadline_miss") or []))
    if misses:
        out["deadline_miss_rate"] = sum(1 for m in misses if m >= 0.5) / len(misses)

    # Prefer series-derived miss count; fall back to metrics.json.
    deadline_ok: bool | None = None
    if "deadline_miss_rate" in out:
        deadline_ok = out["deadline_miss_rate"] <= 0.0
    elif isinstance(run_metrics, dict) and run_metrics.get("deadline_miss_count") is not None:
        deadline_ok = int(run_metrics["deadline_miss_count"]) == 0

    flags = grade_layered_flags(run_metrics, deadline_ok=deadline_ok)
    out["flags"] = flags
    return out
