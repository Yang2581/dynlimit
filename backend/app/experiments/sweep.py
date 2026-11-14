"""Offline MPPI (or other) param-grid sweeps — no Live WS required."""

from __future__ import annotations

import itertools
import json
import time
import uuid
from pathlib import Path
from typing import Any

from backend.app.controllers.registry import create_controller
from backend.app.core.engine import Engine
from backend.app.platform.paths import get_paths
from backend.app.runtime.actuation import ActuationGuard
from backend.app.runtime.monitor import CompositeMonitor, GeometryMonitor, TimeoutMonitor
from backend.app.runtime.publisher import NullPublisher
from backend.app.scenario.simple import SimpleScenario
from backend.app.telemetry.mcap_writer import NullRecorder
from backend.app.vehicle.mock_adapter import MockDynamicsAdapter


def _cartesian(grid: dict[str, list[Any]]) -> list[dict[str, Any]]:
    if not grid:
        return [{}]
    keys = list(grid.keys())
    values = [list(grid[k]) for k in keys]
    return [dict(zip(keys, combo)) for combo in itertools.product(*values)]


def _run_one(
    *,
    run_dir: Path,
    track_path: Path,
    scenario_path: Path,
    controller_name: str,
    params: dict[str, Any],
    duration_s: float,
) -> dict[str, Any]:
    run_dir.mkdir(parents=True, exist_ok=True)
    scenario = SimpleScenario()
    scenario.load_files(track_path, scenario_path)
    cfg = scenario.config()
    # Cap wall-clock sim length for batch (do not mutate on-disk scenario)
    cfg.termination.max_sim_time_s = float(duration_s)

    ctrl_kwargs = dict(params)
    if controller_name == "mppi":
        allowed = {
            "horizon",
            "num_samples",
            "dt_s",
            "wheelbase_m",
            "seed",
            "temperature",
            "steer_std",
            "throttle_std",
            "max_steer_rad",
            "throttle_nominal",
            "w_lat",
            "w_yaw",
            "w_steer",
            "w_throttle",
            "top_k_viz",
            "target_speed_mps",
            "rollout_backend",
        }
        # Map throttle_cmd → throttle_nominal for convenience
        if "throttle_cmd" in ctrl_kwargs and "throttle_nominal" not in ctrl_kwargs:
            ctrl_kwargs["throttle_nominal"] = ctrl_kwargs.pop("throttle_cmd")
        filtered = {k: v for k, v in ctrl_kwargs.items() if k in allowed}
        controller = create_controller("mppi", **filtered)
        controller.set_waypoints(scenario.centerline_m())
    elif controller_name == "pure_pursuit":
        controller = create_controller(
            "pure_pursuit",
            throttle_cmd=float(ctrl_kwargs.get("throttle_cmd", 0.35)),
            lookahead_m=float(ctrl_kwargs.get("lookahead_m", 12.0)),
        )
        controller.set_waypoints(scenario.centerline_m())
    elif controller_name == "strategy01":
        # Match SimSession._build_engine: PP defaults plus optional lattice keys.
        kw: dict[str, Any] = {
            "throttle_cmd": float(ctrl_kwargs.get("throttle_cmd", 0.35)),
            "lookahead_m": float(ctrl_kwargs.get("lookahead_m", 12.0)),
            "replan_every_n": int(ctrl_kwargs.get("replan_every_n", 1)),
        }
        for key in (
            "lateral_offsets_m",
            "d_m",
            "horizon_m",
            "ds_m",
            "ego_radius_m",
            "npc_radius_m",
            "predict_dt_s",
            "w_lat",
            "w_change",
            "v_min_mps",
            "wheelbase_m",
            "max_steer_rad",
            "target_speed_mps",
        ):
            if key in ctrl_kwargs:
                kw[key] = ctrl_kwargs[key]
        controller = create_controller("strategy01", **kw)
        controller.set_waypoints(scenario.centerline_m())
    else:
        controller = create_controller(
            "constant",
            steering_cmd=float(ctrl_kwargs.get("steering_cmd", 0.0)),
            throttle_cmd=float(ctrl_kwargs.get("throttle_cmd", 0.35)),
        )

    vehicle = MockDynamicsAdapter()
    engine = Engine(
        vehicle=vehicle,
        controller=controller,
        scenario=scenario,
        recorder=NullRecorder(),
        actuation_guard=ActuationGuard(),
        snapshot_publisher=NullPublisher(),
        event_monitor=CompositeMonitor(TimeoutMonitor(), GeometryMonitor()),
        # Batch: coarser control/publish ok for speed
        dt_physics_s=0.001,
        dt_control_s=0.02,
        dt_publish_s=0.05,
    )
    meta = {
        "schema_version": "1.0",
        "kind": "sweep_run",
        "controller": controller_name,
        "params": params,
        "duration_s_cap": duration_s,
        "track_path": str(track_path),
        "scenario_path": str(scenario_path),
        "track_id": scenario.track_id(),
        "scenario_id": scenario.scenario_id(),
    }
    t0 = time.perf_counter()
    engine.load(meta)
    engine.start()
    status = engine.run_while_running()
    wall_s = time.perf_counter() - t0
    metrics = dict(engine.experiment_metrics or {})
    metrics["wall_s"] = wall_s
    metrics["engine_status"] = status
    metrics["params"] = params
    metrics["controller"] = controller_name
    if hasattr(controller, "last_diagnostics"):
        diag = getattr(controller, "last_diagnostics") or {}
        metrics["last_solve_ms"] = diag.get("solve_ms")
        metrics["rollout_backend"] = diag.get("rollout_backend")
    try:
        vehicle.close()
    except Exception:
        pass

    (run_dir / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    (run_dir / "metadata.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    return metrics


def run_param_sweep(
    *,
    grid: dict[str, list[Any]],
    base_params: dict[str, Any] | None = None,
    controller: str = "mppi",
    scenario_id: str = "web_demo_001",
    track_id: str | None = None,
    duration_s: float = 25.0,
    sweeps_dir: Path | None = None,
) -> dict[str, Any]:
    """Cartesian product of ``grid`` over ``base_params``; write under data/sweeps/."""
    base = dict(base_params or {})
    combos = _cartesian(grid)
    if len(combos) > 48:
        raise ValueError(f"grid too large ({len(combos)} runs); max 48")

    paths = get_paths()
    scenario_path = paths.scenarios_dir / f"{scenario_id}.json"
    if not scenario_path.is_file():
        raise FileNotFoundError(f"scenario not found: {scenario_id}")
    if track_id:
        track_path = paths.tracks_dir / f"{track_id}.json"
    else:
        # Resolve from scenario
        scen = json.loads(scenario_path.read_text(encoding="utf-8"))
        tid = scen.get("track_id") or "oval_01"
        track_path = paths.tracks_dir / f"{tid}.json"
    if not track_path.is_file():
        raise FileNotFoundError(f"track not found: {track_path.name}")

    root = sweeps_dir or paths.sweeps_dir
    sweep_id = "sweep_" + uuid.uuid4().hex[:10]
    sweep_dir = root / sweep_id
    sweep_dir.mkdir(parents=True, exist_ok=True)

    runs: list[dict[str, Any]] = []
    for i, combo in enumerate(combos):
        params = {**base, **combo}
        run_id = f"run_{i:03d}"
        metrics = _run_one(
            run_dir=sweep_dir / run_id,
            track_path=track_path,
            scenario_path=scenario_path,
            controller_name=controller,
            params=params,
            duration_s=duration_s,
        )
        runs.append(
            {
                "run_id": run_id,
                "params": params,
                "metrics": {
                    k: metrics.get(k)
                    for k in (
                        "duration_s",
                        "path_length_m",
                        "mean_abs_lat_m",
                        "max_vx",
                        "off_track",
                        "collision",
                        "finished_reason",
                        "wall_s",
                        "last_solve_ms",
                        "rollout_backend",
                        "engine_status",
                    )
                    if k in metrics
                },
            }
        )

    # Rank: prefer finished without off_track; then lower mean |lat|
    def _rank_key(r: dict[str, Any]) -> tuple:
        m = r["metrics"]
        bad = 1 if m.get("off_track") or m.get("collision") else 0
        lat = float(m.get("mean_abs_lat_m") or 1e9)
        path = -float(m.get("path_length_m") or 0.0)
        return (bad, lat, path)

    ranked = sorted(runs, key=_rank_key)
    for i, r in enumerate(ranked):
        r["rank"] = i + 1

    summary = {
        "schema_version": "1.0",
        "sweep_id": sweep_id,
        "controller": controller,
        "scenario_id": scenario_id,
        "track_id": track_path.stem,
        "duration_s": duration_s,
        "base_params": base,
        "grid": grid,
        "n_runs": len(runs),
        "runs": ranked,
        "best_run_id": ranked[0]["run_id"] if ranked else None,
    }
    (sweep_dir / "sweep.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def list_sweeps(sweeps_dir: Path | None = None) -> list[dict[str, Any]]:
    root = sweeps_dir or get_paths().sweeps_dir
    if not root.is_dir():
        return []
    out: list[dict[str, Any]] = []
    for d in sorted(root.iterdir(), reverse=True):
        if not d.is_dir():
            continue
        path = d / "sweep.json"
        if not path.is_file():
            continue
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            continue
        out.append(
            {
                "sweep_id": data.get("sweep_id", d.name),
                "controller": data.get("controller"),
                "n_runs": data.get("n_runs"),
                "best_run_id": data.get("best_run_id"),
                "scenario_id": data.get("scenario_id"),
                "created_path": str(d),
            }
        )
    return out


def load_sweep(sweep_id: str, sweeps_dir: Path | None = None) -> dict[str, Any]:
    root = sweeps_dir or get_paths().sweeps_dir
    path = root / sweep_id / "sweep.json"
    if not path.is_file():
        raise FileNotFoundError(sweep_id)
    return json.loads(path.read_text(encoding="utf-8"))
