"""REST routes for sessions and assets."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Literal

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel, Field

from backend.app.runtime.replay_manager import ReplayManager
from backend.app.runtime.session import SessionManager
from backend.app.scenario import config_store
from backend.app.scenario.track_export import export_track_geometry
from backend.app.scenario.track_generate import apply_generator
from backend.app.scenario.validator import ScenarioValidationError
from backend.app.schemas.scenario import ScenarioConfig
from backend.app.schemas.track import TrackDefinition

ROOT = Path(__file__).resolve().parents[3]
router = APIRouter()


class CreateSessionRequest(BaseModel):
    plant: Literal["mock", "carsim"] = "mock"
    controller: Literal[
        "pure_pursuit",
        "lane_pp",
        "lattice_pid",
        "lattice_pid_ml",
        "stanley",
        "constant",
        "mppi",
        "strategy01",
    ] = (
        "pure_pursuit"
    )
    track_id: str | None = None
    scenario_id: str | None = None
    steering_cmd: float = 0.0
    throttle_cmd: float = 0.35
    realtime: bool = True  # wall-clock 1x for smooth Live viewport
    record: bool = True
    display_name: str | None = None


class CommandResponse(BaseModel):
    accepted: bool
    state: str | None = None
    detail: Any = None
    error: str | None = None


def _manager(request: Request) -> SessionManager:
    return request.app.state.sessions


def _replays(request: Request) -> ReplayManager:
    return request.app.state.replays


@router.get("/health")
def health() -> dict[str, Any]:
    from backend.app.platform.paths import get_paths

    p = get_paths()
    payload: dict[str, Any] = {
        "schema_version": "1.0",
        "status": "ok",
        "service": "dynlimit",
        "paths": {
            "config_root": str(p.config_root),
            "data_root": str(p.data_root),
            "carsim_db": str(p.carsim_db),
            "sources": p.sources,
        },
    }
    try:
        from backend.app.compute.rollout import backend_info

        payload["rollout"] = backend_info()
    except Exception as exc:  # pragma: no cover
        payload["rollout"] = {"error": str(exc)}
    return payload


@router.get("/tracks")
def list_tracks() -> dict[str, Any]:
    return {"tracks": config_store.list_tracks()}


@router.get("/tracks/{track_id}")
def get_track(track_id: str) -> dict[str, Any]:
    try:
        return config_store.load_track(track_id).model_dump()
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.get("/tracks/{track_id}/export")
def export_track(track_id: str, resolution_m: float | None = None) -> dict[str, Any]:
    """Export reference line + left/right boundaries for a saved track."""
    try:
        track = config_store.load_track(track_id)
        return export_track_geometry(track, resolution_m=resolution_m)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.post("/tracks/export")
def export_track_body(body: dict[str, Any]) -> dict[str, Any]:
    """Export from an inline TrackDefinition (supports unsaved editor drafts)."""
    try:
        data = dict(body)
        resolution_m = data.pop("resolution_m", None)
        track = TrackDefinition.model_validate(data)
        return export_track_geometry(
            track,
            resolution_m=float(resolution_m) if resolution_m is not None else None,
        )
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.post("/tracks/generate")
def generate_track_body(body: dict[str, Any]) -> dict[str, Any]:
    """Parametric reshape: stadium/rectangle (closed) or straight (open)."""
    try:
        shape = str(body.get("shape", "stadium"))
        params = dict(body.get("params") or {})
        track_body = dict(body.get("track") or body)
        track_body.pop("shape", None)
        track_body.pop("params", None)
        track_body.pop("track", None)
        track = TrackDefinition.model_validate(track_body)
        updated = apply_generator(track, shape=shape, params=params)  # type: ignore[arg-type]
        return {"ok": True, "track": updated.model_dump()}
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.put("/tracks/{track_id}")
def put_track(track_id: str, body: dict[str, Any]) -> dict[str, Any]:
    try:
        data = dict(body)
        data["track_id"] = track_id
        track = TrackDefinition.model_validate(data)
        path = config_store.save_track(track, overwrite=True)
        return {"ok": True, "path": str(path), "track": track.model_dump()}
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.post("/tracks")
def create_track(body: dict[str, Any]) -> dict[str, Any]:
    try:
        track = TrackDefinition.model_validate(body)
        path = config_store.save_track(track, overwrite=False)
        return {"ok": True, "path": str(path), "track": track.model_dump()}
    except FileExistsError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.delete("/tracks/{track_id}")
def delete_track(track_id: str) -> dict[str, Any]:
    try:
        config_store.delete_track(track_id)
        return {"ok": True}
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


@router.get("/scenarios")
def list_scenarios() -> dict[str, Any]:
    return {"scenarios": config_store.list_scenarios()}


@router.get("/scenarios/{scenario_id}")
def get_scenario(scenario_id: str) -> dict[str, Any]:
    try:
        return config_store.load_scenario(scenario_id).model_dump()
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.put("/scenarios/{scenario_id}")
def put_scenario(scenario_id: str, body: dict[str, Any]) -> dict[str, Any]:
    try:
        data = dict(body)
        data["scenario_id"] = scenario_id
        scenario = ScenarioConfig.model_validate(data)
        path, warnings = config_store.save_scenario(scenario, overwrite=True)
        return {"ok": True, "path": str(path), "warnings": warnings, "scenario": scenario.model_dump()}
    except (ScenarioValidationError, FileNotFoundError, ValueError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.post("/scenarios")
def create_scenario(body: dict[str, Any]) -> dict[str, Any]:
    try:
        scenario = ScenarioConfig.model_validate(body)
        path, warnings = config_store.save_scenario(scenario, overwrite=False)
        return {"ok": True, "path": str(path), "warnings": warnings, "scenario": scenario.model_dump()}
    except FileExistsError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except (ScenarioValidationError, FileNotFoundError, ValueError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@router.delete("/scenarios/{scenario_id}")
def delete_scenario(scenario_id: str) -> dict[str, Any]:
    try:
        config_store.delete_scenario(scenario_id)
        return {"ok": True}
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


@router.post("/sessions")
def create_session(body: CreateSessionRequest, request: Request) -> dict[str, Any]:
    mgr = _manager(request)
    scenario_id = body.scenario_id or "web_demo_001"
    try:
        scenario_cfg = config_store.load_scenario(scenario_id)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    # Scenario owns its track; client track_id must match when provided.
    track_id = scenario_cfg.track_id
    if body.track_id and body.track_id != track_id:
        raise HTTPException(
            status_code=400,
            detail=(
                f"scenario '{scenario_id}' belongs to track '{track_id}', "
                f"not '{body.track_id}'. Select the track first, then a scenario on it."
            ),
        )
    try:
        track = config_store.resolve_track_file(track_id)
        scenario = config_store.resolve_scenario_file(scenario_id)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    from backend.app.experiments.naming import normalize_display_name

    try:
        display_name = normalize_display_name(body.display_name)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    try:
        session = mgr.create(
            plant=body.plant,
            controller=body.controller,
            track_path=track,
            scenario_path=scenario,
            steering_cmd=body.steering_cmd,
            throttle_cmd=body.throttle_cmd,
            realtime=body.realtime,
            record=body.record,
            display_name=display_name,
        )
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    if session.error:
        raise HTTPException(status_code=500, detail=session.error)
    return session.status()


@router.get("/sessions/{session_id}")
def get_session(session_id: str, request: Request) -> dict[str, Any]:
    session = _manager(request).get(session_id)
    if session is None:
        raise HTTPException(status_code=404, detail="session not found")
    return session.status()


def _cmd(session_id: str, request: Request, name: str) -> CommandResponse:
    session = _manager(request).get(session_id)
    if session is None:
        raise HTTPException(status_code=404, detail="session not found")
    result = session.submit(name)  # type: ignore[arg-type]
    if not result.get("accepted"):
        return CommandResponse(
            accepted=False,
            state=result.get("state"),
            error=str(result.get("detail") or result.get("error")),
        )
    return CommandResponse(accepted=True, state=result.get("state"), detail=result.get("detail"))


@router.post("/sessions/{session_id}/start")
def start_session(session_id: str, request: Request) -> CommandResponse:
    return _cmd(session_id, request, "start")


@router.post("/sessions/{session_id}/pause")
def pause_session(session_id: str, request: Request) -> CommandResponse:
    return _cmd(session_id, request, "pause")


@router.post("/sessions/{session_id}/step")
def step_session(session_id: str, request: Request) -> CommandResponse:
    return _cmd(session_id, request, "step")


@router.post("/sessions/{session_id}/reset")
def reset_session(session_id: str, request: Request) -> CommandResponse:
    return _cmd(session_id, request, "reset")


@router.post("/sessions/{session_id}/stop")
def stop_session(session_id: str, request: Request) -> CommandResponse:
    return _cmd(session_id, request, "stop")


@router.get("/tracks/{track_id}/geometry")
def track_geometry(track_id: str) -> dict[str, Any]:
    """Return TrackDefinition JSON (must include centerline_m), not scenarios."""
    try:
        return config_store.load_track(track_id).model_dump()
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


@router.get("/experiments")
def experiments(request: Request) -> dict[str, Any]:
    return {"experiments": _replays(request).list()}


@router.get("/experiments/{experiment_id}")
def experiment_detail(experiment_id: str, request: Request) -> dict[str, Any]:
    items = _replays(request).list()
    for e in items:
        if e["id"] == experiment_id:
            return e
    raise HTTPException(status_code=404, detail="experiment not found")


@router.get("/experiments/{experiment_id}/series")
def experiment_series(experiment_id: str, request: Request) -> dict[str, Any]:
    try:
        return _replays(request).series(experiment_id)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


class ExperimentPatch(BaseModel):
    display_name: str | None = None


@router.patch("/experiments/{experiment_id}")
def patch_experiment(
    experiment_id: str, body: ExperimentPatch, request: Request
) -> dict[str, Any]:
    from backend.app.experiments.naming import normalize_display_name

    try:
        name = normalize_display_name(body.display_name)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    try:
        return _replays(request).set_display_name(experiment_id, name)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


@router.delete("/experiments/{experiment_id}")
def delete_experiment(experiment_id: str, request: Request) -> dict[str, Any]:
    # Drop live recorder / replay handles first (Windows WinError 32 otherwise).
    released = _manager(request).release_experiment(experiment_id)
    try:
        _replays(request).delete(experiment_id)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except OSError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True, "id": experiment_id, "released_sessions": released}


class ReplayCommand(BaseModel):
    action: Literal["play", "pause", "seek", "rate"]
    t_ns: int | None = None
    rate: float | None = None


@router.post("/experiments/{experiment_id}/replay")
def replay_command(experiment_id: str, body: ReplayCommand, request: Request) -> dict[str, Any]:
    try:
        player = _replays(request).get_or_create_player(experiment_id)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    if body.action == "play":
        player.play()
    elif body.action == "pause":
        player.pause()
    elif body.action == "seek":
        if body.t_ns is None:
            raise HTTPException(status_code=400, detail="t_ns required for seek")
        snap = player.seek(body.t_ns)
        return {"accepted": True, "status": player.status(), "snapshot": snap}
    elif body.action == "rate":
        if body.rate is None:
            raise HTTPException(status_code=400, detail="rate required")
        player.set_rate(body.rate)
    return {"accepted": True, "status": player.status()}


class EgoIcOverride(BaseModel):
    x_m: float | None = None
    y_m: float | None = None
    z_m: float | None = None
    yaw_rad: float | None = None
    speed_mps: float | None = None


class RunRequest(BaseModel):
    """Blocking one-shot experiment (no Live realtime). ``plant`` is required."""

    plant: Literal["mock", "carsim"]
    controller: Literal[
        "pure_pursuit",
        "lane_pp",
        "lattice_pid",
        "lattice_pid_ml",
        "stanley",
        "mppi",
        "constant",
        "strategy01",
    ] = (
        "pure_pursuit"
    )
    scenario_id: str = "web_demo_001"
    track_id: str | None = None
    display_name: str | None = None
    duration_s: float | None = Field(default=None, ge=0.5, le=600.0)
    record: bool = True
    controller_params: dict[str, Any] = Field(default_factory=dict)
    carsim_db: str | None = None
    ego_ic: EgoIcOverride | None = None


@router.post("/runs")
def runs_create(body: RunRequest, request: Request) -> dict[str, Any]:
    """Run one experiment to completion; returns experiment_id + metrics."""
    from backend.app.experiments.run_once import CarSimUnavailableError, run_experiment

    # Free CarSim DLL if a Live session still holds it.
    if body.plant == "carsim":
        _manager(request).close_all()
        _replays(request).close_all()

    ego_ic = body.ego_ic.model_dump(exclude_none=True) if body.ego_ic else None
    try:
        return run_experiment(
            plant=body.plant,
            controller=body.controller,
            scenario_id=body.scenario_id,
            track_id=body.track_id,
            display_name=body.display_name,
            duration_s=body.duration_s,
            record=body.record,
            controller_params=body.controller_params,
            carsim_db=Path(body.carsim_db) if body.carsim_db else None,
            ego_ic=ego_ic,
        )
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except CarSimUnavailableError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc


class SweepRequest(BaseModel):
    controller: Literal[
        "mppi",
        "pure_pursuit",
        "lane_pp",
        "lattice_pid",
        "lattice_pid_ml",
        "stanley",
        "constant",
        "strategy01",
    ] = "mppi"
    scenario_id: str = "web_demo_001"
    track_id: str | None = None
    duration_s: float = Field(default=20.0, ge=2.0, le=120.0)
    base_params: dict[str, Any] = Field(default_factory=dict)
    grid: dict[str, list[Any]] = Field(
        default_factory=lambda: {"num_samples": [32, 64], "horizon": [20, 30]}
    )


@router.get("/sweeps")
def sweeps_list() -> dict[str, Any]:
    from backend.app.experiments.sweep import list_sweeps

    return {"sweeps": list_sweeps()}


@router.get("/sweeps/{sweep_id}")
def sweeps_get(sweep_id: str) -> dict[str, Any]:
    from backend.app.experiments.sweep import load_sweep

    try:
        return load_sweep(sweep_id)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc


@router.post("/sweeps")
def sweeps_run(body: SweepRequest) -> dict[str, Any]:
    """Synchronous param-grid sweep (mock plant). May take tens of seconds."""
    from backend.app.experiments.sweep import run_param_sweep

    try:
        return run_param_sweep(
            grid=body.grid,
            base_params=body.base_params,
            controller=body.controller,
            scenario_id=body.scenario_id,
            track_id=body.track_id,
            duration_s=body.duration_s,
        )
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc
