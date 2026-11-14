"""Filesystem CRUD for tracks and scenarios under configs/."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

from backend.app.platform.paths import get_paths
from backend.app.scenario.validator import ScenarioValidationError, validate_scenario, validate_track
from backend.app.schemas.scenario import ScenarioConfig
from backend.app.schemas.track import TrackDefinition

_ID_RE = re.compile(r"^[a-zA-Z0-9_\-]+$")


def _tracks_dir() -> Path:
    return get_paths().tracks_dir


def _scenarios_dir() -> Path:
    return get_paths().scenarios_dir


def _config_root() -> Path:
    return get_paths().config_root


def ensure_dirs() -> None:
    _tracks_dir().mkdir(parents=True, exist_ok=True)
    _scenarios_dir().mkdir(parents=True, exist_ok=True)


def _safe_id(id_: str) -> str:
    if not _ID_RE.match(id_):
        raise ValueError(f"invalid id '{id_}' (use alphanumeric, _, -)")
    return id_


def track_path(track_id: str) -> Path:
    return _tracks_dir() / f"{_safe_id(track_id)}.json"


def scenario_path(scenario_id: str) -> Path:
    return _scenarios_dir() / f"{_safe_id(scenario_id)}.json"


def resolve_track_file(track_id: str) -> Path:
    """Prefer configs/tracks/{id}.json; fall back to legacy configs/*track*."""
    ensure_dirs()
    p = track_path(track_id)
    if p.exists():
        return p
    legacy = _config_root() / "oval_track.json"
    if track_id == "oval_01" and legacy.exists():
        return legacy
    for cand in _config_root().glob("*track*.json"):
        try:
            data = json.loads(cand.read_text(encoding="utf-8"))
            if data.get("track_id") == track_id:
                return cand
        except Exception:
            continue
    raise FileNotFoundError(f"track not found: {track_id}")


def resolve_scenario_file(scenario_id: str) -> Path:
    ensure_dirs()
    p = scenario_path(scenario_id)
    if p.exists():
        return p
    for name in (f"{scenario_id}.json", "web_demo_scenario.json", "demo_scenario.json"):
        cand = _config_root() / name
        if cand.exists():
            data = json.loads(cand.read_text(encoding="utf-8"))
            if data.get("scenario_id") == scenario_id or cand.stem.startswith(scenario_id):
                return cand
    for cand in _config_root().glob("*scenario*.json"):
        try:
            data = json.loads(cand.read_text(encoding="utf-8"))
            if data.get("scenario_id") == scenario_id:
                return cand
        except Exception:
            continue
    raise FileNotFoundError(f"scenario not found: {scenario_id}")


def list_tracks() -> list[dict[str, Any]]:
    ensure_dirs()
    seen: set[str] = set()
    items: list[dict[str, Any]] = []
    for path in sorted(_tracks_dir().glob("*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        tid = data.get("track_id", path.stem)
        seen.add(tid)
        items.append({"id": tid, "path": str(path), "name": data.get("name"), "width_m": data.get("width_m")})
    # legacy
    for path in sorted(_config_root().glob("*track*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        tid = data.get("track_id", path.stem)
        if tid in seen:
            continue
        items.append({"id": tid, "path": str(path), "name": data.get("name"), "width_m": data.get("width_m")})
    return items


def list_scenarios() -> list[dict[str, Any]]:
    ensure_dirs()
    seen: set[str] = set()
    items: list[dict[str, Any]] = []
    for path in sorted(_scenarios_dir().glob("*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        sid = data.get("scenario_id", path.stem)
        seen.add(sid)
        items.append(
            {
                "id": sid,
                "path": str(path),
                "track_id": data.get("track_id"),
            }
        )
    for path in sorted(_config_root().glob("*scenario*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        sid = data.get("scenario_id", path.stem)
        if sid in seen:
            continue
        items.append({"id": sid, "path": str(path), "track_id": data.get("track_id")})
    return items


def load_track(track_id: str) -> TrackDefinition:
    return TrackDefinition.model_validate_json(resolve_track_file(track_id).read_text(encoding="utf-8"))


def load_scenario(scenario_id: str) -> ScenarioConfig:
    return ScenarioConfig.model_validate_json(
        resolve_scenario_file(scenario_id).read_text(encoding="utf-8")
    )


def save_track(track: TrackDefinition, *, overwrite: bool = True) -> Path:
    ensure_dirs()
    validate_track(track)
    path = track_path(track.track_id)
    if path.exists() and not overwrite:
        raise FileExistsError(track.track_id)
    path.write_text(track.model_dump_json(indent=2), encoding="utf-8")
    return path


def save_scenario(scenario: ScenarioConfig, *, overwrite: bool = True) -> tuple[Path, list[str]]:
    ensure_dirs()
    track = load_track(scenario.track_id)
    warnings = validate_scenario(track, scenario)
    path = scenario_path(scenario.scenario_id)
    if path.exists() and not overwrite:
        raise FileExistsError(scenario.scenario_id)
    path.write_text(scenario.model_dump_json(indent=2), encoding="utf-8")
    return path, warnings


def delete_track(track_id: str) -> None:
    path = track_path(track_id)
    if not path.exists():
        raise FileNotFoundError(track_id)
    path.unlink()


def delete_scenario(scenario_id: str) -> None:
    path = scenario_path(scenario_id)
    if not path.exists():
        raise FileNotFoundError(scenario_id)
    path.unlink()
