"""Validate track + scenario consistency before run / save."""

from __future__ import annotations

from backend.app.scenario.track_query import TrackQuery
from backend.app.schemas.scenario import ScenarioConfig
from backend.app.schemas.track import TrackDefinition


class ScenarioValidationError(ValueError):
    pass


def validate_track(track: TrackDefinition) -> list[str]:
    """Return warnings; raises ScenarioValidationError on hard failures."""
    # Pydantic already enforces basics; extra soft checks here.
    warnings: list[str] = []
    if track.width_m < 3.0:
        warnings.append("track width_m < 3 m may be too narrow")
    return warnings


def validate_scenario(track: TrackDefinition, scenario: ScenarioConfig) -> list[str]:
    if scenario.track_id != track.track_id:
        raise ScenarioValidationError(
            f"scenario.track_id={scenario.track_id} != track.track_id={track.track_id}"
        )
    query = TrackQuery(track)
    warnings: list[str] = []
    pose = scenario.ego.initial_pose
    if not query.is_inside(pose.x_m, pose.y_m, margin_m=1.0):
        warnings.append("ego initial pose is outside track (+1 m margin)")
    for npc in scenario.npcs:
        for i, p in enumerate(npc.trajectory_xy):
            if not query.is_inside(p[0], p[1], margin_m=2.0):
                warnings.append(
                    f"NPC {npc.id} trajectory point {i} off-track (+2 m); "
                    "prefer empty trajectory_xy + lane_index"
                )
                break
    for obs in scenario.obstacles:
        if not query.is_inside(obs.x_m, obs.y_m, margin_m=2.0):
            warnings.append(f"obstacle {obs.id} center off-track (+2 m)")
    return warnings
