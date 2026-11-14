"""True multi-lane lattice: candidates are lane centerlines (adjacent only)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from backend.app.planning.lane_local_planner import (
    LaneLocalConfig,
    LaneLocalOutput,
    LaneLocalPlanner,
    adjacent_queue,
    nearest_lane_index,
)
from backend.app.planning.strategies.strategy01.geometry import circles_collide
from backend.app.planning.strategies.strategy01.lattice import (
    _npc_xy,
    _point_nearest_travel,
    obstacle_radius_m,
)
from backend.app.scenario.lanes import LaneGeometry

Npc = Mapping[str, object]


@dataclass
class MultiLaneLatticeConfig(LaneLocalConfig):
    ego_radius_m: float = 1.5
    npc_radius_m: float = 1.5
    predict_dt_s: float = 0.2
    w_change: float = 0.5
    v_min_mps: float = 1.0
    replan_every_n: int = 1


@dataclass
class MultiLaneLatticeOutput(LaneLocalOutput):
    chosen_lane: int = 0
    collision_free: bool = True
    n_candidates: int = 0


class MultiLaneLatticePlanner(LaneLocalPlanner):
    """FOLLOW / LC FSM with lattice over adjacent lane indices."""

    def __init__(
        self,
        lanes: Sequence[LaneGeometry],
        *,
        closed_loop: bool = False,
        config: MultiLaneLatticeConfig | None = None,
        global_ref_m: Sequence[Sequence[float]] | None = None,
    ) -> None:
        self.ml_cfg = config or MultiLaneLatticeConfig()
        super().__init__(
            lanes,
            closed_loop=closed_loop,
            config=self.ml_cfg,
            global_ref_m=global_ref_m,
        )
        self._tick = 0
        self._last_chosen = 0
        self._last_collision_free = True
        self._last_n_candidates = 0

    def reset(self) -> None:
        super().reset()
        self._tick = 0
        self._last_chosen = 0
        self._last_collision_free = True
        self._last_n_candidates = 0

    def _candidate_lanes(self) -> list[int]:
        n = len(self._lanes)
        if n <= 0:
            return []
        a = int(self.active_lane)
        out = {a}
        if a - 1 >= 0:
            out.add(a - 1)
        if a + 1 < n:
            out.add(a + 1)
        return sorted(out)

    def _path_for_lane(self, x: float, y: float, yaw: float, lane_idx: int) -> list[list[float]]:
        if lane_idx == self.active_lane:
            # Temporarily clip active lane
            saved = self.active_lane
            self.active_lane = lane_idx
            path = self._follow_path(x, y)
            self.active_lane = saved
            return path
        # Short LC blend toward candidate (preview only; does not commit FSM)
        return self._build_lc_path(x, y, yaw, self.active_lane, lane_idx)

    def _score_path(
        self,
        path: list[list[float]],
        *,
        ego_v: float,
        npcs: Sequence[Npc],
        obstacles: Sequence[Npc],
        lane_idx: int,
    ) -> tuple[bool, int, float]:
        """Return (legal, illegal_count, cost)."""
        v = max(float(ego_v), self.ml_cfg.v_min_mps)
        horizon = float(self.ml_cfg.horizon_m)
        T = horizon / v if v > 0 else 0.0
        dt = max(float(self.ml_cfg.predict_dt_s), 1e-3)
        times = [0.0]
        t = dt
        while t <= T + 1e-12:
            times.append(t)
            t += dt
        pts = [(float(p[0]), float(p[1])) for p in path if len(p) >= 2]
        if len(pts) < 2:
            return False, 999, 1e9
        illegal = 0
        for ti in times:
            ex, ey = _point_nearest_travel(pts, v * ti)
            for npc in npcs:
                nx, ny = _npc_xy(npc, ti)
                if circles_collide(
                    ex, ey, self.ml_cfg.ego_radius_m, nx, ny, self.ml_cfg.npc_radius_m
                ):
                    illegal += 1
            for obs in obstacles:
                ox = float(obs.get("x_m", 0.0) or 0.0)
                oy = float(obs.get("y_m", 0.0) or 0.0)
                orad = obstacle_radius_m(obs)
                if circles_collide(ex, ey, self.ml_cfg.ego_radius_m, ox, oy, orad):
                    illegal += 1
        cost = self.ml_cfg.w_change * abs(lane_idx - self.active_lane)
        # Prefer staying if last chosen matches
        cost += 0.05 * abs(lane_idx - self._last_chosen)
        return illegal == 0, illegal, cost

    def step(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        target_lane_index: int | None = None,
        suggest_adjacent: int | None = None,
        ego_v: float = 0.0,
        npcs: Sequence[Npc] | None = None,
        obstacles: Sequence[Npc] | None = None,
    ) -> MultiLaneLatticeOutput:
        npc_list = list(npcs or [])
        obs_list = list(obstacles or [])

        # External / single-lane / LC tracking via base FSM first for non-lattice bits
        if target_lane_index is not None:
            self.set_target_lane(target_lane_index)

        self._ensure_init(x, y)
        if self.planner_mode == "single_ref":
            base = super().step(x, y, yaw, target_lane_index=None, suggest_adjacent=None)
            return MultiLaneLatticeOutput(
                local_path_xy=base.local_path_xy,
                active_lane=base.active_lane,
                target_queue=base.target_queue,
                lc_phase=base.lc_phase,
                planner_mode=base.planner_mode,
                diagnostics=base.diagnostics,
                chosen_lane=base.active_lane,
                collision_free=True,
                n_candidates=1,
            )

        # Auto lattice only in FOLLOW with empty queue and no pending external
        do_lattice = (
            self.lc_phase == "follow"
            and not self.target_queue
            and self._pending_external is None
            and suggest_adjacent is None
            and self._tick % max(1, int(self.ml_cfg.replan_every_n)) == 0
        )
        chosen = self.active_lane
        if do_lattice:
            scored: list[tuple[bool, int, float, int, list[list[float]]]] = []
            for li in self._candidate_lanes():
                path = self._path_for_lane(x, y, yaw, li)
                legal, illegal, cost = self._score_path(
                    path, ego_v=ego_v, npcs=npc_list, obstacles=obs_list, lane_idx=li
                )
                scored.append((legal, illegal, cost, li, path))
            self._last_n_candidates = len(scored)
            if scored:
                legal_ones = [c for c in scored if c[0]]
                if legal_ones:
                    _lg, _il, _c, chosen, _p = min(legal_ones, key=lambda c: (c[2], abs(c[3] - self.active_lane)))
                    self._last_collision_free = True
                else:
                    _lg, _il, _c, chosen, _p = min(
                        scored, key=lambda c: (c[1], abs(c[3] - self.active_lane), c[2])
                    )
                    self._last_collision_free = False
                self._last_chosen = chosen
                if chosen != self.active_lane and abs(chosen - self.active_lane) == 1:
                    # Commit adjacent LC via base queue
                    suggest_adjacent = chosen

        base = super().step(
            x, y, yaw, target_lane_index=None, suggest_adjacent=suggest_adjacent
        )
        self._tick += 1
        diag = dict(base.diagnostics or {})
        diag.update(
            {
                "chosen_lane": self._last_chosen,
                "collision_free": self._last_collision_free,
                "n_candidates": self._last_n_candidates,
            }
        )
        return MultiLaneLatticeOutput(
            local_path_xy=base.local_path_xy,
            active_lane=base.active_lane,
            target_queue=base.target_queue,
            lc_phase=base.lc_phase,
            planner_mode=base.planner_mode,
            diagnostics=diag,
            chosen_lane=int(self._last_chosen),
            collision_free=bool(self._last_collision_free),
            n_candidates=int(self._last_n_candidates),
        )
