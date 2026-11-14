"""Lane-local reference planner: FOLLOW / adjacent LC queue → one polyline."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Sequence

from backend.app.planning.strategies.strategy01.geometry import project_to_polyline
from backend.app.scenario.lanes import LaneGeometry

_EPS = 1e-9


@dataclass
class LaneLocalConfig:
    horizon_m: float = 40.0
    lc_length_m: float = 30.0
    arrive_lat_m: float = 0.6
    arrive_yaw_rad: float = 0.15
    ds_m: float = 1.0
    arrive_s_frac: float = 0.85


@dataclass
class LaneLocalOutput:
    local_path_xy: list[list[float]]
    active_lane: int
    target_queue: list[int]
    lc_phase: str  # follow | lane_change
    planner_mode: str  # single_ref | lane_local
    diagnostics: dict[str, Any] = field(default_factory=dict)


def adjacent_queue(from_lane: int, to_lane: int) -> list[int]:
    """Expand absolute target into adjacent steps (exclusive of from_lane)."""
    if to_lane == from_lane:
        return []
    step = 1 if to_lane > from_lane else -1
    return list(range(from_lane + step, to_lane + step, step))


def _yaw_err(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def _polyline_s_table(pts: Sequence[Sequence[float]]) -> tuple[list[tuple[float, float]], list[float]]:
    xy = [(float(p[0]), float(p[1])) for p in pts if len(p) >= 2]
    if not xy:
        return [], [0.0]
    s = [0.0]
    for i in range(len(xy) - 1):
        s.append(s[-1] + math.hypot(xy[i + 1][0] - xy[i][0], xy[i + 1][1] - xy[i][1]))
    return xy, s


def _pose_at_s_open(
    xy: list[tuple[float, float]], s_table: list[float], s_m: float
) -> tuple[float, float, float]:
    """(x, y, yaw) at arc length along open polyline (clamped)."""
    if not xy:
        return 0.0, 0.0, 0.0
    if len(xy) == 1:
        return xy[0][0], xy[0][1], 0.0
    total = s_table[-1]
    s = max(0.0, min(float(s_m), total))
    # find segment
    i = 0
    for j in range(len(s_table) - 1):
        if s_table[j + 1] + _EPS >= s:
            i = j
            break
        i = j
    s0, s1 = s_table[i], s_table[i + 1]
    seg_len = max(s1 - s0, _EPS)
    t = (s - s0) / seg_len
    x0, y0 = xy[i]
    x1, y1 = xy[i + 1]
    yaw = math.atan2(y1 - y0, x1 - x0)
    return x0 + t * (x1 - x0), y0 + t * (y1 - y0), yaw


def _clip_forward(
    pts: Sequence[Sequence[float]],
    s0: float,
    horizon_m: float,
    ds_m: float,
    *,
    closed_loop: bool,
) -> list[list[float]]:
    xy, s_table = _polyline_s_table(pts)
    if len(xy) < 2:
        return [[p[0], p[1]] for p in xy] if xy else [[0.0, 0.0], [1.0, 0.0]]
    total = s_table[-1]
    if total < _EPS:
        return [[xy[0][0], xy[0][1]], [xy[-1][0], xy[-1][1]]]
    step = max(float(ds_m), 0.25)
    horizon = max(float(horizon_m), step)
    out: list[list[float]] = []
    n = int(math.ceil(horizon / step)) + 1
    for i in range(n):
        s = s0 + i * step
        if closed_loop and total > _EPS:
            s = s % total
            if s < 0:
                s += total
        elif s > total:
            x, y, _ = _pose_at_s_open(xy, s_table, total)
            out.append([x, y])
            break
        x, y, _ = _pose_at_s_open(xy, s_table, s)
        out.append([x, y])
    if len(out) < 2:
        x, y, yaw = _pose_at_s_open(xy, s_table, min(s0 + step, total))
        out.append([x + 0.1 * math.cos(yaw), y + 0.1 * math.sin(yaw)])
    return out


def nearest_lane_index(
    x: float, y: float, lanes: Sequence[LaneGeometry]
) -> tuple[int, float]:
    """Return (lane_index, signed lateral m on that lane)."""
    best_i = 0
    best_d = float("inf")
    best_lat = 0.0
    for L in lanes:
        s_m, _idx, px, py, tx, ty, nx, ny = project_to_polyline(x, y, L.centerline_m)
        del s_m, _idx, tx, ty
        lat = (x - px) * nx + (y - py) * ny
        d = abs(lat)
        # Prefer true distance if projection is odd
        d_eucl = math.hypot(x - px, y - py)
        score = min(d, d_eucl)
        if score < best_d:
            best_d = score
            best_i = int(L.index)
            best_lat = lat
    return best_i, best_lat


class LaneLocalPlanner:
    """FSM: FOLLOW ↔ LANE_CHANGE with adjacent target_queue."""

    def __init__(
        self,
        lanes: Sequence[LaneGeometry],
        *,
        closed_loop: bool = False,
        config: LaneLocalConfig | None = None,
        global_ref_m: Sequence[Sequence[float]] | None = None,
    ) -> None:
        self._lanes = list(lanes)
        self._closed = bool(closed_loop)
        self.cfg = config or LaneLocalConfig()
        self._global_ref = [list(p) for p in global_ref_m] if global_ref_m else None
        self.active_lane = 0
        self.target_queue: list[int] = []
        self.lc_phase = "follow"
        self._initialized = False
        self._lc_from = 0
        self._lc_to = 0
        self._lc_s0 = 0.0
        self._lc_path: list[list[float]] = []
        self._pending_external: int | None = None

    @property
    def planner_mode(self) -> str:
        if len(self._lanes) <= 1:
            return "single_ref"
        return "lane_local"

    def reset(self) -> None:
        self.active_lane = 0
        self.target_queue = []
        self.lc_phase = "follow"
        self._initialized = False
        self._lc_path = []
        self._pending_external = None

    def set_target_lane(self, lane_index: int | None) -> None:
        """External absolute target; applied when not mid-LC (or after current LC)."""
        if lane_index is None:
            self._pending_external = None
            return
        self._pending_external = int(lane_index)

    def _apply_external(self) -> None:
        if self._pending_external is None:
            return
        if self.lc_phase == "lane_change":
            return  # finish current adjacent LC first
        n = len(self._lanes)
        if n <= 0:
            return
        goal = max(0, min(int(self._pending_external), n - 1))
        self.target_queue = adjacent_queue(self.active_lane, goal)
        self._pending_external = None

    def _ensure_init(self, x: float, y: float) -> None:
        if self._initialized:
            return
        if not self._lanes:
            self.active_lane = 0
            self._initialized = True
            return
        self.active_lane, _ = nearest_lane_index(x, y, self._lanes)
        self._initialized = True

    def _lane(self, idx: int) -> LaneGeometry:
        return self._lanes[idx]

    def _project_lane(self, x: float, y: float, lane_idx: int):
        L = self._lane(lane_idx)
        return project_to_polyline(x, y, L.centerline_m)

    def _build_lc_path(self, x: float, y: float, yaw: float, from_i: int, to_i: int) -> list[list[float]]:
        L_from = self._lane(from_i)
        L_to = self._lane(to_i)
        s0, *_rest = project_to_polyline(x, y, L_from.centerline_m)
        self._lc_s0 = s0
        lc_len = max(float(self.cfg.lc_length_m), 5.0)
        ds = max(float(self.cfg.ds_m), 0.5)
        n = max(2, int(math.ceil(lc_len / ds)) + 1)
        xy_f, s_f = _polyline_s_table(L_from.centerline_m)
        xy_t, s_t = _polyline_s_table(L_to.centerline_m)
        path: list[list[float]] = [[float(x), float(y)]]
        for i in range(1, n):
            alpha = _smoothstep(i / (n - 1))
            s = s0 + (i / (n - 1)) * lc_len
            fx, fy, fyaw = _pose_at_s_open(xy_f, s_f, s)
            tx, ty, tyaw = _pose_at_s_open(xy_t, s_t, s)
            bx = (1.0 - alpha) * fx + alpha * tx
            by = (1.0 - alpha) * fy + alpha * ty
            # Seed first blend toward current pose continuity
            if i == 1:
                bx = 0.5 * bx + 0.5 * x
                by = 0.5 * by + 0.5 * y
            del fyaw, tyaw
            path.append([bx, by])
        # Ensure last point sits on target lane
        end_s = s0 + lc_len
        ex, ey, _ = _pose_at_s_open(xy_t, s_t, end_s)
        path[-1] = [ex, ey]
        if len(path) < 2:
            path.append([ex + math.cos(yaw), ey + math.sin(yaw)])
        return path

    def _arrived(self, x: float, y: float, yaw: float, lane_idx: int) -> bool:
        _s, _i, px, py, tx, ty, nx, ny = project_to_polyline(
            x, y, self._lane(lane_idx).centerline_m
        )
        lat = (x - px) * nx + (y - py) * ny
        path_yaw = math.atan2(ty, tx)
        if abs(lat) > self.cfg.arrive_lat_m:
            return False
        if abs(_yaw_err(yaw, path_yaw)) > self.cfg.arrive_yaw_rad:
            return False
        if self._lc_path and self.cfg.arrive_s_frac > 0:
            total = 0.0
            for k in range(len(self._lc_path) - 1):
                total += math.hypot(
                    self._lc_path[k + 1][0] - self._lc_path[k][0],
                    self._lc_path[k + 1][1] - self._lc_path[k][1],
                )
            total = max(total, _EPS)
            progress = 0.0
            best_d = float("inf")
            acc = 0.0
            for j in range(len(self._lc_path) - 1):
                ax, ay = self._lc_path[j]
                bx, by = self._lc_path[j + 1]
                seg = math.hypot(bx - ax, by - ay)
                if seg > _EPS:
                    t = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / (seg * seg)
                    t = max(0.0, min(1.0, t))
                    qx = ax + t * (bx - ax)
                    qy = ay + t * (by - ay)
                    d = math.hypot(x - qx, y - qy)
                    if d < best_d:
                        best_d = d
                        progress = (acc + t * seg) / total
                acc += seg
            if progress < self.cfg.arrive_s_frac and abs(lat) > 0.25:
                return False
        return True

    def _follow_path(self, x: float, y: float) -> list[list[float]]:
        if self.planner_mode == "single_ref":
            ref = self._global_ref
            if ref is None and self._lanes:
                ref = self._lanes[0].centerline_m
            if not ref:
                return [[x, y], [x + 1.0, y]]
            s0, *_ = project_to_polyline(x, y, ref)
            return _clip_forward(
                ref, s0, self.cfg.horizon_m, self.cfg.ds_m, closed_loop=self._closed
            )
        L = self._lane(self.active_lane)
        s0, *_ = project_to_polyline(x, y, L.centerline_m)
        return _clip_forward(
            L.centerline_m, s0, self.cfg.horizon_m, self.cfg.ds_m, closed_loop=self._closed
        )

    def step(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        target_lane_index: int | None = None,
        suggest_adjacent: int | None = None,
    ) -> LaneLocalOutput:
        """Advance FSM and emit local_path_xy for the controller."""
        if target_lane_index is not None:
            self.set_target_lane(target_lane_index)

        self._ensure_init(x, y)

        if self.planner_mode == "single_ref":
            path = self._follow_path(x, y)
            return LaneLocalOutput(
                local_path_xy=path,
                active_lane=self.active_lane,
                target_queue=list(self.target_queue),
                lc_phase="follow",
                planner_mode="single_ref",
                diagnostics={"mode": "single_ref"},
            )

        self._apply_external()

        # Auto intent: at most one adjacent suggestion while idle FOLLOW
        if (
            suggest_adjacent is not None
            and self.lc_phase == "follow"
            and not self.target_queue
            and self._pending_external is None
        ):
            adj = int(suggest_adjacent)
            if abs(adj - self.active_lane) == 1 and 0 <= adj < len(self._lanes):
                self.target_queue = [adj]

        if self.lc_phase == "follow":
            if self.target_queue:
                nxt = self.target_queue[0]
                if abs(nxt - self.active_lane) != 1:
                    # sanitize: rebuild from first valid adjacent toward goal
                    self.target_queue = adjacent_queue(self.active_lane, nxt)
                    if not self.target_queue:
                        path = self._follow_path(x, y)
                        return self._out(path)
                    nxt = self.target_queue[0]
                self._lc_from = self.active_lane
                self._lc_to = nxt
                self._lc_path = self._build_lc_path(x, y, yaw, self._lc_from, self._lc_to)
                self.lc_phase = "lane_change"
                return self._out(list(self._lc_path))

            return self._out(self._follow_path(x, y))

        # LANE_CHANGE
        if self._arrived(x, y, yaw, self._lc_to):
            self.active_lane = self._lc_to
            if self.target_queue and self.target_queue[0] == self._lc_to:
                self.target_queue = self.target_queue[1:]
            elif self.target_queue:
                # head was expected; drop if matches
                self.target_queue = [q for q in self.target_queue if q != self._lc_to]
                # rebuild remaining from new active
                if self.target_queue:
                    goal = self.target_queue[-1]
                    self.target_queue = adjacent_queue(self.active_lane, goal)
            self.lc_phase = "follow"
            self._lc_path = []
            self._apply_external()
            # Immediately start next adjacent LC if queued
            if self.target_queue:
                nxt = self.target_queue[0]
                if abs(nxt - self.active_lane) == 1:
                    self._lc_from = self.active_lane
                    self._lc_to = nxt
                    self._lc_path = self._build_lc_path(x, y, yaw, self._lc_from, self._lc_to)
                    self.lc_phase = "lane_change"
                    return self._out(list(self._lc_path))
            return self._out(self._follow_path(x, y))

        # Still changing: optionally refresh LC if drifted hard (v1: keep path)
        path = list(self._lc_path) if self._lc_path else self._follow_path(x, y)
        return self._out(path)

    def _out(self, path: list[list[float]]) -> LaneLocalOutput:
        return LaneLocalOutput(
            local_path_xy=path,
            active_lane=int(self.active_lane),
            target_queue=list(self.target_queue),
            lc_phase=str(self.lc_phase),
            planner_mode=self.planner_mode,
            diagnostics={
                "lc_from": self._lc_from if self.lc_phase == "lane_change" else None,
                "lc_to": self._lc_to if self.lc_phase == "lane_change" else None,
            },
        )
