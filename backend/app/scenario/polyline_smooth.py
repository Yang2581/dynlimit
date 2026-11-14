"""Smooth polyline densification (centripetal Catmull-Rom → arc-length resample).

Authoring polylines stay sparse in track JSON; runtime / control / lane
geometry should call ``smooth_resample_polyline`` so corners are curved and
spacing ≈ ``resolution_m``.

Centripetal parameterization reduces overshoot / local reversals that break
lane offset normals (NPC heading spikes).
"""

from __future__ import annotations

import math
from typing import Sequence


def _as_xyz(pts: Sequence[Sequence[float]]) -> list[tuple[float, float, float]]:
    out: list[tuple[float, float, float]] = []
    for p in pts:
        if len(p) < 2:
            continue
        x, y = float(p[0]), float(p[1])
        z = float(p[2]) if len(p) > 2 else 0.0
        out.append((x, y, z))
    return out


def _strip_closing_dup(
    pts: list[tuple[float, float, float]], *, closed: bool
) -> list[tuple[float, float, float]]:
    if (
        closed
        and len(pts) > 2
        and math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]) < 1e-6
    ):
        return pts[:-1]
    return pts


def _tj(ti: float, pi: tuple[float, float, float], pj: tuple[float, float, float]) -> float:
    """Centripetal Catmull-Rom knot increment (α = 0.5)."""
    d = math.hypot(pj[0] - pi[0], pj[1] - pi[1])
    return ti + math.sqrt(max(d, 1e-12))


def _cr_centripetal(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    p2: tuple[float, float, float],
    p3: tuple[float, float, float],
    t: float,
) -> tuple[float, float, float]:
    """Sample centripetal CR on segment p1→p2, t∈[0,1]."""
    t0 = 0.0
    t1 = _tj(t0, p0, p1)
    t2 = _tj(t1, p1, p2)
    t3 = _tj(t2, p2, p3)
    if t2 - t1 < 1e-12:
        return p1
    tt = t1 + t * (t2 - t1)

    def _lerp(
        a: tuple[float, float, float],
        b: tuple[float, float, float],
        ta: float,
        tb: float,
        u: float,
    ) -> tuple[float, float, float]:
        if abs(tb - ta) < 1e-12:
            return a
        w = (u - ta) / (tb - ta)
        return (
            a[0] + w * (b[0] - a[0]),
            a[1] + w * (b[1] - a[1]),
            a[2] + w * (b[2] - a[2]),
        )

    a1 = _lerp(p0, p1, t0, t1, tt)
    a2 = _lerp(p1, p2, t1, t2, tt)
    a3 = _lerp(p2, p3, t2, t3, tt)
    b1 = _lerp(a1, a2, t0, t2, tt)
    b2 = _lerp(a2, a3, t1, t3, tt)
    return _lerp(b1, b2, t1, t2, tt)


def _segment_samples(
    ctrl: list[tuple[float, float, float]],
    *,
    closed: bool,
    samples_per_seg: int,
) -> list[tuple[float, float, float]]:
    n = len(ctrl)
    if n < 2:
        return list(ctrl)
    if n == 2:
        out = [ctrl[0]]
        for k in range(1, max(2, samples_per_seg) + 1):
            u = k / max(2, samples_per_seg)
            out.append(
                (
                    ctrl[0][0] + u * (ctrl[1][0] - ctrl[0][0]),
                    ctrl[0][1] + u * (ctrl[1][1] - ctrl[0][1]),
                    ctrl[0][2] + u * (ctrl[1][2] - ctrl[0][2]),
                )
            )
        return out

    def _at(i: int) -> tuple[float, float, float]:
        if closed:
            return ctrl[i % n]
        return ctrl[max(0, min(n - 1, i))]

    segs = n if closed else n - 1
    dense: list[tuple[float, float, float]] = []
    for i in range(segs):
        p0, p1, p2, p3 = _at(i - 1), _at(i), _at(i + 1), _at(i + 2)
        steps = max(2, samples_per_seg)
        t0 = 0 if i == 0 else 1
        for k in range(t0, steps + 1):
            dense.append(_cr_centripetal(p0, p1, p2, p3, k / steps))
    return dense


def _remove_local_reversals(
    dense: list[tuple[float, float, float]],
) -> list[tuple[float, float, float]]:
    """Drop samples that reverse travel sense vs the running chord direction."""
    if len(dense) < 3:
        return dense
    out: list[tuple[float, float, float]] = [dense[0]]
    # Establish a stable forward direction from the first non-trivial step.
    fx = fy = 0.0
    for p in dense[1:]:
        dx = p[0] - out[-1][0]
        dy = p[1] - out[-1][1]
        if math.hypot(dx, dy) < 1e-9:
            continue
        if fx == 0.0 and fy == 0.0:
            fx, fy = dx, dy
            out.append(p)
            continue
        # Reject near-180° folds relative to recent forward.
        if dx * fx + dy * fy < 0.0:
            continue
        # Blend forward estimate
        fx = 0.8 * fx + 0.2 * dx
        fy = 0.8 * fy + 0.2 * dy
        out.append(p)
    return out if len(out) >= 2 else dense


def _arc_length_resample(
    dense: list[tuple[float, float, float]],
    *,
    closed: bool,
    resolution_m: float,
) -> list[list[float]]:
    if len(dense) < 2:
        return [[p[0], p[1], p[2]] for p in dense]
    if closed and math.hypot(dense[-1][0] - dense[0][0], dense[-1][1] - dense[0][1]) > 1e-6:
        dense = list(dense) + [dense[0]]

    cum = [0.0]
    for i in range(len(dense) - 1):
        cum.append(
            cum[-1]
            + math.hypot(dense[i + 1][0] - dense[i][0], dense[i + 1][1] - dense[i][1])
        )
    total = cum[-1]
    if total < 1e-9:
        return [[dense[0][0], dense[0][1], dense[0][2]]]

    target_len = total if not closed else max(total - 1e-6, 0.0)
    n_out = max(3, int(math.ceil(target_len / resolution_m)) + 1)
    out: list[list[float]] = []
    j = 0
    for i in range(n_out):
        s_target = min(target_len, (i / (n_out - 1)) * target_len) if n_out > 1 else 0.0
        while j < len(cum) - 2 and cum[j + 1] < s_target:
            j += 1
        s0, s1 = cum[j], cum[j + 1]
        seg = s1 - s0
        u = 0.0 if seg < 1e-12 else (s_target - s0) / seg
        a, b = dense[j], dense[j + 1]
        out.append(
            [
                a[0] + u * (b[0] - a[0]),
                a[1] + u * (b[1] - a[1]),
                a[2] + u * (b[2] - a[2]),
            ]
        )
    if closed and out:
        if math.hypot(out[-1][0] - out[0][0], out[-1][1] - out[0][1]) > 1e-4:
            out.append([out[0][0], out[0][1], out[0][2]])
    return out


def smooth_resample_polyline(
    pts: Sequence[Sequence[float]],
    *,
    closed: bool,
    resolution_m: float = 0.5,
    samples_per_seg: int | None = None,
) -> list[list[float]]:
    """Centripetal Catmull-Rom smooth then resample at ~resolution_m spacing.

    Returns a denser polyline suitable for Pure Pursuit / lane offsets.
    Input control vertices are not mutated.
    """
    ctrl = _strip_closing_dup(_as_xyz(pts), closed=closed)
    if len(ctrl) < 2:
        return [[p[0], p[1], p[2]] for p in ctrl]
    res = max(float(resolution_m), 0.05)
    if samples_per_seg is None:
        lens = [
            math.hypot(
                ctrl[(i + 1) % len(ctrl)][0] - ctrl[i][0],
                ctrl[(i + 1) % len(ctrl)][1] - ctrl[i][1],
            )
            for i in range(len(ctrl) if closed else len(ctrl) - 1)
        ]
        mean_seg = sum(lens) / max(len(lens), 1)
        samples_per_seg = max(8, int(math.ceil(mean_seg / res)) * 4)

    dense = _segment_samples(ctrl, closed=closed, samples_per_seg=samples_per_seg)
    dense = _remove_local_reversals(dense)
    return _arc_length_resample(dense, closed=closed, resolution_m=res)
