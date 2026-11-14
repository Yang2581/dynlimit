/** Closest-point projection onto a 2D polyline (platform XY). */

export type PolyPoint = number[] | { 0: number; 1: number };

export type SnapResult = {
  x_m: number;
  y_m: number;
  station_m: number;
  yaw_rad: number;
};

function xy(p: PolyPoint): [number, number] {
  return [Number(p[0]), Number(p[1])];
}

/** Project (x,y) onto polyline; yaw = tangent of the winning segment. */
export function snapToPolyline(
  x_m: number,
  y_m: number,
  polyline: PolyPoint[] | null | undefined
): SnapResult | null {
  if (!polyline || polyline.length < 2) return null;
  const pts = polyline.map(xy).filter(([a, b]) => Number.isFinite(a) && Number.isFinite(b));
  if (pts.length < 2) return null;

  let bestS = 0;
  let bestD2 = Infinity;
  let bestX = pts[0][0];
  let bestY = pts[0][1];
  let bestYaw = Math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0]);
  let sAt = 0;

  for (let i = 0; i < pts.length - 1; i++) {
    const [x0, y0] = pts[i];
    const [x1, y1] = pts[i + 1];
    const dx = x1 - x0;
    const dy = y1 - y0;
    const L2 = dx * dx + dy * dy;
    if (L2 < 1e-18) continue;
    let t = ((x_m - x0) * dx + (y_m - y0) * dy) / L2;
    t = Math.max(0, Math.min(1, t));
    const px = x0 + t * dx;
    const py = y0 + t * dy;
    const d2 = (x_m - px) ** 2 + (y_m - py) ** 2;
    const L = Math.sqrt(L2);
    if (d2 < bestD2) {
      bestD2 = d2;
      bestS = sAt + t * L;
      bestX = px;
      bestY = py;
      bestYaw = Math.atan2(dy, dx);
    }
    sAt += L;
  }

  return { x_m: bestX, y_m: bestY, station_m: bestS, yaw_rad: bestYaw };
}

export function referencePolyline(
  track: {
    reference_line_m?: number[][];
    centerline_m?: number[][];
  } | null | undefined
): number[][] | null {
  if (!track) return null;
  if (track.reference_line_m && track.reference_line_m.length >= 2) {
    return track.reference_line_m;
  }
  if (track.centerline_m && track.centerline_m.length >= 2) {
    return track.centerline_m;
  }
  return null;
}
