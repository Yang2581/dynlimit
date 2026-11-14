import { useEffect, useRef, useState } from "react";

type Props = {
  t: number[];
  series: Array<{ name: string; values: number[]; color: string }>;
  cursorT?: number;
};

function niceNum(span: number, round: boolean): number {
  const exp = Math.floor(Math.log10(Math.max(Math.abs(span), 1e-12)));
  const f = span / 10 ** exp;
  let nf: number;
  if (round) {
    if (f < 1.5) nf = 1;
    else if (f < 3) nf = 2;
    else if (f < 7) nf = 5;
    else nf = 10;
  } else if (f <= 1) nf = 1;
  else if (f <= 2) nf = 2;
  else if (f <= 5) nf = 5;
  else nf = 10;
  return nf * 10 ** exp;
}

/** Generate ~nice tick values covering [min, max]. */
function niceTicks(min: number, max: number, maxTicks = 5): number[] {
  if (!Number.isFinite(min) || !Number.isFinite(max)) return [0];
  if (Math.abs(max - min) < 1e-15) {
    const v = min;
    const step = niceNum(Math.abs(v) || 1, true);
    return [v - step, v, v + step];
  }
  const range = niceNum(max - min, false);
  const step = niceNum(range / Math.max(1, maxTicks - 1), true);
  const lo = Math.floor(min / step) * step;
  const hi = Math.ceil(max / step) * step;
  const out: number[] = [];
  // Guard against float drift / huge ranges.
  for (let v = lo, i = 0; v <= hi + step * 0.5 && i < 20; v = lo + ++i * step) {
    out.push(Number(v.toPrecision(12)));
  }
  return out.length ? out : [min, max];
}

function fmtTick(v: number): string {
  const a = Math.abs(v);
  if (a >= 1000 || (a > 0 && a < 0.01)) return v.toExponential(1);
  if (a >= 100) return v.toFixed(0);
  if (a >= 10) return v.toFixed(1);
  if (a >= 1) return v.toFixed(2);
  return v.toFixed(3);
}

export function Curves({ t, series, cursorT }: Props) {
  const chartRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ w: 320, h: 140 });

  useEffect(() => {
    const el = chartRef.current;
    if (!el || typeof ResizeObserver === "undefined") return;
    const sync = () => {
      const rect = el.getBoundingClientRect();
      const w = Math.max(120, Math.floor(rect.width));
      const h = Math.max(80, Math.floor(rect.height));
      setSize((prev) => (prev.w === w && prev.h === h ? prev : { w, h }));
    };
    sync();
    const ro = new ResizeObserver(sync);
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  if (!t.length || !series.length) {
    return (
      <div className="curves empty">
        <div className="curves-chart" ref={chartRef}>
          No series
        </div>
      </div>
    );
  }

  const { w, h } = size;
  const padL = 46;
  const padR = series.length > 1 ? 46 : 14;
  const padT = 10;
  const padB = 28;
  const plotW = Math.max(1, w - padL - padR);
  const plotH = Math.max(1, h - padT - padB);

  const t0 = t[0];
  const t1 = t[t.length - 1] || t0 + 1;
  const tSpan = t1 - t0 || 1;

  const ranges = series.map((s) => {
    const finite = s.values.filter((v) => Number.isFinite(v));
    const ymin = finite.length ? Math.min(...finite) : 0;
    const ymax = finite.length ? Math.max(...finite) : 1;
    return { ymin, ymax, span: ymax - ymin || 1 };
  });

  const xTicks = niceTicks(t0, t1, 5);
  const yTicksL = niceTicks(ranges[0].ymin, ranges[0].ymax, 4);
  const yTicksR =
    series.length > 1 ? niceTicks(ranges[1].ymin, ranges[1].ymax, 4) : [];

  const xToPx = (tv: number) => padL + ((tv - t0) / tSpan) * plotW;
  const yToPx = (v: number, ri: number) => {
    const { ymin, span } = ranges[ri] ?? ranges[0];
    return padT + plotH - ((v - ymin) / span) * plotH;
  };

  const paths = series.map((s, si) => {
    const parts: string[] = [];
    let penDown = false;
    for (let i = 0; i < s.values.length; i++) {
      const v = s.values[i];
      if (!Number.isFinite(v) || !Number.isFinite(t[i])) {
        penDown = false;
        continue;
      }
      const x = xToPx(t[i]);
      const y = yToPx(v, si);
      parts.push(`${penDown ? "L" : "M"}${x.toFixed(1)},${y.toFixed(1)}`);
      penDown = true;
    }
    return { ...s, d: parts.join(" ") };
  });

  let cursorX: number | null = null;
  if (cursorT != null && cursorT >= t0 && cursorT <= t1) {
    cursorX = xToPx(cursorT);
  } else if (cursorT != null) {
    // Still show if slightly outside due to playback rounding.
    cursorX = xToPx(Math.min(t1, Math.max(t0, cursorT)));
  }

  return (
    <div className="curves">
      <div className="curves-chart" ref={chartRef}>
        <svg width={w} height={h} viewBox={`0 0 ${w} ${h}`}>
          {/* plot background */}
          <rect
            x={padL}
            y={padT}
            width={plotW}
            height={plotH}
            fill="rgba(0,0,0,0.2)"
            rx={4}
          />

          {/* vertical grid + x ticks */}
          {xTicks.map((tv) => {
            const x = xToPx(tv);
            if (x < padL - 0.5 || x > padL + plotW + 0.5) return null;
            return (
              <g key={`x-${tv}`}>
                <line
                  x1={x}
                  x2={x}
                  y1={padT}
                  y2={padT + plotH}
                  stroke="rgba(255,255,255,0.06)"
                  strokeWidth={1}
                />
                <text
                  x={x}
                  y={padT + plotH + 14}
                  textAnchor="middle"
                  fill="#9aa3b2"
                  fontSize={10}
                  fontFamily="ui-monospace, monospace"
                >
                  {fmtTick(tv)}
                </text>
              </g>
            );
          })}

          {/* horizontal grid + left y ticks (series 0) */}
          {yTicksL.map((v) => {
            const y = yToPx(v, 0);
            if (y < padT - 0.5 || y > padT + plotH + 0.5) return null;
            return (
              <g key={`yl-${v}`}>
                <line
                  x1={padL}
                  x2={padL + plotW}
                  y1={y}
                  y2={y}
                  stroke="rgba(255,255,255,0.06)"
                  strokeWidth={1}
                />
                <text
                  x={padL - 6}
                  y={y + 3}
                  textAnchor="end"
                  fill={series[0].color}
                  fontSize={10}
                  fontFamily="ui-monospace, monospace"
                >
                  {fmtTick(v)}
                </text>
              </g>
            );
          })}

          {/* right y ticks (series 1) */}
          {yTicksR.map((v) => {
            const y = yToPx(v, 1);
            if (y < padT - 0.5 || y > padT + plotH + 0.5) return null;
            return (
              <text
                key={`yr-${v}`}
                x={padL + plotW + 6}
                y={y + 3}
                textAnchor="start"
                fill={series[1].color}
                fontSize={10}
                fontFamily="ui-monospace, monospace"
              >
                {fmtTick(v)}
              </text>
            );
          })}

          {/* axes */}
          <line
            x1={padL}
            x2={padL}
            y1={padT}
            y2={padT + plotH}
            stroke="rgba(255,255,255,0.25)"
            strokeWidth={1}
          />
          <line
            x1={padL}
            x2={padL + plotW}
            y1={padT + plotH}
            y2={padT + plotH}
            stroke="rgba(255,255,255,0.25)"
            strokeWidth={1}
          />
          {series.length > 1 && (
            <line
              x1={padL + plotW}
              x2={padL + plotW}
              y1={padT}
              y2={padT + plotH}
              stroke="rgba(255,255,255,0.2)"
              strokeWidth={1}
            />
          )}

          {/* series paths */}
          {paths.map((p) => (
            <path key={p.name} d={p.d} fill="none" stroke={p.color} strokeWidth="1.5" />
          ))}

          {cursorX != null && (
            <line
              x1={cursorX}
              x2={cursorX}
              y1={padT}
              y2={padT + plotH}
              stroke="#5eead4"
              strokeWidth="1"
              opacity="0.85"
            />
          )}

          <text
            x={padL + plotW / 2}
            y={h - 2}
            textAnchor="middle"
            fill="#6b7280"
            fontSize={9}
            fontFamily="ui-monospace, monospace"
          >
            t [s]
          </text>
        </svg>
      </div>
      <div className="curve-legend">
        {series.map((s, i) => (
          <span key={s.name} style={{ color: s.color }}>
            {s.name}
            {i === 0 ? " (L)" : i === 1 ? " (R)" : ""}
          </span>
        ))}
      </div>
    </div>
  );
}
