import { useMemo, useRef, useState, type PointerEvent as ReactPointerEvent } from "react";
import type { SeriesPayload } from "./api";
import { Curves } from "./Curves";

export const PLOT_COLORS = [
  "#5eead4",
  "#ffb020",
  "#5b8cff",
  "#f472b6",
  "#a3e635",
  "#c084fc",
  "#fb7185",
  "#38bdf8",
];

export type PlotWindowState = {
  id: string;
  x: number;
  y: number;
  w: number;
  h: number;
  z: number;
  channels: string[];
  /** Normalized [0,1] window over full series time. */
  range: [number, number];
};

type Props = {
  win: PlotWindowState;
  series: SeriesPayload;
  channelOptions: string[];
  cursorT?: number;
  onChange: (next: PlotWindowState) => void;
  onClose: () => void;
  onFocus: () => void;
};

function sliceSeries(
  series: SeriesPayload,
  channels: string[],
  range: [number, number]
): {
  t: number[];
  plotted: Array<{ name: string; values: number[]; color: string }>;
  label: string;
} {
  const tAll = series.series.t_s ?? [];
  if (!tAll.length) {
    return { t: [], plotted: [], label: "—" };
  }
  const t0 = tAll[0];
  const t1 = tAll[tAll.length - 1] ?? t0;
  const span = t1 - t0 || 1;
  const lo = t0 + range[0] * span;
  const hi = t0 + range[1] * span;
  const idx: number[] = [];
  for (let i = 0; i < tAll.length; i++) {
    if (tAll[i] >= lo && tAll[i] <= hi) idx.push(i);
  }
  if (idx.length < 2) {
    const mid = Math.max(
      0,
      Math.min(
        tAll.length - 1,
        Math.floor(((range[0] + range[1]) * 0.5) * (tAll.length - 1))
      )
    );
    idx.length = 0;
    idx.push(Math.max(0, mid - 1), mid);
  }
  const t = idx.map((i) => tAll[i]);
  const plotted = channels
    .filter((name) => Array.isArray(series.series[name]))
    .slice(0, PLOT_COLORS.length)
    .map((name, i) => ({
      name,
      values: idx.map((j) => {
        const raw = series.series[name]![j];
        if (raw == null) return Number.NaN;
        const v = Number(raw);
        return Number.isFinite(v) ? v : Number.NaN;
      }),
      color: PLOT_COLORS[i % PLOT_COLORS.length],
    }));
  const label = `${lo.toFixed(1)}–${hi.toFixed(1)}s · ${t.length} pts`;
  return { t, plotted, label };
}

export function PlotWindow({
  win,
  series,
  channelOptions,
  cursorT,
  onChange,
  onClose,
  onFocus,
}: Props) {
  const winRef = useRef(win);
  winRef.current = win;
  const onChangeRef = useRef(onChange);
  onChangeRef.current = onChange;
  const dragRef = useRef<{
    mode: "move" | "resize";
    pointerId: number;
    ox: number;
    oy: number;
    sx: number;
    sy: number;
    sw: number;
    sh: number;
  } | null>(null);
  const [channelsOpen, setChannelsOpen] = useState(true);

  const sliced = useMemo(
    () => sliceSeries(series, win.channels, win.range),
    [series, win.channels, win.range]
  );

  const beginDrag = (mode: "move" | "resize", ev: ReactPointerEvent<HTMLElement>) => {
    // Ignore non-primary buttons (right/middle).
    if (ev.button !== 0) return;
    ev.preventDefault();
    ev.stopPropagation();
    onFocus();
    const cur = winRef.current;
    dragRef.current = {
      mode,
      pointerId: ev.pointerId,
      ox: ev.clientX,
      oy: ev.clientY,
      sx: cur.x,
      sy: cur.y,
      sw: cur.w,
      sh: cur.h,
    };
    try {
      ev.currentTarget.setPointerCapture(ev.pointerId);
    } catch {
      /* ignore — capture unsupported */
    }
  };

  const onDragMove = (ev: ReactPointerEvent<HTMLElement>) => {
    const d = dragRef.current;
    if (!d || d.pointerId !== ev.pointerId) return;
    const cur = winRef.current;
    if (d.mode === "move") {
      onChangeRef.current({
        ...cur,
        x: Math.max(0, d.sx + (ev.clientX - d.ox)),
        y: Math.max(0, d.sy + (ev.clientY - d.oy)),
      });
    } else {
      onChangeRef.current({
        ...cur,
        w: Math.max(280, d.sw + (ev.clientX - d.ox)),
        h: Math.max(200, d.sh + (ev.clientY - d.oy)),
      });
    }
  };

  const endDrag = (ev: ReactPointerEvent<HTMLElement>) => {
    const d = dragRef.current;
    if (!d || d.pointerId !== ev.pointerId) return;
    dragRef.current = null;
    try {
      if (ev.currentTarget.hasPointerCapture?.(ev.pointerId)) {
        ev.currentTarget.releasePointerCapture(ev.pointerId);
      }
    } catch {
      /* ignore */
    }
  };

  const title =
    win.channels.length > 0 ? win.channels.slice(0, 3).join(" · ") : "Plot";

  return (
    <div
      className="plot-window"
      style={{
        left: win.x,
        top: win.y,
        width: win.w,
        height: win.h,
        zIndex: 40 + win.z,
      }}
      onPointerDown={() => onFocus()}
    >
      <div
        className="plot-window-title"
        onPointerDown={(e) => {
          if ((e.target as HTMLElement | null)?.closest?.(".plot-window-actions")) return;
          beginDrag("move", e);
        }}
        onPointerMove={onDragMove}
        onPointerUp={endDrag}
        onPointerCancel={endDrag}
      >
        <span className="plot-window-title-text" title={title}>
          {title}
        </span>
        <div
          className="plot-window-actions"
          onPointerDown={(e) => e.stopPropagation()}
        >
          <button
            type="button"
            className={channelsOpen ? "active" : ""}
            title="Toggle channels"
            onClick={() => setChannelsOpen((v) => !v)}
          >
            Ch
          </button>
          <button type="button" title="Close" onClick={onClose}>
            ×
          </button>
        </div>
      </div>

      <div className="plot-window-body">
        <div className="curve-range" title="Time window within saved series">
          <label>
            t₀
            <input
              type="range"
              min={0}
              max={1000}
              value={Math.round(win.range[0] * 1000)}
              onChange={(e) => {
                const a = Number(e.target.value) / 1000;
                onChange({
                  ...win,
                  range: [Math.min(a, win.range[1] - 0.01), win.range[1]],
                });
              }}
            />
          </label>
          <label>
            t₁
            <input
              type="range"
              min={0}
              max={1000}
              value={Math.round(win.range[1] * 1000)}
              onChange={(e) => {
                const b = Number(e.target.value) / 1000;
                onChange({
                  ...win,
                  range: [win.range[0], Math.max(b, win.range[0] + 0.01)],
                });
              }}
            />
          </label>
          <span className="curve-range-label">{sliced.label}</span>
        </div>

        {channelsOpen && (
          <div className="curve-channels">
            {channelOptions.map((key) => {
              const on = win.channels.includes(key);
              return (
                <label key={key} className={on ? "on" : ""}>
                  <input
                    type="checkbox"
                    checked={on}
                    onChange={() => {
                      const next = on
                        ? win.channels.filter((k) => k !== key)
                        : win.channels.length >= PLOT_COLORS.length
                          ? win.channels
                          : [...win.channels, key];
                      onChange({ ...win, channels: next });
                    }}
                  />
                  {key}
                </label>
              );
            })}
          </div>
        )}

        <div className="plot-window-chart">
          <Curves t={sliced.t} series={sliced.plotted} cursorT={cursorT} />
        </div>
      </div>

      <div
        className="plot-window-resize"
        title="Drag to resize"
        onPointerDown={(e) => beginDrag("resize", e)}
        onPointerMove={onDragMove}
        onPointerUp={endDrag}
        onPointerCancel={endDrag}
      />
    </div>
  );
}

let _plotSeq = 1;

export function createPlotWindow(
  partial?: Partial<PlotWindowState>,
  stagger = 0
): PlotWindowState {
  const id = `plot_${_plotSeq++}`;
  return {
    id,
    x: 24 + stagger * 28,
    y: 88 + stagger * 28,
    w: 420,
    h: 300,
    z: stagger + 1,
    channels: ["vx_mps", "steering_cmd"],
    range: [0, 1],
    ...partial,
  };
}
