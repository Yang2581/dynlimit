"""Build session-local CarSim road overlay with Constant MU_ROAD from Track friction_mu."""

from __future__ import annotations

import math
import re
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

_MU_MIN, _MU_MAX = 0.05, 2.0

# Library paths look like Roads\Friction\RdMu_….par; Live-rewritten paths are
# absolute …\RdMu_constant_….par (no "Friction" in the path). Match both.
_FRICTION_ENTER = re.compile(
    r"ENTER_PARSFILE\s+(?P<path>[^\r\n]*RdMu[^\r\n]*)\s*\n"
    r"(?P<body>.*?)"
    r"EXIT_PARSFILE\s+(?P=path)\s*",
    re.IGNORECASE | re.DOTALL,
)
# Do not consume trailing whitespace/newline — replacement must not glue the next
# line (e.g. ``.par#FullDataName``), which breaks vs_read / exports / TSTEP.
_ROAD_ENTER = re.compile(
    r"ENTER_PARSFILE\s+(?P<path>[^\r\n]*3D_Road[^\r\n]*Road_[^\r\n]*)",
    re.IGNORECASE,
)
# Flattened Straight-East (etc.) Segment Builder path datasets in Run_all.
_PATH_SEG_ENTER = re.compile(
    r"ENTER_PARSFILE\s+(?P<path>[^\r\n]*BuilderSegment[^\r\n]*RoadSeg_[^\r\n]*)\s*\n"
    r"(?P<body>.*?)"
    r"EXIT_PARSFILE\s+(?P=path)\s*",
    re.IGNORECASE | re.DOTALL,
)
_INPUT_LINE = re.compile(r"^\s*INPUT\s+(.+)$", re.IGNORECASE | re.MULTILINE)
_DATADIR_LINE = re.compile(r"^\s*DATADIR\s+(.+)$", re.IGNORECASE | re.MULTILINE)

# Cap XY table size so session Run_all stays readable / fast to parse.
_MAP_PATH_MAX_POINTS = 200


@dataclass(frozen=True)
class RoadOverlayResult:
    session_dir: Path
    simfile_path: Path
    rdmu_path: Path
    road_path: Path
    friction_mu: float
    source_road_id: str
    # CarSim procedure stop time (s) from Run_all TSTOP, if present.
    procedure_tstop_s: float | None = None
    # Scenario ego IC rewritten into Run_all.par (speed / path station).
    ic_par: dict[str, Any] = field(default_factory=dict)


def clamp_friction_mu(mu: float) -> float:
    return max(_MU_MIN, min(_MU_MAX, float(mu)))


def parse_procedure_tstop_s(run_all_text: str) -> float | None:
    """Return the last ``TSTOP`` value in a Run_all / parsfile (procedure duration)."""
    matches = list(
        re.finditer(r"(?m)^\s*TSTOP\s+([0-9]+(?:\.[0-9]+)?)\s*(?:;.*)?$", run_all_text)
    )
    if not matches:
        return None
    return float(matches[-1].group(1))


def force_external_opt_stop(run_all_text: str) -> str:
    """Force ``OPT_STOP -1`` for Python-owned ``vs_integrate_io`` stepping.

    With ``OPT_STOP 1`` and an effective one-step ``TSTOP``, CarSim can accept
    further integrate calls but freeze exports at the IC (Vx constant, Xo stuck).
    External control should run until ``vs_terminate_run``.
    """
    new_text, n = re.subn(
        r"(?m)^\s*OPT_STOP\s+\S+.*$",
        "OPT_STOP -1",
        run_all_text,
        count=0,  # replace all OPT_STOP lines
    )
    if n:
        return new_text
    new_text, n2 = re.subn(
        r"(?m)^(\s*TSTOP\s+\S+.*)$",
        "OPT_STOP -1\n\\1",
        run_all_text,
        count=1,
    )
    if n2:
        return new_text
    return run_all_text.rstrip() + "\nOPT_STOP -1\n"


# Default CarSim "Straight East" segment is only 100 m — past the end the plant
# snaps Xo/Yo back to the path start (~every 7 s at 60 km/h). Stretch for Live.
_LIVE_PATH_LENGTH_M = 10000.0


def extend_live_path_length(
    run_all_text: str, *, length_m: float = _LIVE_PATH_LENGTH_M
) -> str:
    """Lengthen Straight-East / segment-builder path and bump short ``SSTOP`` values."""

    def _seg(m: re.Match[str]) -> str:
        return f"{m.group(1)} {length_m:g}"

    text = re.sub(
        r"(?m)^(\s*SEGMENT_LENGTH(?:\(\d+,\d+\))?)\s+[0-9.]+",
        _seg,
        run_all_text,
    )

    def _sstop(m: re.Match[str]) -> str:
        prefix, val = m.group(1), float(m.group(2))
        return f"{prefix} {max(val, length_m):g}"

    text = re.sub(r"(?m)^(\s*SSTOP(?:\(\d+\))?)\s+([0-9.]+)", _sstop, text)
    # MIXTABLE straight length cells: "#straight, , 100, 100, ..."
    text = re.sub(
        r"(?m)^(#straight,\s*,\s*)([0-9.]+)(,\s*)([0-9.]+)",
        lambda m: f"{m.group(1)}{length_m:g}{m.group(3)}{length_m:g}",
        text,
    )
    return text


def cleanup_overlay(session_dir: Path) -> None:
    if session_dir.is_dir():
        shutil.rmtree(session_dir, ignore_errors=True)


_ROOT_FILE_NAME_RE = re.compile(
    r"^\s*SET_MACRO\s+\$\(ROOT_FILE_NAME\)\$\s+(\S+)\s*$",
    re.IGNORECASE | re.MULTILINE,
)


def parse_root_file_name(simfile_text: str) -> str | None:
    m = _ROOT_FILE_NAME_RE.search(simfile_text)
    if not m:
        return None
    name = m.group(1).strip().strip('"').strip("'")
    return name or None


def mirror_session_out_to_gui_results(
    session_dir: Path,
    *,
    carsim_db: Path,
) -> dict[str, Any]:
    """Copy Live session ``out/LastRun*`` into ``carsim_db/Results/<ROOT>/``.

    CarSim GUI Animate reads the library Results folder; Live redirects
    ``OUTPUT_PATH`` to the session overlay ``out/`` so the DLL does not fight
    the browser. Mirroring keeps GUI playback working after a Web/Live run.
    """
    report: dict[str, Any] = {"mirrored": False, "files": []}
    session_dir = Path(session_dir)
    out_dir = session_dir / "out"
    if not out_dir.is_dir():
        report["reason"] = "no_out_dir"
        return report

    sim_path = session_dir / "simfile.sim"
    root_name: str | None = None
    if sim_path.is_file():
        try:
            root_name = parse_root_file_name(
                sim_path.read_text(encoding="utf-8", errors="replace")
            )
        except OSError:
            root_name = None
    if not root_name:
        report["reason"] = "no_root_file_name"
        return report

    sources = sorted(
        p
        for p in out_dir.iterdir()
        if p.is_file() and p.name.startswith("LastRun") and p.stat().st_size > 0
    )
    if not sources:
        report["reason"] = "no_lastrun_files"
        return report

    dest_dir = Path(carsim_db) / "Results" / root_name
    try:
        dest_dir.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        report["reason"] = f"mkdir_failed:{exc}"
        return report

    copied: list[str] = []
    for src in sources:
        dest = dest_dir / src.name
        try:
            shutil.copy2(src, dest)
            copied.append(src.name)
        except OSError:
            continue
    # Do NOT overwrite Results/<ROOT>/Run_all.par — that file is the procedure
    # *source* for the next Live overlay (via simfile INPUT). Replacing it with
    # the session-rewritten Run_all breaks Reset (RdMu ENTER/EXIT path changes).

    report["mirrored"] = bool(copied)
    report["files"] = copied
    report["dest_dir"] = str(dest_dir)
    report["root_file_name"] = root_name
    if not copied:
        report["reason"] = "copy_failed"
    return report


def project_station_m(
    x_m: float,
    y_m: float,
    polyline_xy: list[list[float]] | list[tuple[float, float]],
) -> float:
    """Closest-point arc length (m) along a polyline; 0 if degenerate."""
    pts = [(float(p[0]), float(p[1])) for p in polyline_xy if len(p) >= 2]
    if len(pts) < 2:
        return 0.0
    best_s = 0.0
    best_d2 = float("inf")
    s_at = 0.0
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        if L2 < 1e-18:
            continue
        t = ((x_m - x0) * dx + (y_m - y0) * dy) / L2
        t = max(0.0, min(1.0, t))
        px, py = x0 + t * dx, y0 + t * dy
        d2 = (x_m - px) ** 2 + (y_m - py) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_s = s_at + t * math.sqrt(L2)
        s_at += math.sqrt(L2)
    return float(best_s)


def _set_or_insert_par_scalar(
    text: str,
    key: str,
    value: str,
    *,
    insert_after_keys: tuple[str, ...] = ("SV_VXS", "OPT_SC", "SSTART"),
) -> tuple[str, bool]:
    """Replace ``KEY value`` lines, or insert ``KEY value`` near a known anchor."""
    pat = re.compile(rf"(?m)^(\s*{re.escape(key)})(\s*=\s*|\s+)([^\s!;].*)$")
    new_text, n = pat.subn(lambda m: f"{m.group(1)} {value}", text, count=0)
    if n:
        return new_text, True
    line = f"{key} {value}"
    for anchor in insert_after_keys:
        inserted, n_ins = re.subn(
            rf"(?m)^(\s*{re.escape(anchor)}(\s*=\s*|\s+).+)$",
            lambda m: f"{m.group(1)}\n{line}",
            text,
            count=1,
        )
        if n_ins:
            return inserted, True
    return text.rstrip() + f"\n{line}\n", True


def apply_ego_ic_to_run_all(
    run_all_text: str,
    *,
    speed_mps: float | None = None,
    station_m: float | None = None,
    station_via_sstart_only: bool = False,
    free_plane: dict[str, float] | None = None,
) -> tuple[str, dict[str, Any]]:
    """Rewrite procedure IC for speed and ego start pose.

    Prefer ``free_plane={x_m,y_m,yaw_rad}`` → ``OPT_INIT_PATH=0`` + ``SV_XO/YO/YAW``
    (degrees) so Live matches the cyan Edit marker off the reference line.

    Fallback (no free plane): path station via ``SSTART`` / ``SPATH_START``.
    When ``station_via_sstart_only`` (map XY path), write vehicle station to
    top-level ``SSTART`` only — path ``SPATH_START`` stays 0.

    Returns ``(new_text, report)`` where report lists applied fields and values.
    """
    text = run_all_text
    fields: list[str] = []
    report: dict[str, Any] = {"fields": fields}

    if speed_mps is not None and float(speed_mps) >= 0.0:
        speed_kmh = float(speed_mps) * 3.6
        report["speed_mps"] = float(speed_mps)
        report["speed_kmh"] = speed_kmh
        text, ok = _set_or_insert_par_scalar(text, "SV_VXS", f"{speed_kmh:g}")
        if ok:
            fields.append("SV_VXS")

    fp = free_plane or None
    if fp is not None:
        x_m = float(fp["x_m"])
        y_m = float(fp["y_m"])
        yaw_rad = float(fp.get("yaw_rad", 0.0))
        if math.isfinite(x_m) and math.isfinite(y_m) and math.isfinite(yaw_rad):
            yaw_deg = yaw_rad * (180.0 / math.pi)
            report["ic_mode"] = "free_plane"
            report["x_m"] = x_m
            report["y_m"] = y_m
            report["yaw_rad"] = yaw_rad
            report["yaw_deg"] = yaw_deg
            # Strip any prior path-init / leftover free-plane lines, then write.
            for _key in ("OPT_INIT_PATH", "SV_XO", "SV_YO", "SV_YAW"):
                text = re.sub(rf"(?m)^\s*{_key}\s+[^\n]*\n?", "", text)
            for key, val in (
                ("OPT_INIT_PATH", "0"),
                ("SV_XO", f"{x_m:g}"),
                ("SV_YO", f"{y_m:g}"),
                ("SV_YAW", f"{yaw_deg:g}"),
            ):
                text, ok = _set_or_insert_par_scalar(text, key, val)
                if ok:
                    fields.append(key)
            if station_m is not None and math.isfinite(float(station_m)):
                report["station_m"] = max(0.0, float(station_m))
            report["applied"] = bool(fields)
            return text, report

    if station_m is not None and math.isfinite(float(station_m)):
        s = max(0.0, float(station_m))
        report["station_m"] = s
        report["ic_mode"] = "path_station"
        if not station_via_sstart_only:
            # Legacy Straight-East mode: also nudge path SPATH (historical).
            new_text, n = re.subn(
                r"(?m)^(\s*OPT_PATH_START)\s+[0-9.+\-eE]+",
                lambda m: f"{m.group(1)} 1",
                text,
            )
            if n:
                text = new_text
                fields.append("OPT_PATH_START")
            new_text, n = re.subn(
                r"(?m)^(\s*SPATH_START)\s+[0-9.+\-eE]+",
                lambda m: f"{m.group(1)} {s:g}",
                text,
            )
            if n:
                text = new_text
                fields.append("SPATH_START")
        # Top-level SSTART only (not SSTART(1)). Vehicle station on PATH_ID_DM.
        new_text, n = re.subn(
            r"(?m)^(\s*SSTART)\s+([0-9.+\-eE]+)\s*$",
            lambda m: f"{m.group(1)} {s:g}",
            text,
        )
        if n:
            text = new_text
            fields.append("SSTART")
        elif station_via_sstart_only:
            text, ok = _set_or_insert_par_scalar(text, "SSTART", f"{s:g}")
            if ok and "SSTART" not in fields:
                fields.append("SSTART")

    report["applied"] = bool(fields)
    return text, report


def downsample_polyline_xy(
    polyline_xy: list[list[float]] | list[tuple[float, float]],
    *,
    max_points: int = _MAP_PATH_MAX_POINTS,
) -> list[list[float]]:
    """Keep ends; uniform index stride so length ≤ max_points."""
    pts = [[float(p[0]), float(p[1])] for p in polyline_xy if len(p) >= 2]
    n = len(pts)
    if n <= max_points or max_points < 2:
        return pts
    # Inclusive endpoints via linspace indices.
    idxs = [round(i * (n - 1) / (max_points - 1)) for i in range(max_points)]
    out: list[list[float]] = []
    prev = -1
    for i in idxs:
        if i == prev:
            continue
        out.append(pts[i])
        prev = i
    if out[-1] != pts[-1]:
        out.append(pts[-1])
    return out


def ensure_closed_xy(
    polyline_xy: list[list[float]], *, closed: bool, tol_m: float = 1e-3
) -> list[list[float]]:
    """If closed, append first point when endpoints differ."""
    pts = [[float(p[0]), float(p[1])] for p in polyline_xy if len(p) >= 2]
    if not closed or len(pts) < 2:
        return pts
    x0, y0 = pts[0]
    x1, y1 = pts[-1]
    if abs(x0 - x1) > tol_m or abs(y0 - y1) > tol_m:
        pts.append([x0, y0])
    return pts


# Custom path / XY-table IDs (>=999) so both flattened Run_all path blocks
# update the same objects instead of creating orphans with XY_SEGMENT_ID=0.
_MAP_PATH_UID = 999


def render_map_path_par_body(
    polyline_xy: list[list[float]],
    *,
    closed: bool,
) -> str:
    """Inner parsfile body for Path: X-Y Coordinates (no PARSFILE/END wrapper).

    CarSim requires ``XY_SEGMENT_ID`` to reference the XY table; without it the
    solver stops at T=0 (``XY_SEGMENT_ID was set to 0``) and leaves Xo/Yo at 0.
    """
    pts = ensure_closed_xy(
        downsample_polyline_xy(polyline_xy), closed=bool(closed)
    )
    if len(pts) < 2:
        raise ValueError("map path needs at least 2 XY points")
    loop = 1 if closed else 0
    uid = _MAP_PATH_UID
    rows = "\n".join(f"{p[0]:.6g} {p[1]:.6g}" for p in pts)
    return (
        "#FullDataName Path: X-Y Coordinates`Map reference`Map\n"
        f"SET_IPATH_FOR_ID {uid}\n"
        f"SET_ITAB_XY_FOR_ID {uid}\n"
        "OPT_PATH_START 0\n"
        f"OPT_PATH_LOOP {loop}\n"
        "SPATH_START 0\n"
        f"PATH_ID_DM {uid}\n"
        "set_description path_id Map reference\n"
        "NSEGMENTS 1\n"
        "IPATHSEG 1\n"
        "SEGMENT_TYPE 1\n"
        f"XY_SEGMENT_ID {uid}\n"
        "SEGMENT_XY_TABLE\n"
        f"{rows}\n"
        "ENDTABLE\n"
        "Title Map reference <Map>\n"
        "LOG_ENTRY Used Dataset: Path: X-Y Coordinates; Map reference\n"
    )


def inject_map_xy_path(
    run_all_text: str,
    polyline_xy: list[list[float]] | list[tuple[float, float]],
    *,
    closed: bool = False,
    path_par_abs: Path | None = None,
) -> tuple[str, dict[str, Any]]:
    """Replace BuilderSegment path ENTER/EXIT blocks with map XY table path.

    Returns ``(new_text, report)``. On missing blocks, returns original text with
    ``applied=False``.
    """
    report: dict[str, Any] = {"applied": False, "blocks": 0}
    pts = [[float(p[0]), float(p[1])] for p in polyline_xy if len(p) >= 2]
    if len(pts) < 2:
        report["error"] = "reference_xy_too_short"
        return run_all_text, report

    body = render_map_path_par_body(pts, closed=bool(closed))
    abs_s = str(path_par_abs.resolve()) if path_par_abs is not None else "Path_map_ref.par"
    repl = f"ENTER_PARSFILE {abs_s}\n{body}EXIT_PARSFILE {abs_s}\n"

    new_text, n = _PATH_SEG_ENTER.subn(lambda _m: repl, run_all_text)
    report["blocks"] = int(n)
    if n == 0:
        report["error"] = "no_builder_segment_blocks"
        return run_all_text, report
    report["applied"] = True
    report["point_count"] = len(
        ensure_closed_xy(downsample_polyline_xy(pts), closed=bool(closed))
    )
    report["closed"] = bool(closed)
    return new_text, report


def _repo_data_sessions() -> Path:
    from backend.app.platform.paths import get_paths
    return get_paths().carsim_sessions_dir


_SET_MACRO = re.compile(
    r"^\s*SET_MACRO\s+\$\((?P<name>[^)]+)\)\$\s+(?P<value>.+)$",
    re.IGNORECASE | re.MULTILINE,
)


def _parse_simfile_macros(simfile_text: str, *, work_dir: Path) -> dict[str, str]:
    """Parse SET_MACRO lines; always provide WORK_DIR from DATADIR/simfile parent."""
    macros: dict[str, str] = {}
    for m in _SET_MACRO.finditer(simfile_text):
        macros[m.group("name").strip()] = m.group("value").strip()
    wd = str(work_dir)
    if not wd.endswith(("\\", "/")):
        wd = wd + "\\"
    # Prefer explicit WORK_DIR macro if present, else DATADIR-derived.
    macros.setdefault("WORK_DIR", wd)
    if "WORK_DIR" in macros:
        # Ensure trailing separator for CarSim-style concatenation
        w = macros["WORK_DIR"]
        if not w.endswith(("\\", "/")):
            macros["WORK_DIR"] = w + "\\"
    return macros


def _expand_macros(text: str, macros: dict[str, str]) -> str:
    t = text.strip().strip('"')
    # Expand repeatedly so nested macros in values resolve (bounded).
    for _ in range(8):
        nxt = t
        for name, value in macros.items():
            nxt = nxt.replace(f"$({name})$", value)
        if nxt == t:
            break
        t = nxt
    return t


def _read_simfile_paths(simfile: Path) -> tuple[Path, Path]:
    text = simfile.read_text(encoding="utf-8", errors="replace")
    work = simfile.parent
    m_data = _DATADIR_LINE.search(text)
    if m_data:
        # First pass: expand DATADIR with WORK_DIR = simfile parent
        boot = _parse_simfile_macros(text, work_dir=work)
        work = Path(_expand_macros(m_data.group(1), boot))
    macros = _parse_simfile_macros(text, work_dir=work)
    m_in = _INPUT_LINE.search(text)
    if not m_in:
        raise FileNotFoundError(f"No INPUT in {simfile}")
    raw = _expand_macros(m_in.group(1), macros)
    run_all = Path(raw)
    if not run_all.is_file():
        # Fallback: Results/<ROOT_FILE_NAME>/Run_all.par or newest Run_all
        root_name = macros.get("ROOT_FILE_NAME", "").strip()
        results = simfile.parent / "Results"
        if root_name:
            candidate = results / root_name / "Run_all.par"
            if candidate.is_file():
                run_all = candidate
        if not run_all.is_file() and results.is_dir():
            found = sorted(
                results.glob("**/Run_all.par"),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
            if found:
                run_all = found[0]
        if not run_all.is_file():
            raise FileNotFoundError(f"Run_all not found: {raw}")
    return work, run_all


def _pick_constant_template(friction_dir: Path) -> Path:
    pars = sorted(friction_dir.glob("RdMu_*.par"))
    if not pars:
        raise FileNotFoundError(f"No RdMu_*.par under {friction_dir}")
    for p in pars:
        t = p.read_text(encoding="utf-8", errors="replace")
        if "MU_ROAD_CONSTANT" in t and "MU_ROAD_TABLE" not in t:
            return p
    return pars[0]


def _render_constant_rdmu(template: str, mu: float, file_id: str) -> str:
    out = template
    out = re.sub(
        r"MU_ROAD_TABLE\s+\w+\s*\n.*?ENDTABLE\s*\n",
        f"MU_ROAD_CONSTANT {mu}\n",
        out,
        count=1,
        flags=re.DOTALL | re.IGNORECASE,
    )
    if re.search(r"MU_ROAD_CONSTANT\s+", out):
        out = re.sub(
            r"MU_ROAD_CONSTANT\s+[0-9.eE+-]+", f"MU_ROAD_CONSTANT {mu}", out, count=1
        )
    else:
        out = out.replace(
            "MU_ROAD_COMBINE ADD",
            f"MU_ROAD_COMBINE ADD\nMU_ROAD_CONSTANT {mu}\nMU_ROAD_L_CONSTANT 0",
            1,
        )
    out = re.sub(r"\*SCALAR\s+[0-9.eE+-]+", f"*SCALAR {mu}", out, count=1)
    out = re.sub(r"#FileID\s*:.*", f"#FileID  : {file_id}", out)
    out = re.sub(
        r"#FullDataName.*",
        f"#FullDataName Road: Friction Map, S-L Grid`Constant: {mu}`Constant",
        out,
        count=1,
    )
    return out


def _rewrite_road_parsfile(road_text: str, rdmu_abs: Path) -> str:
    abs_s = str(rdmu_abs)
    # Use callable repl — Windows paths contain \U that break re.sub string templates.
    text = re.sub(
        r"PARSFILE\s+Roads\\Friction\\RdMu_[^\s]+\.par",
        lambda _m: f"PARSFILE {abs_s}",
        road_text,
        count=1,
        flags=re.IGNORECASE,
    )
    stem = rdmu_abs.stem
    text = re.sub(
        r"(#BlueLink3[^\n]*Friction`)[^\n`]+",
        lambda m: m.group(1) + stem,
        text,
        count=1,
        flags=re.IGNORECASE,
    )
    return text


def _rewrite_run_all_friction(run_text: str, rdmu_abs: Path, rdmu_body: str) -> str:
    body_lines = [
        ln
        for ln in rdmu_body.splitlines()
        if not ln.strip().upper().startswith("PARSFILE") and ln.strip().upper() != "END"
    ]
    inner = "\n".join(body_lines) + "\n"
    abs_s = str(rdmu_abs)
    repl = f"ENTER_PARSFILE {abs_s}\n{inner}EXIT_PARSFILE {abs_s}\n"
    new_text, n = _FRICTION_ENTER.subn(lambda _m: repl, run_text, count=1)
    if n == 0:
        raise ValueError("No Friction RdMu ENTER/EXIT block found in Run_all.par")
    return new_text


def _rewrite_run_all_road_path(run_text: str, road_abs: Path) -> str:
    abs_s = str(road_abs)
    new_text, n = _ROAD_ENTER.subn(lambda _m: f"ENTER_PARSFILE {abs_s}", run_text, count=1)
    if n == 0:
        return run_text
    new_text = re.sub(
        r"EXIT_PARSFILE\s+[^\r\n]*3D_Road[^\r\n]*Road_[^\r\n]*",
        lambda _m: f"EXIT_PARSFILE {abs_s}",
        new_text,
        count=1,
        flags=re.IGNORECASE,
    )
    return new_text


def _write_session_simfile(
    *,
    original_sim: Path,
    session_dir: Path,
    datadir: Path,
    run_all: Path,
) -> Path:
    text = original_sim.read_text(encoding="utf-8", errors="replace")
    session_dir = session_dir.resolve()
    datadir = datadir.resolve()
    run_all = run_all.resolve()
    out_prefix = (session_dir / "out" / "LastRun").resolve()
    (session_dir / "out").mkdir(parents=True, exist_ok=True)
    dd = str(datadir)
    if not dd.endswith(("\\", "/")):
        dd = dd + "\\"
    text = _DATADIR_LINE.sub(lambda _m: f"DATADIR {dd}", text, count=1)
    text = re.sub(
        r"^\s*SET_MACRO\s+\$\(WORK_DIR\)\$.*$",
        lambda _m: f"SET_MACRO $(WORK_DIR)$ {dd}",
        text,
        count=1,
        flags=re.MULTILINE | re.IGNORECASE,
    )
    ra = str(run_all)
    text = _INPUT_LINE.sub(lambda _m: f"INPUT {ra}", text, count=1)
    op = str(out_prefix)
    text = re.sub(
        r"^\s*SET_MACRO\s+\$\(OUTPUT_FILE_PREFIX\)\$.*$",
        lambda _m: f"SET_MACRO $(OUTPUT_FILE_PREFIX)$ {op}",
        text,
        count=1,
        flags=re.MULTILINE | re.IGNORECASE,
    )
    text = re.sub(
        r"^\s*SET_MACRO\s+\$\(OUTPUT_PATH\)\$.*$",
        lambda _m: f"SET_MACRO $(OUTPUT_PATH)$ {session_dir / 'out'}",
        text,
        count=1,
        flags=re.MULTILINE | re.IGNORECASE,
    )
    text = re.sub(
        r"^\s*FILEBASE\s+.*$",
        lambda _m: f"FILEBASE {op}",
        text,
        count=1,
        flags=re.MULTILINE | re.IGNORECASE,
    )
    dest = session_dir / "simfile.sim"
    dest.write_text(text, encoding="utf-8")
    return dest


def build_constant_mu_overlay(
    *,
    session_id: str,
    carsim_db: Path,
    friction_mu: float,
    root: Path | None = None,
    ego_ic: dict[str, Any] | None = None,
    reference_xy: list[list[float]] | None = None,
    path_closed: bool | None = None,
) -> RoadOverlayResult:
    mu = clamp_friction_mu(friction_mu)
    carsim_db = Path(carsim_db).resolve()
    root = (Path(root) if root is not None else _repo_data_sessions()).resolve()
    session_dir = (root / session_id).resolve()
    if session_dir.exists():
        cleanup_overlay(session_dir)
    session_dir.mkdir(parents=True, exist_ok=True)

    simfile = carsim_db / "simfile.sim"
    if not simfile.is_file():
        raise FileNotFoundError(f"simfile.sim not found: {simfile}")
    _datadir, run_all_src = _read_simfile_paths(simfile)
    friction_dir = carsim_db / "Roads" / "Friction"
    template = _pick_constant_template(friction_dir)
    file_id = f"RdMu_session_{session_id}_{mu}".replace(".", "p")
    rdmu_path = session_dir / f"RdMu_constant_{str(mu).replace('.', 'p')}.par"
    rdmu_body = _render_constant_rdmu(
        template.read_text(encoding="utf-8", errors="replace"), mu, file_id
    )
    rdmu_path.write_text(rdmu_body, encoding="utf-8")

    run_src_text = run_all_src.read_text(encoding="utf-8", errors="replace")
    procedure_tstop_s = parse_procedure_tstop_s(run_src_text)
    m_road = _ROAD_ENTER.search(run_src_text)
    source_road_id = "unknown"
    road_src: Path | None = None
    if m_road:
        rel = m_road.group("path").strip()
        road_src = carsim_db.joinpath(*Path(rel.replace("\\", "/")).parts)
        if not road_src.is_file():
            road_src = Path(rel)
        if road_src.is_file():
            source_road_id = road_src.stem

    if road_src is not None and road_src.is_file():
        road_path = session_dir / f"{road_src.stem}_mu.par"
        road_path.write_text(
            _rewrite_road_parsfile(
                road_src.read_text(encoding="utf-8", errors="replace"), rdmu_path
            ),
            encoding="utf-8",
        )
    else:
        road_path = session_dir / "Road_session_mu.par"
        road_path.write_text(
            "PARSFILE\n"
            f"PARSFILE {rdmu_path}\n"
            "END\n",
            encoding="utf-8",
        )

    run_text = _rewrite_run_all_friction(run_src_text, rdmu_path, rdmu_body)
    run_text = _rewrite_run_all_road_path(run_text, road_path)
    run_text = force_external_opt_stop(run_text)

    ic_par: dict[str, Any] = {}
    map_path_applied = False
    closed = bool(path_closed) if path_closed is not None else False
    if reference_xy is not None and len(reference_xy) >= 2:
        path_par = session_dir / "Path_map_ref.par"
        try:
            body = render_map_path_par_body(
                [[float(p[0]), float(p[1])] for p in reference_xy if len(p) >= 2],
                closed=closed,
            )
            path_par.write_text("PARSFILE\n" + body + "END\n", encoding="utf-8")
            run_text, path_report = inject_map_xy_path(
                run_text,
                reference_xy,
                closed=closed,
                path_par_abs=path_par,
            )
            if path_report.get("applied"):
                map_path_applied = True
                ic_par["path_source"] = "map_reference"
                ic_par["path_points"] = path_report.get("point_count")
                ic_par["path_closed"] = closed
            else:
                ic_par["path_source"] = "straight_east"
                ic_par["path_error"] = path_report.get("error", "inject_failed")
        except Exception as exc:  # noqa: BLE001 — overlay must not kill Create
            ic_par["path_source"] = "straight_east"
            ic_par["path_error"] = f"map_path:{exc}"

    if not map_path_applied:
        run_text = extend_live_path_length(run_text)
        ic_par.setdefault("path_source", "straight_east")

    # Drop stale free-plane / path-init lines; apply_ego_ic may rewrite them.
    for _key in ("OPT_INIT_PATH", "SV_XO", "SV_YO", "SV_YAW"):
        run_text = re.sub(rf"(?m)^\s*{_key}\s+[^\n]*\n?", "", run_text)
    # Prevent VS Solver modal dialogs from blocking the Live worker thread.
    run_text, _ = _set_or_insert_par_scalar(run_text, "OPT_ERROR_DIALOG", "0")

    if ego_ic:
        speed = ego_ic.get("speed_mps")
        pose = ego_ic.get("pose") or {}
        station: float | None = None
        free_plane: dict[str, float] | None = None
        ref = reference_xy
        if isinstance(pose, dict):
            if pose.get("x_m") is not None and pose.get("y_m") is not None:
                x_m = float(pose["x_m"])
                y_m = float(pose["y_m"])
                yaw_rad = float(pose.get("yaw_rad") or 0.0)
                if ref is not None:
                    station = project_station_m(x_m, y_m, ref)
                # With a valid map path, free-plane IC matches Edit cyan marker.
                # (Earlier hang was from broken XY_SEGMENT_ID path, not free-plane.)
                if map_path_applied:
                    free_plane = {"x_m": x_m, "y_m": y_m, "yaw_rad": yaw_rad}
        run_text, ic_report = apply_ego_ic_to_run_all(
            run_text,
            speed_mps=float(speed) if speed is not None else None,
            station_m=station,
            station_via_sstart_only=map_path_applied and free_plane is None,
            free_plane=free_plane,
        )
        # Preserve path_* keys already set above.
        for k, v in ic_report.items():
            if k == "fields":
                ic_par["fields"] = v
            else:
                ic_par[k] = v

    run_dest = session_dir / "Run_all.par"
    run_dest.write_text(run_text, encoding="utf-8")
    sim_dest = _write_session_simfile(
        original_sim=simfile,
        session_dir=session_dir,
        datadir=carsim_db,
        run_all=run_dest,
    )
    return RoadOverlayResult(
        session_dir=session_dir,
        simfile_path=sim_dest,
        rdmu_path=rdmu_path,
        road_path=road_path,
        friction_mu=mu,
        source_road_id=source_road_id,
        procedure_tstop_s=procedure_tstop_s,
        ic_par=ic_par,
    )
