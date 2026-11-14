from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib  # type: ignore[no-redef]


class PathsError(ValueError):
    """Invalid platform path configuration."""


@dataclass(frozen=True)
class PlatformPaths:
    repo_root: Path
    config_root: Path
    data_root: Path
    carsim_db: Path
    tracks_dir: Path
    scenarios_dir: Path
    experiments_dir: Path
    carsim_sessions_dir: Path
    sweeps_dir: Path
    toml_path: Path | None
    sources: dict[str, str]


_paths: PlatformPaths | None = None


def discover_repo_root() -> Path:
    # backend/app/platform/paths.py → parents[3] = repo root
    return Path(__file__).resolve().parents[3]


def reset_paths_for_tests() -> None:
    global _paths
    _paths = None


def _as_path(value: Path | str, *, base: Path) -> Path:
    p = Path(value)
    if not p.is_absolute():
        p = base / p
    return p.resolve()


def _load_toml(path: Path) -> dict[str, Any]:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise PathsError(f"cannot read toml {path}: {exc}") from exc
    try:
        data = tomllib.loads(raw.decode("utf-8"))
    except Exception as exc:
        raise PathsError(f"invalid toml {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise PathsError(f"invalid toml {path}: root must be a table")
    return data


def resolve_paths(
    *,
    repo_root: Path | None = None,
    config_root: Path | str | None = None,
    data_root: Path | str | None = None,
    carsim_db: Path | str | None = None,
    toml_path: Path | str | None = None,
    environ: Mapping[str, str] | None = None,
) -> PlatformPaths:
    env = environ if environ is not None else os.environ
    repo = Path(repo_root).resolve() if repo_root is not None else discover_repo_root()

    toml_file: Path | None
    if toml_path is not None:
        toml_file = Path(toml_path)
        if not toml_file.is_absolute():
            toml_file = (Path.cwd() / toml_file).resolve()
        else:
            toml_file = toml_file.resolve()
    elif env.get("DYNLIMIT_TOML"):
        toml_file = _as_path(env["DYNLIMIT_TOML"], base=Path.cwd())
    else:
        cand = repo / "dynlimit.toml"
        toml_file = cand if cand.is_file() else None

    toml_cfg: str | None = None
    toml_data: str | None = None
    toml_carsim: str | None = None
    if toml_file is not None:
        if not toml_file.is_file():
            raise PathsError(f"toml not found: {toml_file}")
        doc = _load_toml(toml_file)
        section = doc.get("paths") or {}
        if section and not isinstance(section, dict):
            raise PathsError(f"invalid toml {toml_file}: [paths] must be a table")
        if isinstance(section, dict):
            if "config_root" in section:
                toml_cfg = str(section["config_root"])
            if "data_root" in section:
                toml_data = str(section["data_root"])
            if "carsim_db" in section:
                toml_carsim = str(section["carsim_db"])

    sources: dict[str, str] = {}
    toml_base = toml_file.parent if toml_file is not None else repo

    if config_root is not None:
        cfg = _as_path(config_root, base=Path.cwd())
        sources["config_root"] = "cli"
    elif env.get("DYNLIMIT_CONFIG_ROOT"):
        cfg = _as_path(env["DYNLIMIT_CONFIG_ROOT"], base=Path.cwd())
        sources["config_root"] = "env"
    elif toml_cfg is not None:
        cfg = _as_path(toml_cfg, base=toml_base)
        sources["config_root"] = "toml"
    else:
        cfg = (repo / "configs").resolve()
        sources["config_root"] = "default"

    if data_root is not None:
        data = _as_path(data_root, base=Path.cwd())
        sources["data_root"] = "cli"
    elif env.get("DYNLIMIT_DATA_ROOT"):
        data = _as_path(env["DYNLIMIT_DATA_ROOT"], base=Path.cwd())
        sources["data_root"] = "env"
    elif toml_data is not None:
        data = _as_path(toml_data, base=toml_base)
        sources["data_root"] = "toml"
    else:
        data = (repo / "data").resolve()
        sources["data_root"] = "default"

    # Soft path: Mock users may not have a CarSim dataset on disk.
    if carsim_db is not None:
        csdb = _as_path(carsim_db, base=Path.cwd())
        sources["carsim_db"] = "cli"
    elif env.get("DYNLIMIT_CARSIM_DB"):
        csdb = _as_path(env["DYNLIMIT_CARSIM_DB"], base=Path.cwd())
        sources["carsim_db"] = "env"
    elif toml_carsim is not None:
        csdb = _as_path(toml_carsim, base=toml_base)
        sources["carsim_db"] = "toml"
    else:
        csdb = (repo / "carsim_db").resolve()
        sources["carsim_db"] = "default"

    if not cfg.is_dir():
        raise PathsError(
            f"config_root does not exist or is not a directory "
            f"({sources['config_root']}): {cfg}"
        )

    return PlatformPaths(
        repo_root=repo,
        config_root=cfg,
        data_root=data,
        carsim_db=csdb,
        tracks_dir=cfg / "tracks",
        scenarios_dir=cfg / "scenarios",
        experiments_dir=data / "experiments",
        carsim_sessions_dir=data / "carsim_sessions",
        sweeps_dir=data / "sweeps",
        toml_path=toml_file,
        sources=sources,
    )


def configure_paths(**kwargs: Any) -> PlatformPaths:
    global _paths
    p = resolve_paths(**kwargs)
    for d in (p.experiments_dir, p.carsim_sessions_dir, p.sweeps_dir):
        d.mkdir(parents=True, exist_ok=True)
    _paths = p
    return p


def get_paths() -> PlatformPaths:
    global _paths
    if _paths is None:
        _paths = resolve_paths()
        for d in (_paths.experiments_dir, _paths.carsim_sessions_dir, _paths.sweeps_dir):
            d.mkdir(parents=True, exist_ok=True)
    return _paths
