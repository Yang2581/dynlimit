"""CLI entry: legacy constant session + ``dynlimit run`` one-shot."""

from __future__ import annotations

import argparse
import json
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from backend.app.controllers.registry import create_controller
from backend.app.core.engine import Engine
from backend.app.runtime.actuation import ActuationGuard
from backend.app.runtime.monitor import TimeoutMonitor
from backend.app.runtime.publisher import NullPublisher
from backend.app.scenario.simple import SimpleScenario
from backend.app.telemetry.mcap_writer import McapRecorder
from backend.app.vehicle.carsim_adapter import CarSimAdapter
from backend.app.vehicle.mock_adapter import MockDynamicsAdapter


def build_vehicle(plant: str, carsim_db: Path) -> Any:
    if plant == "mock":
        return MockDynamicsAdapter()
    if plant == "carsim":
        return CarSimAdapter(carsim_db)
    raise ValueError(f"Unknown plant '{plant}' (use mock|carsim)")


def build_engine(
    track_path: Path,
    scenario_path: Path,
    experiment_dir: Path,
    steering_cmd: float,
    throttle_cmd: float,
    plant: str,
    carsim_db: Path,
) -> Engine:
    scenario = SimpleScenario()
    scenario.load_files(track_path, scenario_path)
    vehicle = build_vehicle(plant, carsim_db)
    controller = create_controller(
        "constant",
        steering_cmd=steering_cmd,
        throttle_cmd=throttle_cmd,
        brake_cmd=0.0,
    )
    recorder = McapRecorder(experiment_dir)
    return Engine(
        vehicle=vehicle,
        controller=controller,
        scenario=scenario,
        recorder=recorder,
        actuation_guard=ActuationGuard(),
        snapshot_publisher=NullPublisher(),
        event_monitor=TimeoutMonitor(),
    )


def _cmd_legacy(args: argparse.Namespace) -> int:
    from backend.app.platform.paths import get_paths

    paths = get_paths()
    exp_id = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
    track = args.track or (paths.config_root / "oval_track.json")
    scenario = args.scenario or (paths.config_root / "demo_scenario.json")
    carsim_db = args.carsim_db or paths.carsim_db
    experiment_dir = args.out or (paths.experiments_dir / exp_id)

    engine = build_engine(
        track,
        scenario,
        experiment_dir,
        steering_cmd=args.steering,
        throttle_cmd=args.throttle,
        plant=args.plant,
        carsim_db=carsim_db,
    )
    plant_name = type(engine.vehicle).__name__
    engine.load(
        {
            "schema_version": "1.0",
            "experiment_id": exp_id,
            "plant": plant_name,
            "controller": "constant",
            "carsim_db": str(carsim_db) if args.plant == "carsim" else None,
        }
    )
    engine.start()
    status = engine.run_while_running()
    print(f"status={status}")
    print(f"plant={plant_name}")
    print(f"experiment_dir={experiment_dir}")
    engine.vehicle.close()
    return 0


def _cmd_run(args: argparse.Namespace) -> int:
    from backend.app.experiments.run_once import CarSimUnavailableError, run_experiment

    ego_ic: dict[str, Any] = {}
    if args.x is not None:
        ego_ic["x_m"] = float(args.x)
    if args.y is not None:
        ego_ic["y_m"] = float(args.y)
    if args.yaw_deg is not None:
        import math

        ego_ic["yaw_rad"] = math.radians(float(args.yaw_deg))
    if args.speed is not None:
        ego_ic["speed_mps"] = float(args.speed)

    from backend.app.platform.paths import get_paths

    paths = get_paths()
    carsim_db = args.carsim_db or paths.carsim_db
    try:
        result = run_experiment(
            plant=args.plant,
            controller=args.controller,
            scenario_id=args.scenario_id,
            track_id=args.track_id,
            display_name=args.display_name,
            duration_s=args.duration,
            record=not args.no_record,
            controller_params={},
            carsim_db=carsim_db,
            ego_ic=ego_ic or None,
        )
    except (FileNotFoundError, ValueError, CarSimUnavailableError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}), flush=True)
        return 2
    except Exception as exc:  # pragma: no cover
        print(json.dumps({"ok": False, "error": str(exc)}), flush=True)
        return 1
    print(json.dumps({"ok": True, **result}, indent=2), flush=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="DynLimit CLI")
    sub = parser.add_subparsers(dest="command")

    run_p = sub.add_parser("run", help="One-shot experiment (required --plant); prints JSON")
    run_p.add_argument(
        "--plant",
        required=True,
        choices=("mock", "carsim"),
        help="Dynamics backend (required)",
    )
    run_p.add_argument(
        "--controller",
        choices=(
            "pure_pursuit",
            "lane_pp",
            "lattice_pid",
            "lattice_pid_ml",
            "stanley",
            "mppi",
            "constant",
            "strategy01",
        ),
        default="pure_pursuit",
    )
    run_p.add_argument("--scenario-id", default="web_demo_001")
    run_p.add_argument("--track-id", default=None)
    run_p.add_argument("--display-name", default=None)
    run_p.add_argument("--duration", type=float, default=None, help="Sim time cap (s)")
    run_p.add_argument("--config-root", type=Path, default=None)
    run_p.add_argument("--data-root", type=Path, default=None)
    run_p.add_argument("--toml", type=Path, default=None)
    run_p.add_argument("--carsim-db", type=Path, default=None)
    run_p.add_argument("--no-record", action="store_true", help="Skip MCAP writing")
    run_p.add_argument("--x", type=float, default=None, help="Override ego x_m")
    run_p.add_argument("--y", type=float, default=None, help="Override ego y_m")
    run_p.add_argument(
        "--yaw-deg",
        type=float,
        default=None,
        help="Override ego yaw (degrees)",
    )
    run_p.add_argument("--speed", type=float, default=None, help="Override ego speed_mps")

    # Legacy flat args (constant controller) when no subcommand.
    parser.add_argument("--config-root", type=Path, default=None)
    parser.add_argument("--data-root", type=Path, default=None)
    parser.add_argument("--toml", type=Path, default=None)
    parser.add_argument(
        "--plant",
        choices=("mock", "carsim"),
        default="mock",
        help="(legacy) Dynamics backend",
    )
    parser.add_argument("--carsim-db", type=Path, default=None)
    parser.add_argument(
        "--track",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--scenario",
        type=Path,
        default=None,
    )
    parser.add_argument("--steering", type=float, default=0.05)
    parser.add_argument("--throttle", type=float, default=0.3)
    parser.add_argument("--out", type=Path, default=None)

    args = parser.parse_args(argv)
    from backend.app.platform.paths import configure_paths

    configure_paths(
        config_root=getattr(args, "config_root", None),
        data_root=getattr(args, "data_root", None),
        carsim_db=getattr(args, "carsim_db", None),
        toml_path=getattr(args, "toml", None),
    )
    if args.command == "run":
        return _cmd_run(args)
    return _cmd_legacy(args)


if __name__ == "__main__":
    raise SystemExit(main())
