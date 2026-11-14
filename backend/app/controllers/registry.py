"""Controller name → factory registry."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from backend.app.controllers.constant import ConstantController
from backend.app.controllers.mppi import MppiController
from backend.app.controllers.pure_pursuit import PurePursuitController
from backend.app.controllers.stanley import StanleyController
from backend.app.planning.strategies.strategy01.controller import LatticePidController

ControllerFactory = Callable[..., Any]


def _lane_pp_factory(**kwargs: Any) -> Any:
    from backend.app.controllers.lane_pp import LanePurePursuitController

    return LanePurePursuitController(**kwargs)


def _lattice_pid_ml_factory(**kwargs: Any) -> Any:
    from backend.app.controllers.lattice_pid_ml import LatticePidMlController

    return LatticePidMlController(**kwargs)


_REGISTRY: dict[str, ControllerFactory] = {
    "constant": ConstantController,
    "pure_pursuit": PurePursuitController,
    "lane_pp": _lane_pp_factory,
    "stanley": StanleyController,
    "mppi": MppiController,
    "lattice_pid": LatticePidController,
    "lattice_pid_ml": _lattice_pid_ml_factory,
    # Deprecated alias
    "strategy01": LatticePidController,
}


def register_controller(name: str, factory: ControllerFactory) -> None:
    _REGISTRY[name] = factory


def create_controller(name: str, **kwargs: Any) -> Any:
    if name not in _REGISTRY:
        raise KeyError(f"Unknown controller '{name}'. Known: {sorted(_REGISTRY)}")
    return _REGISTRY[name](**kwargs)


def list_controllers() -> list[str]:
    return sorted(_REGISTRY)
