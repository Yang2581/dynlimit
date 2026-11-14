"""Batch kinematic-bicycle rollout + path cost (CPU numpy + Torch CUDA/CPU)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal, Protocol, runtime_checkable

import numpy as np


@dataclass
class RolloutParams:
    dt_s: float = 0.05
    wheelbase_m: float = 2.8
    max_steer_rad: float = 0.45
    target_speed_mps: float = 16.0
    w_lat: float = 4.0
    w_yaw: float = 1.5
    w_steer: float = 0.05
    w_throttle: float = 0.02
    throttle_nominal: float = 0.35


@runtime_checkable
class RolloutBackend(Protocol):
    name: str

    def rollout_and_cost(
        self,
        *,
        x0: float,
        y0: float,
        yaw0: float,
        v0: float,
        U: np.ndarray,
        path_xy: np.ndarray,
        params: RolloutParams,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return trajs (N,T+1,2) and costs (N,) as numpy on host."""
        ...


def bicycle_rollout_numpy(
    *,
    x0: float,
    y0: float,
    yaw0: float,
    v0: float,
    U: np.ndarray,
    params: RolloutParams,
) -> np.ndarray:
    """U: (N, T, 2) steers, throttles → xy (N, T+1, 2)."""
    n, t_h, _ = U.shape
    xy = np.zeros((n, t_h + 1, 2), dtype=np.float64)
    x = np.full(n, x0, dtype=np.float64)
    y = np.full(n, y0, dtype=np.float64)
    yaw = np.full(n, yaw0, dtype=np.float64)
    v = np.full(n, max(v0, 0.5), dtype=np.float64)
    xy[:, 0, 0] = x
    xy[:, 0, 1] = y
    L = params.wheelbase_m
    dt = params.dt_s
    for k in range(t_h):
        delta = np.clip(U[:, k, 0], -params.max_steer_rad, params.max_steer_rad)
        thr = np.clip(U[:, k, 1], 0.0, 1.0)
        a = (thr - 0.3) * 4.0 + 0.5 * (params.target_speed_mps - v) * 0.15
        a = np.clip(a, -4.0, 3.0)
        v = np.clip(v + a * dt, 0.3, 40.0)
        yaw = yaw + (v / L) * np.tan(delta) * dt
        x = x + v * np.cos(yaw) * dt
        y = y + v * np.sin(yaw) * dt
        xy[:, k + 1, 0] = x
        xy[:, k + 1, 1] = y
    return xy


def path_cost_numpy(
    xy: np.ndarray,
    U: np.ndarray,
    path_xy: np.ndarray,
    params: RolloutParams,
) -> np.ndarray:
    """Vectorized lateral/heading cost vs polyline + control effort."""
    n, t1, _ = xy.shape
    if path_xy.shape[0] < 2:
        return np.sum(U[:, :, 0] ** 2, axis=1) * params.w_steer

    # heading along traj
    dx = xy[:, 1:, 0] - xy[:, :-1, 0]
    dy = xy[:, 1:, 1] - xy[:, :-1, 1]
    yaw = np.arctan2(dy, dx)  # (N, T)

    # subsample time indices on traj points 1..T
    step = max(1, (t1 - 1) // 8)
    idx = np.arange(1, t1, step)
    pts = xy[:, idx, :]  # (N, K, 2)
    yaws = yaw[:, idx - 1]  # (N, K)

    # nearest path vertex (coarse but fast/consistent)
    # path (P,2), pts (N,K,2) → d2 (N,K,P)
    dxy = pts[:, :, None, :] - path_xy[None, None, :, :]
    d2 = dxy[..., 0] ** 2 + dxy[..., 1] ** 2
    j = np.argmin(d2, axis=-1)  # (N,K)
    j = np.clip(j, 0, path_xy.shape[0] - 2)

    # gather segments
    flat = j.reshape(-1)
    a = path_xy[flat]
    b = path_xy[flat + 1]
    seg = b - a
    seg_len = np.hypot(seg[:, 0], seg[:, 1]) + 1e-12
    tang = seg / seg_len[:, None]
    normal = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    pflat = pts.reshape(-1, 2)
    rel = pflat - a
    lat = rel[:, 0] * normal[:, 0] + rel[:, 1] * normal[:, 1]
    path_yaw = np.arctan2(tang[:, 1], tang[:, 0])
    hyaw = path_yaw - yaws.reshape(-1)
    hyaw = np.arctan2(np.sin(hyaw), np.cos(hyaw))

    k_count = idx.shape[0]
    lat = lat.reshape(n, k_count)
    hyaw = hyaw.reshape(n, k_count)
    costs = params.w_lat * np.sum(lat**2, axis=1) + params.w_yaw * np.sum(hyaw**2, axis=1)
    costs = costs + params.w_steer * np.sum(U[:, :, 0] ** 2, axis=1)
    costs = costs + params.w_throttle * np.sum((U[:, :, 1] - params.throttle_nominal) ** 2, axis=1)
    return costs


class NumpyRolloutBackend:
    name = "numpy"

    def rollout_and_cost(
        self,
        *,
        x0: float,
        y0: float,
        yaw0: float,
        v0: float,
        U: np.ndarray,
        path_xy: np.ndarray,
        params: RolloutParams,
    ) -> tuple[np.ndarray, np.ndarray]:
        trajs = bicycle_rollout_numpy(
            x0=x0, y0=y0, yaw0=yaw0, v0=v0, U=U, params=params
        )
        costs = path_cost_numpy(trajs, U, path_xy, params)
        return trajs, costs


class TorchRolloutBackend:
    """Same bicycle + cost on Torch (CPU or CUDA)."""

    def __init__(self, device: str = "cpu") -> None:
        import torch

        self._torch = torch
        self.device = torch.device(device)
        self.name = f"torch:{self.device.type}"

    def rollout_and_cost(
        self,
        *,
        x0: float,
        y0: float,
        yaw0: float,
        v0: float,
        U: np.ndarray,
        path_xy: np.ndarray,
        params: RolloutParams,
    ) -> tuple[np.ndarray, np.ndarray]:
        torch = self._torch
        dev = self.device
        U_t = torch.as_tensor(U, dtype=torch.float64, device=dev)
        path = torch.as_tensor(path_xy, dtype=torch.float64, device=dev)
        n, t_h, _ = U_t.shape
        x = torch.full((n,), x0, dtype=torch.float64, device=dev)
        y = torch.full((n,), y0, dtype=torch.float64, device=dev)
        yaw = torch.full((n,), yaw0, dtype=torch.float64, device=dev)
        v = torch.full((n,), max(v0, 0.5), dtype=torch.float64, device=dev)
        xs = [x.clone()]
        ys = [y.clone()]
        L = params.wheelbase_m
        dt = params.dt_s
        for k in range(t_h):
            delta = torch.clamp(U_t[:, k, 0], -params.max_steer_rad, params.max_steer_rad)
            thr = torch.clamp(U_t[:, k, 1], 0.0, 1.0)
            a = (thr - 0.3) * 4.0 + 0.5 * (params.target_speed_mps - v) * 0.15
            a = torch.clamp(a, -4.0, 3.0)
            v = torch.clamp(v + a * dt, 0.3, 40.0)
            yaw = yaw + (v / L) * torch.tan(delta) * dt
            x = x + v * torch.cos(yaw) * dt
            y = y + v * torch.sin(yaw) * dt
            xs.append(x.clone())
            ys.append(y.clone())
        xy = torch.stack([torch.stack(xs, dim=1), torch.stack(ys, dim=1)], dim=2)  # N,T+1,2

        # cost (mirror numpy)
        if path.shape[0] < 2:
            costs = params.w_steer * torch.sum(U_t[:, :, 0] ** 2, dim=1)
        else:
            dx = xy[:, 1:, 0] - xy[:, :-1, 0]
            dy = xy[:, 1:, 1] - xy[:, :-1, 1]
            traj_yaw = torch.atan2(dy, dx)
            step = max(1, (xy.shape[1] - 1) // 8)
            idx = torch.arange(1, xy.shape[1], step, device=dev)
            pts = xy[:, idx, :]
            yaws = traj_yaw[:, idx - 1]
            dxy = pts.unsqueeze(2) - path.view(1, 1, -1, 2)
            d2 = dxy[..., 0] ** 2 + dxy[..., 1] ** 2
            j = torch.argmin(d2, dim=-1).clamp(0, path.shape[0] - 2)
            flat = j.reshape(-1)
            a = path[flat]
            b = path[flat + 1]
            seg = b - a
            seg_len = torch.hypot(seg[:, 0], seg[:, 1]) + 1e-12
            tang = seg / seg_len.unsqueeze(1)
            normal = torch.stack([-tang[:, 1], tang[:, 0]], dim=1)
            pflat = pts.reshape(-1, 2)
            rel = pflat - a
            lat = rel[:, 0] * normal[:, 0] + rel[:, 1] * normal[:, 1]
            path_yaw = torch.atan2(tang[:, 1], tang[:, 0])
            hyaw = path_yaw - yaws.reshape(-1)
            hyaw = torch.atan2(torch.sin(hyaw), torch.cos(hyaw))
            k_count = idx.numel()
            lat = lat.view(n, k_count)
            hyaw = hyaw.view(n, k_count)
            costs = params.w_lat * torch.sum(lat**2, dim=1) + params.w_yaw * torch.sum(
                hyaw**2, dim=1
            )
            costs = costs + params.w_steer * torch.sum(U_t[:, :, 0] ** 2, dim=1)
            costs = costs + params.w_throttle * torch.sum(
                (U_t[:, :, 1] - params.throttle_nominal) ** 2, dim=1
            )

        if self.device.type == "cuda":
            torch.cuda.synchronize()
        return xy.detach().cpu().numpy(), costs.detach().cpu().numpy()


BackendName = Literal["auto", "numpy", "torch_cpu", "torch_cuda"]


def list_rollout_backends() -> list[str]:
    names = ["numpy", "torch_cpu"]
    try:
        import torch

        if torch.cuda.is_available():
            names.append("torch_cuda")
    except ImportError:
        pass
    return names


def create_rollout_backend(name: BackendName | str = "auto") -> RolloutBackend:
    key = (name or "auto").lower().strip()
    if key == "auto":
        try:
            import torch

            if torch.cuda.is_available():
                return TorchRolloutBackend("cuda")
            return TorchRolloutBackend("cpu")
        except ImportError:
            return NumpyRolloutBackend()
    if key in ("numpy", "cpu"):
        return NumpyRolloutBackend()
    if key in ("torch_cpu", "torch:cpu"):
        return TorchRolloutBackend("cpu")
    if key in ("torch_cuda", "cuda", "torch:cuda"):
        import torch

        if not torch.cuda.is_available():
            raise RuntimeError("torch_cuda requested but torch.cuda.is_available() is False")
        return TorchRolloutBackend("cuda")
    raise ValueError(f"unknown rollout backend '{name}'; known={list_rollout_backends()}+auto")


def backend_info() -> dict[str, Any]:
    info: dict[str, Any] = {"available": list_rollout_backends()}
    try:
        import torch

        info["torch"] = torch.__version__
        info["cuda_available"] = bool(torch.cuda.is_available())
        if torch.cuda.is_available():
            info["cuda_device"] = torch.cuda.get_device_name(0)
    except ImportError:
        info["torch"] = None
        info["cuda_available"] = False
    return info
