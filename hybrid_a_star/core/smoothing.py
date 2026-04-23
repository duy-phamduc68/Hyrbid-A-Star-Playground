from __future__ import annotations

import math

import numpy as np

from config_loader import Config

from .geometry import center_to_rear_axle, collides_state


def smooth_path_gradient_descent(
    cfg: Config,
    path_cx: list[float],
    path_cy: list[float],
    path_yaw: list[float],
    path_dir: list[int],
    base_obs: set[tuple[int, int]],
    cont_obs_polys: list[list[tuple[float, float]]],
    dist_grid,
):
    n = len(path_cx)
    if n < 3:
        return path_cx, path_cy

    ox = np.array(path_cx, dtype=float)
    oy = np.array(path_cy, dtype=float)
    sx = np.array(path_cx, dtype=float)
    sy = np.array(path_cy, dtype=float)

    rows, cols = dist_grid.shape
    kappa_max = 1.0 / max(cfg.min_turn_rad, 1e-9)

    def sample_dist(px: float, py: float) -> float:
        c = int(px // cfg.grid_size)
        r = int(py // cfg.grid_size)
        if c < 1 or c >= cols - 1 or r < 1 or r >= rows - 1:
            return 0.0
        return float(dist_grid[r, c])

    def sample_dist_grad(px: float, py: float) -> tuple[float, float]:
        c = int(px // cfg.grid_size)
        r = int(py // cfg.grid_size)
        if c < 1 or c >= cols - 1 or r < 1 or r >= rows - 1:
            return 0.0, 0.0

        ddx = (dist_grid[r, c + 1] - dist_grid[r, c - 1]) / (2.0 * cfg.grid_size)
        ddy = (dist_grid[r + 1, c] - dist_grid[r - 1, c]) / (2.0 * cfg.grid_size)
        if not np.isfinite(ddx) or not np.isfinite(ddy):
            return 0.0, 0.0
        return float(ddx), float(ddy)

    for _ in range(cfg.smooth_iters):
        changed = False
        for i in range(1, n - 1):
            if path_dir[i - 1] != path_dir[i] or path_dir[i] != path_dir[i + 1]:
                continue

            gx = cfg.smooth_smoothness_weight * (sx[i - 1] + sx[i + 1] - 2.0 * sx[i])
            gy = cfg.smooth_smoothness_weight * (sy[i - 1] + sy[i + 1] - 2.0 * sy[i])

            gx += cfg.smooth_data_weight * (ox[i] - sx[i])
            gy += cfg.smooth_data_weight * (oy[i] - sy[i])

            d_obs = sample_dist(sx[i], sy[i])
            if d_obs < cfg.smooth_safety_buffer:
                grad_dx, grad_dy = sample_dist_grad(sx[i], sy[i])
                margin = cfg.smooth_safety_buffer - d_obs
                gx += cfg.smooth_obstacle_weight * margin * grad_dx
                gy += cfg.smooth_obstacle_weight * margin * grad_dy

            ax = sx[i] - sx[i - 1]
            ay = sy[i] - sy[i - 1]
            bx = sx[i + 1] - sx[i]
            by = sy[i + 1] - sy[i]
            la = math.hypot(ax, ay)
            lb = math.hypot(bx, by)
            if la > 1e-6 and lb > 1e-6:
                yaw1 = math.atan2(ay, ax)
                yaw2 = math.atan2(by, bx)
                d_yaw = (yaw2 - yaw1 + math.pi) % (2.0 * math.pi) - math.pi
                ds = 0.5 * (la + lb)
                kappa = abs(d_yaw) / max(ds, 1e-6)
                if kappa > kappa_max:
                    excess = kappa - kappa_max
                    gx += cfg.smooth_curvature_weight * excess * (0.5 * (sx[i - 1] + sx[i + 1]) - sx[i])
                    gy += cfg.smooth_curvature_weight * excess * (0.5 * (sy[i - 1] + sy[i + 1]) - sy[i])

            step_norm = math.hypot(gx, gy)
            if step_norm < 1e-9:
                continue
            if step_norm > cfg.smooth_max_step:
                scale = cfg.smooth_max_step / step_norm
                gx *= scale
                gy *= scale

            cand_x = sx[i] + gx
            cand_y = sy[i] + gy
            cand_yaw = path_yaw[i]
            cand_rx, cand_ry = center_to_rear_axle(cfg, cand_x, cand_y, cand_yaw)
            if collides_state(cfg, cand_rx, cand_ry, cand_yaw, base_obs, cont_obs_polys):
                continue

            sx[i] = cand_x
            sy[i] = cand_y
            changed = True

        if not changed:
            break

    return sx.tolist(), sy.tolist()
