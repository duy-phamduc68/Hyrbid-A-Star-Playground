from __future__ import annotations

from collections import deque
import heapq
import math

import numpy as np

from config_loader import Config


def precompute_environment(cfg: Config, scenario: dict):
    wall_obs = set(tuple(x) for x in scenario.get("grid_obs", []))
    rasterized_cont_obs = set()
    base_obs = set(wall_obs)
    cont_obs_polys = []

    for obs in scenario.get("cont_obs", []):
        hw, hl = cfg.car_w / 2.0, cfg.car_l / 2.0
        cx, cy, t = obs["x"], obs["y"], math.radians(obs["theta"])
        cos_t, sin_t = math.cos(t), math.sin(t)
        cont_obs_polys.append(
            [
                (cx + hl * cos_t - hw * sin_t, cy + hl * sin_t + hw * cos_t),
                (cx + hl * cos_t + hw * sin_t, cy + hl * sin_t - hw * cos_t),
                (cx - hl * cos_t + hw * sin_t, cy - hl * sin_t - hw * cos_t),
                (cx - hl * cos_t - hw * sin_t, cy - hl * sin_t + hw * cos_t),
            ]
        )

        min_c = max(0, int((cx - cfg.circumscribed_rad) // cfg.grid_size))
        max_c = min(cfg.cols - 1, int((cx + cfg.circumscribed_rad) // cfg.grid_size))
        min_r = max(0, int((cy - cfg.circumscribed_rad) // cfg.grid_size))
        max_r = min(cfg.rows - 1, int((cy + cfg.circumscribed_rad) // cfg.grid_size))

        for c in range(min_c, max_c + 1):
            for r in range(min_r, max_r + 1):
                px = c * cfg.grid_size + (cfg.grid_size / 2)
                py = r * cfg.grid_size + (cfg.grid_size / 2)
                dx, dy = px - cx, py - cy
                rx = dx * math.cos(-t) - dy * math.sin(-t)
                ry = dx * math.sin(-t) + dy * math.cos(-t)
                if abs(rx) <= hl and abs(ry) <= hw:
                    rasterized_cont_obs.add((c, r))
                    base_obs.add((c, r))

    dist_grid = build_distance_grid(cfg, base_obs)
    wall_dist_grid = build_distance_grid(cfg, wall_obs)
    return base_obs, wall_obs, rasterized_cont_obs, cont_obs_polys, dist_grid, wall_dist_grid


def build_distance_grid(cfg: Config, obstacles: set[tuple[int, int]]):
    grid = np.full((cfg.rows, cfg.cols), np.inf)
    queue = deque()
    for c, r in obstacles:
        if 0 <= c < cfg.cols and 0 <= r < cfg.rows:
            grid[r, c] = 0
            queue.append((c, r))

    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]
    while queue:
        c, r = queue.popleft()
        d = grid[r, c]
        for dc, dr in dirs:
            nc, nr = c + dc, r + dr
            if 0 <= nc < cfg.cols and 0 <= nr < cfg.rows:
                step = math.sqrt(dc * dc + dr * dr) * cfg.grid_size
                if d + step < grid[nr, nc]:
                    grid[nr, nc] = d + step
                    queue.append((nc, nr))

    return grid


def compute_h2d(cfg: Config, goal_rx: float, goal_ry: float, dist_grid):
    h2d = np.full((cfg.rows, cfg.cols), np.inf)
    gc, gr = int(goal_rx // cfg.grid_size), int(goal_ry // cfg.grid_size)

    if not (0 <= gc < cfg.cols and 0 <= gr < cfg.rows):
        return h2d
    if dist_grid[gr, gc] <= cfg.collision_dilation_radius:
        return h2d

    pq = [(0.0, gc, gr)]
    h2d[gr, gc] = 0.0
    dirs = [
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, 1.41),
        (-1, -1, 1.41),
        (1, -1, 1.41),
        (-1, 1, 1.41),
    ]

    while pq:
        d, c, r = heapq.heappop(pq)
        if d > h2d[r, c]:
            continue
        for dc, dr, cost in dirs:
            nc, nr = c + dc, r + dr
            if 0 <= nc < cfg.cols and 0 <= nr < cfg.rows and dist_grid[nr, nc] > cfg.collision_dilation_radius:
                nd = d + cost
                if nd < h2d[nr, nc]:
                    h2d[nr, nc] = nd
                    heapq.heappush(pq, (nd, nc, nr))

    return h2d


def compute_dilated_overlay_cells(base_obs: set[tuple[int, int]], dist_grid, dilation_radius: float):
    rows, cols = dist_grid.shape
    dilated = set()
    for r in range(rows):
        for c in range(cols):
            if (c, r) in base_obs:
                continue
            if dist_grid[r, c] <= dilation_radius:
                dilated.add((c, r))
    return dilated
