from __future__ import annotations

import math

from config_loader import Config

from .sat import cell_poly, sat_check


def center_to_rear_axle(cfg: Config, cx: float, cy: float, yaw_deg: float) -> tuple[float, float]:
    rad = math.radians(yaw_deg)
    dist = (cfg.car_l / 2.0) - cfg.rear_overhang
    rx = cx - dist * math.cos(rad)
    ry = cy - dist * math.sin(rad)
    return rx, ry


def rear_axle_to_center(cfg: Config, rx: float, ry: float, yaw_deg: float) -> tuple[float, float]:
    rad = math.radians(yaw_deg)
    dist = (cfg.car_l / 2.0) - cfg.rear_overhang
    cx = rx + dist * math.cos(rad)
    cy = ry + dist * math.sin(rad)
    return cx, cy


def get_car_corners(cfg: Config, rx: float, ry: float, yaw_deg: float) -> list[tuple[float, float]]:
    c_x, c_y = rear_axle_to_center(cfg, rx, ry, yaw_deg)
    rad = math.radians(yaw_deg)
    cos_t, sin_t = math.cos(rad), math.sin(rad)
    hw, hl = cfg.car_w / 2.0, cfg.car_l / 2.0
    return [
        (c_x + hl * cos_t - hw * sin_t, c_y + hl * sin_t + hw * cos_t),
        (c_x + hl * cos_t + hw * sin_t, c_y + hl * sin_t - hw * cos_t),
        (c_x - hl * cos_t + hw * sin_t, c_y - hl * sin_t - hw * cos_t),
        (c_x - hl * cos_t - hw * sin_t, c_y - hl * sin_t + hw * cos_t),
    ]


def collides_state(
    cfg: Config,
    rx: float,
    ry: float,
    yaw: float,
    base_obs: set[tuple[int, int]],
    cont_obs_polys: list[list[tuple[float, float]]],
) -> bool:
    corners = get_car_corners(cfg, rx, ry, yaw)

    for x, y in corners:
        if x < 0 or x > cfg.width or y < 0 or y > cfg.height:
            return True

    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    car_min_x, car_max_x = min(xs), max(xs)
    car_min_y, car_max_y = min(ys), max(ys)

    for obs_poly in cont_obs_polys:
        obs_xs = [p[0] for p in obs_poly]
        obs_ys = [p[1] for p in obs_poly]
        
        if (car_max_x < min(obs_xs) or car_min_x > max(obs_xs) or
            car_max_y < min(obs_ys) or car_min_y > max(obs_ys)):
            continue
            
        if sat_check(corners, obs_poly):
            return True

    min_c = max(0, int(car_min_x // cfg.grid_size) - 1)
    max_c = min(cfg.cols - 1, int(car_max_x // cfg.grid_size) + 1)
    min_r = max(0, int(car_min_y // cfg.grid_size) - 1)
    max_r = min(cfg.rows - 1, int(car_max_y // cfg.grid_size) + 1)

    for c in range(min_c, max_c + 1):
        for r in range(min_r, max_r + 1):
            if (c, r) in base_obs and sat_check(corners, cell_poly(cfg, c, r)):
                return True

    return False


def point_in_polygon(px: float, py: float, polygon: list[tuple[float, float]]) -> bool:
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        intersects = ((yi > py) != (yj > py)) and (
            px < (xj - xi) * (py - yi) / ((yj - yi) + 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def point_to_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    seg_len2 = vx * vx + vy * vy
    if seg_len2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / seg_len2))
    qx, qy = ax + t * vx, ay + t * vy
    return math.hypot(px - qx, py - qy)


def point_to_polygon_distance(px: float, py: float, polygon: list[tuple[float, float]]) -> float:
    if point_in_polygon(px, py, polygon):
        return 0.0
    min_d = float("inf")
    n = len(polygon)
    for i in range(n):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % n]
        min_d = min(min_d, point_to_segment_distance(px, py, ax, ay, bx, by))
    return min_d


def obstacle_proximity_penalty(
    cfg: Config,
    rx: float,
    ry: float,
    yaw: float,
    wall_dist_grid,
    cont_obs_polys: list[list[tuple[float, float]]],
) -> float:
    cx, cy = rear_axle_to_center(cfg, rx, ry, yaw)
    c, r = int(cx // cfg.grid_size), int(cy // cfg.grid_size)

    if 0 <= r < wall_dist_grid.shape[0] and 0 <= c < wall_dist_grid.shape[1]:
        wall_dist = float(wall_dist_grid[r, c])
    else:
        wall_dist = 0.0
    wall_margin = max(0.0, cfg.prox_wall_buffer - wall_dist)

    car_dist = float("inf")
    for poly in cont_obs_polys:
        car_dist = min(car_dist, point_to_polygon_distance(cx, cy, poly))
    if not math.isfinite(car_dist):
        car_dist = 1e6
    car_margin = max(0.0, cfg.prox_car_buffer - car_dist)

    return cfg.prox_penalty_weight * ((wall_margin * wall_margin) + (car_margin * car_margin))
