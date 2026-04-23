from __future__ import annotations

import math

from config_loader import Config

from .dubins import dubins_shortest_path
from .reeds_shepp import RS_AVAILABLE, generate_rs_samples, get_rs_cost


def get_analytic_cost(cfg: Config, path_model: str, nx: float, ny: float, nyaw: float, gx: float, gy: float, gyaw: float) -> float:
    model = path_model.upper()
    if model == "DUBINS":
        _, length = dubins_shortest_path(nx, ny, nyaw, gx, gy, gyaw, cfg.min_turn_rad)
        return length if math.isfinite(length) else math.hypot(gx - nx, gy - ny)
    return get_rs_cost(cfg, nx, ny, nyaw, gx, gy, gyaw)


def generate_analytic_samples(
    cfg: Config,
    path_model: str,
    start_x: float,
    start_y: float,
    start_yaw: float,
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
):
    model = path_model.upper()
    samples = []
    cx, cy, cyaw = start_x, start_y, start_yaw

    if model == "DUBINS":
        segments, total_len = dubins_shortest_path(start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw, cfg.min_turn_rad)
        if segments is None:
            return None

        for steering_name, seg_len in segments:
            dist_pixels = abs(seg_len) * cfg.min_turn_rad
            v = cfg.v_max
            steer = 0.0
            if steering_name == "L":
                steer = cfg.steer_max_deg
            elif steering_name == "R":
                steer = -cfg.steer_max_deg

            chunks = int(abs(dist_pixels) / cfg.analytic_step_px)
            chunks = max(int(cfg.analytic_min_chunks), chunks)
            micro_dt = (abs(dist_pixels) / abs(v)) / chunks if v != 0 else 0.0
            for _ in range(chunks):
                cx += v * math.cos(math.radians(cyaw)) * micro_dt
                cy += v * math.sin(math.radians(cyaw)) * micro_dt
                cyaw = (cyaw + math.degrees((v / cfg.wheel_base) * math.tan(math.radians(steer)) * micro_dt)) % cfg.angle_wrap_deg
                samples.append((cx, cy, cyaw, 1))

        return samples, total_len

    return generate_rs_samples(cfg, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw)
