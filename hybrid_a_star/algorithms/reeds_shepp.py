from __future__ import annotations

import math

from config_loader import Config

try:
    from .reeds_shepp_impl import get_optimal_path, path_length

    RS_AVAILABLE = True
except ImportError:
    RS_AVAILABLE = False


def get_rs_cost(cfg: Config, nx: float, ny: float, nyaw: float, gx: float, gy: float, gyaw: float) -> float:
    if not RS_AVAILABLE:
        return math.hypot(gx - nx, gy - ny)
    try:
        sx, sy = nx / cfg.min_turn_rad, ny / cfg.min_turn_rad
        gx_s, gy_s = gx / cfg.min_turn_rad, gy / cfg.min_turn_rad
        path = get_optimal_path((sx, sy, nyaw), (gx_s, gy_s, gyaw))
        return path_length(path) * cfg.min_turn_rad
    except Exception:
        return math.hypot(gx - nx, gy - ny)


def generate_rs_samples(
    cfg: Config,
    start_x: float,
    start_y: float,
    start_yaw: float,
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
):
    if not RS_AVAILABLE:
        return None

    sx, sy = start_x / cfg.min_turn_rad, start_y / cfg.min_turn_rad
    gx_s, gy_s = goal_x / cfg.min_turn_rad, goal_y / cfg.min_turn_rad

    try:
        rs_path = get_optimal_path((sx, sy, start_yaw), (gx_s, gy_s, goal_yaw))
        total_len = path_length(rs_path) * cfg.min_turn_rad
    except Exception:
        return None

    samples = []
    cx, cy, cyaw = start_x, start_y, start_yaw

    for element in rs_path:
        dist_pixels = getattr(element, "param", 0.0) * cfg.min_turn_rad
        gear = getattr(getattr(element, "gear", None), "value", 1)
        v = cfg.v_max * gear

        steering_name = getattr(getattr(element, "steering", None), "name", "STRAIGHT")
        steer = 0.0
        if steering_name == "LEFT":
            steer = cfg.steer_max_deg
        elif steering_name == "RIGHT":
            steer = -cfg.steer_max_deg

        chunks = int(abs(dist_pixels) / cfg.analytic_step_px)
        chunks = max(int(cfg.analytic_min_chunks), chunks)
        micro_dt = (abs(dist_pixels) / abs(v)) / chunks if v != 0 else 0.0
        for _ in range(chunks):
            cx += v * math.cos(math.radians(cyaw)) * micro_dt
            cy += v * math.sin(math.radians(cyaw)) * micro_dt
            cyaw = (cyaw + math.degrees((v / cfg.wheel_base) * math.tan(math.radians(steer)) * micro_dt)) % cfg.angle_wrap_deg
            samples.append((cx, cy, cyaw, 1 if gear >= 0 else -1))

    return samples, total_len
