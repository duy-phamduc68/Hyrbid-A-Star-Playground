from __future__ import annotations

import math

from config_loader import Config

from .node import Node


def kinematic_expansion(cfg: Config, state: Node, v: float, steer_deg: float):
    x, y, yaw = state.x, state.y, state.yaw
    steer_rad = math.radians(steer_deg)
    yaw_rad = math.radians(yaw)
    px, py, pyaw = [], [], []

    substeps = max(1, int(cfg.kinematic_substeps))
    micro_dt = cfg.dt / float(substeps)
    for _ in range(substeps):
        x += v * math.cos(yaw_rad) * micro_dt
        y += v * math.sin(yaw_rad) * micro_dt
        yaw_rad += (v / cfg.wheel_base) * math.tan(steer_rad) * micro_dt
        px.append(x)
        py.append(y)
        pyaw.append(math.degrees(yaw_rad) % cfg.angle_wrap_deg)

    return px, py, pyaw
