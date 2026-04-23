from __future__ import annotations

import math


def mod2pi(theta: float) -> float:
    return theta % (2.0 * math.pi)


def dubins_shortest_path(sx: float, sy: float, syaw_deg: float, gx: float, gy: float, gyaw_deg: float, rho: float):
    dx = gx - sx
    dy = gy - sy
    d = math.hypot(dx, dy) / rho if rho > 1e-9 else 0.0
    theta = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else 0.0

    syaw = math.radians(syaw_deg)
    gyaw = math.radians(gyaw_deg)
    alpha = mod2pi(syaw - theta)
    beta = mod2pi(gyaw - theta)

    def lsl(a, b, dval):
        p2 = 2 + dval * dval - 2 * math.cos(a - b) + 2 * dval * (math.sin(a) - math.sin(b))
        if p2 < 0:
            return None
        tmp = math.atan2(math.cos(b) - math.cos(a), dval + math.sin(a) - math.sin(b))
        t = mod2pi(-a + tmp)
        p = math.sqrt(p2)
        q = mod2pi(b - tmp)
        return [("L", t), ("S", p), ("L", q)]

    def rsr(a, b, dval):
        p2 = 2 + dval * dval - 2 * math.cos(a - b) + 2 * dval * (-math.sin(a) + math.sin(b))
        if p2 < 0:
            return None
        tmp = math.atan2(math.cos(a) - math.cos(b), dval - math.sin(a) + math.sin(b))
        t = mod2pi(a - tmp)
        p = math.sqrt(p2)
        q = mod2pi(-b + tmp)
        return [("R", t), ("S", p), ("R", q)]

    def lsr(a, b, dval):
        p2 = -2 + dval * dval + 2 * math.cos(a - b) + 2 * dval * (math.sin(a) + math.sin(b))
        if p2 < 0:
            return None
        p = math.sqrt(p2)
        tmp = math.atan2(-math.cos(a) - math.cos(b), dval + math.sin(a) + math.sin(b)) - math.atan2(-2.0, p)
        t = mod2pi(-a + tmp)
        q = mod2pi(-b + tmp)
        return [("L", t), ("S", p), ("R", q)]

    def rsl(a, b, dval):
        p2 = -2 + dval * dval + 2 * math.cos(a - b) - 2 * dval * (math.sin(a) + math.sin(b))
        if p2 < 0:
            return None
        p = math.sqrt(p2)
        tmp = math.atan2(math.cos(a) + math.cos(b), dval - math.sin(a) - math.sin(b)) - math.atan2(2.0, p)
        t = mod2pi(a - tmp)
        q = mod2pi(b - tmp)
        return [("R", t), ("S", p), ("L", q)]

    def rlr(a, b, dval):
        tmp = (6 - dval * dval + 2 * math.cos(a - b) + 2 * dval * (math.sin(a) - math.sin(b))) / 8.0
        if abs(tmp) > 1:
            return None
        p = mod2pi(2 * math.pi - math.acos(tmp))
        t = mod2pi(a - math.atan2(math.cos(a) - math.cos(b), dval - math.sin(a) + math.sin(b)) + p / 2.0)
        q = mod2pi(a - b - t + p)
        return [("R", t), ("L", p), ("R", q)]

    def lrl(a, b, dval):
        tmp = (6 - dval * dval + 2 * math.cos(a - b) + 2 * dval * (-math.sin(a) + math.sin(b))) / 8.0
        if abs(tmp) > 1:
            return None
        p = mod2pi(2 * math.pi - math.acos(tmp))
        t = mod2pi(-a - math.atan2(math.cos(a) - math.cos(b), dval + math.sin(a) - math.sin(b)) + p / 2.0)
        q = mod2pi(mod2pi(b) - a - t + p)
        return [("L", t), ("R", p), ("L", q)]

    candidates = [
        lsl(alpha, beta, d),
        rsr(alpha, beta, d),
        lsr(alpha, beta, d),
        rsl(alpha, beta, d),
        rlr(alpha, beta, d),
        lrl(alpha, beta, d),
    ]
    candidates = [c for c in candidates if c is not None]
    if not candidates:
        return None, math.inf

    best_path = min(candidates, key=lambda segs: sum(abs(seg_len) for _, seg_len in segs))
    best_len = sum(abs(seg_len) for _, seg_len in best_path) * rho
    return best_path, best_len
