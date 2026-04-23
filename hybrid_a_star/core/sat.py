from __future__ import annotations

from config_loader import Config


def sat_check(poly1: list[tuple[float, float]], poly2: list[tuple[float, float]]) -> bool:
    for poly in (poly1, poly2):
        for p1 in range(len(poly)):
            p2 = (p1 + 1) % len(poly)
            normal = (poly[p2][1] - poly[p1][1], poly[p1][0] - poly[p2][0])
            min_a, max_a = float("inf"), float("-inf")
            for p in poly1:
                proj = normal[0] * p[0] + normal[1] * p[1]
                min_a, max_a = min(min_a, proj), max(max_a, proj)
            min_b, max_b = float("inf"), float("-inf")
            for p in poly2:
                proj = normal[0] * p[0] + normal[1] * p[1]
                min_b, max_b = min(min_b, proj), max(max_b, proj)
            if max_a < min_b or max_b < min_a:
                return False
    return True


def cell_poly(cfg: Config, c: int, r: int) -> list[tuple[float, float]]:
    x0 = c * cfg.grid_size
    y0 = r * cfg.grid_size
    x1 = x0 + cfg.grid_size
    y1 = y0 + cfg.grid_size
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
