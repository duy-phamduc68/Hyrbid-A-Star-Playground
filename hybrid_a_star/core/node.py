from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class Node:
    x: float
    y: float
    yaw: float
    g: float
    f: float
    path_x: list[float]
    path_y: list[float]
    path_yaw: list[float]
    path_dir: list[int]
    direction: int
    parent: Optional["Node"]
