from __future__ import annotations

import math


def M(theta):
    theta = theta % (2 * math.pi)
    if theta < -math.pi:
        return theta + 2 * math.pi
    if theta >= math.pi:
        return theta - 2 * math.pi
    return theta


def R(x, y):
    r = math.sqrt(x * x + y * y)
    theta = math.atan2(y, x)
    return r, theta


def change_of_basis(p1, p2):
    theta1 = deg2rad(p1[2])
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    new_x = dx * math.cos(theta1) + dy * math.sin(theta1)
    new_y = -dx * math.sin(theta1) + dy * math.cos(theta1)
    new_theta = p2[2] - p1[2]
    return new_x, new_y, new_theta


def rad2deg(rad):
    return 180 * rad / math.pi


def deg2rad(deg):
    return math.pi * deg / 180


def sign(x):
    return 1 if x >= 0 else -1
