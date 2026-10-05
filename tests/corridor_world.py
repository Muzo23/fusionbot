"""Offline corridor world for unit tests: straight corridor along +x, walls at
y = +-W/2 (optionally with gaps), single-ray proximity sensors whose mounts are
DERIVED FROM THE STEP-8a LIVE FIT (so the offline readings reproduce the live
measurement model, incl. the diagonal asymmetry). Not a validation substitute.
"""

import math

from perception.proximity_types import ProximityReading, ProximitySnapshot

HALF_WIDTH_M = 0.728712 / 2

# name: (x forward, y left, angle [rad], range)
MOUNTS = {
    "front": (0.325, 0.0, 0.0, 0.61),
    "front_left": (0.31201, 0.09104, math.radians(54.043), 0.55),
    "front_right": (0.31575, -0.08602, math.radians(-54.430), 0.55),
    "left": (0.1668, 0.18024, math.radians(90.0), 0.45),
    "right": (0.1669, -0.17954, math.radians(-90.0), 0.45),
}


class CorridorWorld:
    def __init__(self, length_m=10.0, gaps=(), end_wall_x=None):
        """gaps: iterable of (side, x0, x1) with side 'left'/'right'."""
        self.segments = []          # (y_wall, x0, x1)
        for side, yw in (("left", HALF_WIDTH_M), ("right", -HALF_WIDTH_M)):
            cuts = sorted((a, b) for s, a, b in gaps if s == side)
            x = -1.0
            for a, b in cuts:
                self.segments.append((yw, x, a))
                x = b
            self.segments.append((yw, x, length_m))
        self.end_wall_x = end_wall_x

    def ray(self, sx, sy, a, rng):
        c, s = math.cos(a), math.sin(a)
        best = math.inf
        if abs(s) > 1e-12:
            for yw, x0, x1 in self.segments:
                t = (yw - sy) / s
                if t > 1e-9 and x0 <= sx + t * c <= x1:
                    best = min(best, t)
        if self.end_wall_x is not None and abs(c) > 1e-12:
            t = (self.end_wall_x - sx) / c
            if t > 1e-9 and abs(sy + t * s) <= HALF_WIDTH_M:
                best = min(best, t)
        return best if best <= rng else None

    def snapshot(self, x, y, th):
        c, s = math.cos(th), math.sin(th)
        out = {}
        for n, (mx, my, ma, rng) in MOUNTS.items():
            d = self.ray(x + c * mx - s * my, y + s * mx + c * my, th + ma, rng)
            out[n] = ProximityReading(n, d is not None, d)
        return ProximitySnapshot(**out)
