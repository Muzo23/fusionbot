"""
Maze wall geometry (step 9): free-space rectangles -> boundary wall segments,
ray casting and free-space queries.

Same extraction algorithm as the generator that built the CoppeliaSim scenes:
the plane is cut into a grid by all rectangle edges, every grid cell is free or
occupied, and a wall segment lies wherever a free cell borders an occupied one;
collinear touching segments are merged.

Used by the offline simulator, the virtual LiDAR model and the map metrics.
Estimators and controllers never import this module (it is ground truth).
"""

from __future__ import annotations

import math
from collections import defaultdict

from mazes.maze_spec import HALF_FREE_WIDTH_M, START_BACK_CLEARANCE_M, START_OFFSET_FROM_ROOT_M, MazeSpec

TOL = 1e-9


def free_rectangles(spec: MazeSpec) -> list[tuple[float, float, float, float, str]]:
    nodes = spec.nodes_m
    h = HALF_FREE_WIDTH_M
    rects = [(x - h, x + h, y - h, y + h, f"node:{n}") for n, (x, y) in nodes.items()]
    for a, b in spec.edges:
        (ax, ay), (bx, by) = nodes[a], nodes[b]
        if abs(ay - by) <= TOL:
            rects.append((min(ax, bx), max(ax, bx), ay - h, ay + h, f"edge:{a}-{b}"))
        elif abs(ax - bx) <= TOL:
            rects.append((ax - h, ax + h, min(ay, by), max(ay, by), f"edge:{a}-{b}"))
        else:
            raise ValueError(f"{spec.maze_id}: edge {a}-{b} is not axis-aligned")
    rx, ry = nodes[spec.root]
    sy = ry - START_OFFSET_FROM_ROOT_M
    rects.append((rx - h, rx + h, sy - START_BACK_CLEARANCE_M, ry, "start_approach"))
    return rects


def _inside(rects, x, y) -> bool:
    return any(x0 - TOL <= x <= x1 + TOL and y0 - TOL <= y <= y1 + TOL for x0, x1, y0, y1, _ in rects)


def boundary_segments(spec: MazeSpec) -> list[tuple[str, float, float, float]]:
    """Wall segments as (axis, start, end, fixed): 'V' = x fixed, spans y; 'H' = y fixed, spans x."""
    rects = free_rectangles(spec)
    xs = sorted({round(v, 12) for r in rects for v in (r[0], r[1])})
    ys = sorted({round(v, 12) for r in rects for v in (r[2], r[3])})
    free = {}
    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            free[(i, j)] = _inside(rects, 0.5 * (xs[i] + xs[i + 1]), 0.5 * (ys[j] + ys[j + 1]))
    raw = defaultdict(list)
    for (i, j), f in free.items():
        if not f:
            continue
        x0, x1, y0, y1 = xs[i], xs[i + 1], ys[j], ys[j + 1]
        if not free.get((i - 1, j), False):
            raw[("V", round(x0, 12), "L")].append((y0, y1))
        if not free.get((i + 1, j), False):
            raw[("V", round(x1, 12), "R")].append((y0, y1))
        if not free.get((i, j - 1), False):
            raw[("H", round(y0, 12), "B")].append((x0, x1))
        if not free.get((i, j + 1), False):
            raw[("H", round(y1, 12), "T")].append((x0, x1))
    segs = []
    for (axis, fixed, _), iv in raw.items():
        iv.sort()
        s, e = iv[0]
        for a, b in iv[1:]:
            if abs(a - e) <= TOL:
                e = max(e, b)
            else:
                segs.append((axis, s, e, fixed))
                s, e = a, b
        segs.append((axis, s, e, fixed))
    segs.sort(key=lambda t: (t[0], t[3], t[1]))
    return segs


class WallMap:
    def __init__(self, spec: MazeSpec) -> None:
        self.spec = spec
        self.rects = free_rectangles(spec)
        self.segments = boundary_segments(spec)
        self.v = [(f, s, e) for a, s, e, f in self.segments if a == "V"]   # x, y0, y1
        self.h = [(f, s, e) for a, s, e, f in self.segments if a == "H"]   # y, x0, x1

    def is_free(self, x: float, y: float) -> bool:
        return _inside(self.rects, x, y)

    def raycast(self, x: float, y: float, angle: float, max_range: float = math.inf) -> float:
        """Distance to the first wall along the ray, math.inf if none within max_range."""
        c, s = math.cos(angle), math.sin(angle)
        best = math.inf
        if abs(c) > 1e-12:
            for X, y0, y1 in self.v:
                t = (X - x) / c
                if TOL < t < best and y0 - TOL <= y + t * s <= y1 + TOL:
                    best = t
        if abs(s) > 1e-12:
            for Y, x0, x1 in self.h:
                t = (Y - y) / s
                if TOL < t < best and x0 - TOL <= x + t * c <= x1 + TOL:
                    best = t
        return best if best <= max_range else math.inf

    def distance_to_walls(self, x: float, y: float) -> float:
        """Euclidean distance from a point to the nearest wall segment."""
        best = math.inf
        for X, y0, y1 in self.v:
            dy = 0.0 if y0 <= y <= y1 else min(abs(y - y0), abs(y - y1))
            best = min(best, math.hypot(x - X, dy))
        for Y, x0, x1 in self.h:
            dx = 0.0 if x0 <= x <= x1 else min(abs(x - x0), abs(x - x1))
            best = min(best, math.hypot(dx, y - Y))
        return best
