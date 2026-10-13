"""
Dynamic environment (step 14b): obstacles that are NOT in the map the robot built while exploring.

They appear only in the goal run (after exploration), triggered by the robot's TRUE position - an
event of the environment, the robot does not know about it. The same definitions drive the
offline ray-cast world (DynamicWorld below), the live CoppeliaSim scene (sim.dynamic_scene) and
the evaluation (body clearance against the obstacles that were present).

    DYN1 crossing   a 0.15 x 0.15 m object crosses the corridor ahead of the robot at 0.20 m/s
                    (e.g. a person or another robot passing through) -> stop, wait, continue
    DYN2 door       a closed door (full corridor width) for DOOR_S seconds -> wait, continue
    DYN3 side       an object standing at one side wall (0.40 x 0.10 m) -> drive past it; the side
                    sensor sees a "wall" 10 cm closer than the map (outlier for guidance / PROX-LOC)
    DYN4 permanent  the same door, never opens -> edge blocked, turn back, plan around it

Placement: when the goal run starts, on the route the robot has just planned (its junctions mapped to
the true maze nodes they were observed at - the experimenter puts the obstacles where the robot is
going to drive; equal-length routes would otherwise miss them), mid-edge, never in a junction and
never on the first edge. DYN4 only on an edge whose removal still leaves a path to the goal.
Scenario ALL = the four on distinct route edges, DYN4 last. Without a route: the true optimal route.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import asdict, dataclass

from mazes.maze_geometry import WallMap
from mazes.maze_spec import HALF_FREE_WIDTH_M

CROSS_SIZE_M = 0.15
CROSS_SPEED_M_S = 0.20
CROSS_TRIGGER_M = 1.30          # robot this far before the crossing line -> the object starts
DOOR_THICK_M = 0.05
DOOR_S = 8.0
DOOR_TRIGGER_M = 1.20           # beyond the FRONT sensor reach (0.325 + 0.61 m) -> appears unseen
SIDE_LEN_M, SIDE_DEPTH_M = 0.40, 0.10
SIDE_TRIGGER_M = 2.00
SCENARIOS = ("DYN1", "DYN2", "DYN3", "DYN4", "ALL")


@dataclass
class DynObstacle:
    name: str                    # DYN1..DYN4
    kind: str                    # crossing / door / side / permanent
    edge: tuple[str, str]
    cx: float                    # mid-edge point
    cy: float
    ux: float                    # unit vector along the corridor
    uy: float
    trigger_m: float
    triggered_t: float | None = None

    # ------------------------------------------------------------------ geometry
    def _box(self, along: float, across: float, c_along: float, c_across: float):
        """Axis-aligned rectangle of a box given in corridor coordinates (corridors are axis-aligned)."""
        nx, ny = -self.uy, self.ux
        px = self.cx + self.ux * c_along + nx * c_across
        py = self.cy + self.uy * c_along + ny * c_across
        hx = 0.5 * (abs(self.ux) * along + abs(nx) * across)
        hy = 0.5 * (abs(self.uy) * along + abs(ny) * across)
        return (px - hx, px + hx, py - hy, py + hy)

    def rect(self, t: float):
        """(x0, x1, y0, y1) at time t, or None when not present."""
        if self.triggered_t is None or t < self.triggered_t:
            return None
        tau = t - self.triggered_t
        h = HALF_FREE_WIDTH_M
        if self.kind == "crossing":
            off = -(h + CROSS_SIZE_M) + CROSS_SPEED_M_S * tau
            if off > h + CROSS_SIZE_M:
                return None
            return self._box(CROSS_SIZE_M, CROSS_SIZE_M, 0.0, off)
        if self.kind == "door" and tau > DOOR_S:
            return None
        if self.kind in ("door", "permanent"):
            return self._box(DOOR_THICK_M, 2 * h + 0.02, 0.0, 0.0)
        if self.kind == "side":
            return self._box(SIDE_LEN_M, SIDE_DEPTH_M + 0.01, 0.0, h - 0.5 * SIDE_DEPTH_M + 0.005)
        raise ValueError(self.kind)

    def maybe_trigger(self, t: float, x: float, y: float, th: float | None = None) -> bool:
        """Trigger when the robot is in this corridor, within trigger_m and DRIVING TOWARDS the obstacle
        (heading within 30 deg; not while it is turning in the junction at the end of the corridor)."""
        if self.triggered_t is not None:
            return False
        # the robot must be IN this corridor (not in a parallel one behind the wall) within trigger_m
        dx, dy = x - self.cx, y - self.cy
        along, across = dx * self.ux + dy * self.uy, -dx * self.uy + dy * self.ux
        if th is not None and abs(along) > 1e-6:
            towards = (-dx * math.cos(th) - dy * math.sin(th)) / max(math.hypot(dx, dy), 1e-9)
            if towards < math.cos(math.radians(30.0)):
                return False
        if abs(across) <= HALF_FREE_WIDTH_M and abs(along) <= self.trigger_m:
            self.triggered_t = t
            return True
        return False

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "DynObstacle":
        d = dict(d)
        d["edge"] = tuple(d["edge"])
        return cls(**d)


# --------------------------------------------------------------------------- placement
def _connected_without(spec, a: str, b: str) -> bool:
    adj = {n: set() for n in spec.nodes_m}
    for p, q in spec.edges:
        if {p, q} == {a, b}:
            continue
        adj[p].add(q)
        adj[q].add(p)
    seen, dq = {spec.root}, deque([spec.root])
    while dq:
        u = dq.popleft()
        for v in adj[u]:
            if v not in seen:
                seen.add(v)
                dq.append(v)
    return spec.goal in seen


# DYN4: the robot must stop and turn back in a CORRIDOR, so that it meets the junction behind it with
# the normal approach (side opening first seen by the diagonal sensor): stop point (obstacle -
# FRONT sensor 0.325 - OBSTACLE_STOP 0.25) at least PERM_STOP_FROM_JUNCTION_M from the junction centre.
PERM_STOP_FROM_JUNCTION_M = 1.0
PERM_POS_FROM_START_M = PERM_STOP_FROM_JUNCTION_M + 0.325 + 0.25 + 0.5 * DOOR_THICK_M   # 1.60 m
# ... and the door must stay out of the side sensors' reach (0.45 m) at the junction at the FAR end, or
# that junction no longer shows its opening when the robot passes it later on the detour (offline M5,
# configuration B, step 16): door at least PERM_FAR_CLEAR_M from the far junction centre.
PERM_FAR_CLEAR_M = HALF_FREE_WIDTH_M + 0.45 + 0.10                                 # 0.91 m
PERM_MIN_EDGE_M = PERM_POS_FROM_START_M + PERM_FAR_CLEAR_M                         # ~2.52 m (3 lattice)
SIDE_PREFERRED_EDGE_M = 2.0                   # DYN3 on a long edge if there is one (object ends far from junctions)


def _edge_len(spec, a: str, b: str) -> float:
    (ax, ay), (bx, by) = spec.nodes_m[a], spec.nodes_m[b]
    return math.hypot(bx - ax, by - ay)


def _make(spec, name: str, kind: str, a: str, b: str) -> DynObstacle:
    """Obstacle on edge a -> b (driving direction): mid-edge, DYN4 at PERM_POS_FROM_START_M from a."""
    (ax, ay), (bx, by) = spec.nodes_m[a], spec.nodes_m[b]
    L = math.hypot(bx - ax, by - ay)
    pos = max(0.5 * L, PERM_POS_FROM_START_M) if kind == "permanent" else 0.5 * L
    trig = {"crossing": CROSS_TRIGGER_M, "door": DOOR_TRIGGER_M, "permanent": DOOR_TRIGGER_M,
            "side": SIDE_TRIGGER_M}[kind]
    ux, uy = (bx - ax) / L, (by - ay) / L
    return DynObstacle(name, kind, (a, b), ax + ux * pos, ay + uy * pos, ux, uy, trig)


def build_scenario(spec, scenario: str, route: list[str] | None = None) -> list[DynObstacle]:
    """route: true maze nodes of the planned goal route (None: the true optimal route)."""
    from navigation.mission import optimal_goal_route
    if scenario not in SCENARIOS:
        raise ValueError(f"unknown scenario {scenario}, choose from {SCENARIOS}")
    if route is None:
        route, _ = optimal_goal_route(spec)
    valid = {frozenset(e) for e in spec.edges}
    edges = [(a, b) for a, b in zip(route[:-1], route[1:]) if frozenset((a, b)) in valid]
    # never the first route edge (the robot leaves START on it) - the obstacle must be met while driving
    cand = edges[1:] if len(edges) > 1 else edges
    alt = [e for e in cand if _connected_without(spec, *e) and _edge_len(spec, *e) >= PERM_MIN_EDGE_M]
    kinds = {"DYN1": "crossing", "DYN2": "door", "DYN3": "side", "DYN4": "permanent"}
    if scenario != "ALL":
        pool = alt if scenario == "DYN4" else cand
        if not pool:
            return []
        e = pool[len(pool) // 2] if scenario != "DYN4" else pool[-1]
        return [_make(spec, scenario, kinds[scenario], *e)]
    out, used = [], set()
    perm = alt[-1] if alt else None
    if perm is not None:
        used.add(perm)
    rest = [e for e in cand if e not in used]
    long_ = [e for e in rest if _edge_len(spec, *e) >= SIDE_PREFERRED_EDGE_M]
    if long_:
        rest.remove(long_[0])
        rest.insert(0, long_[0])                  # DYN3 first, on the first long edge
    for name, e in zip(("DYN3", "DYN1", "DYN2"), rest):
        out.append(_make(spec, name, kinds[name], *e))
        used.add(e)
    if perm is not None:
        out.append(_make(spec, "DYN4", "permanent", *perm))
    return out


# --------------------------------------------------------------------------- offline world
def _ray_box(x, y, c, s, r):
    x0, x1, y0, y1 = r
    tmin, tmax = -math.inf, math.inf
    for p, d, lo, hi in ((x, c, x0, x1), (y, s, y0, y1)):
        if abs(d) < 1e-12:
            if p < lo or p > hi:
                return math.inf
            continue
        t1, t2 = (lo - p) / d, (hi - p) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
    if tmax < max(tmin, 0.0):
        return math.inf
    return tmin if tmin > 1e-9 else math.inf


class DynamicWorld:
    """WallMap + the obstacles present at the current time (same interface as WallMap for the
    offline simulator: raycast, is_free, distance_to_walls)."""

    def __init__(self, spec, obstacles: list[DynObstacle] | None = None, placer=None) -> None:
        """obstacles now, or placer() -> obstacles called once when the goal run starts."""
        self.walls = WallMap(spec)
        self.spec = spec
        self.obstacles = obstacles or []
        self.placer = placer
        self.t = 0.0
        self.rects: list = []

    def update(self, t: float, gt_x: float, gt_y: float, goal_run: bool, gt_th: float | None = None) -> list[str]:
        """Advance to time t; returns the names of obstacles triggered on this call."""
        self.t = t
        fired = []
        if goal_run and self.placer is not None:
            self.obstacles, self.placer = self.placer(), None
        if goal_run:
            for o in self.obstacles:
                if o.maybe_trigger(t, gt_x, gt_y, gt_th):
                    fired.append(o.name)
        self.rects = [r for r in (o.rect(t) for o in self.obstacles) if r is not None]
        return fired

    def raycast(self, x: float, y: float, angle: float, max_range: float = math.inf) -> float:
        best = self.walls.raycast(x, y, angle, max_range)
        if self.rects:
            c, s = math.cos(angle), math.sin(angle)
            for r in self.rects:
                best = min(best, _ray_box(x, y, c, s, r))
        return best if best <= max_range else math.inf

    def is_free(self, x: float, y: float) -> bool:
        if not self.walls.is_free(x, y):
            return False
        return not any(r[0] <= x <= r[1] and r[2] <= y <= r[3] for r in self.rects)

    def distance_to_walls(self, x: float, y: float) -> float:
        best = self.walls.distance_to_walls(x, y)
        for x0, x1, y0, y1 in self.rects:
            dx = max(x0 - x, 0.0, x - x1)
            dy = max(y0 - y, 0.0, y - y1)
            best = min(best, math.hypot(dx, dy))
        return best
